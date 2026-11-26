"""Audit scoped frontier exclusions and per-robot clearance events."""

import math
import re


NEAR_COLLISION_EVENT = re.compile(
    r"near collision sim=([\d.]+) scan=([\d.]+) x=([-\d.]+) y=([-\d.]+) "
    r"clearance=([\d.]+) mode=(navigation|inspection)")


def clearance_events(logs):
    for robot, log in logs.items():
        for match in NEAR_COLLISION_EVENT.finditer(log):
            stamp, scan, x, y, clearance, mode = match.groups()
            yield dict(robot=robot, stamp_s=float(stamp), scan_s=float(scan),
                       x_m=float(x), y_m=float(y), clearance_m=float(clearance), motion=mode)


def audit_scope(run, events, metrics, logs, directory):
    from route_recovery import RouteFailureMemory

    scope = run["recovery_frontier_scope"]
    local = scope == "robot"

    def require(condition, message):
        if not condition:
            raise ValueError(f"{message} in {directory}")

    require(events[0]["recovery_frontier_scope"] == scope, "Frontier recovery scope differs")
    memory = RouteFailureMemory(exclude_failed_positions=local)
    offset = next(event["control_stamp"] - event["elapsed_sim_s"] for event in events
                  if "control_stamp" in event and "elapsed_sim_s" in event)
    last_recovery = None
    peer_retirements = 0
    rejections = 0
    for event in events:
        kind = event["event"]
        if kind == "route_recovery":
            last_recovery = event
            memory.record(event["robot"], event["stuck_xy"], event["goal_xy"], event["control_stamp"])
        elif kind == "route_peer_retirement":
            require(not local and last_recovery is not None
                    and event["reporting_robot"] == last_recovery["robot"]
                    and event["robot"] != event["reporting_robot"]
                    and event["stuck_xy"] == last_recovery["stuck_xy"]
                    and math.dist(event["stuck_xy"], event["goal_xy"]) < 1.8,
                    "Peer retirement outside shared recovery")
            peer_retirements += 1
        elif kind == "assigned":
            require(memory.available(event["robot"], event["selected_xy"],
                                     event["elapsed_sim_s"] + offset + 0.05),
                    "Assigned target violates robot failure memory")
        elif kind == "passage_continuation_decision" and event["target_xy"] is not None:
            require(memory.available(event["robot"], event["target_xy"], event["control_stamp"] + 1e-6),
                    "Continuation target violates robot failure memory")
        elif kind == "route_target_rejected":
            require(not memory.available(event["robot"], event["target_xy"],
                                         event["control_stamp"] - 1e-6),
                    "Target rejection outside robot failure memory")
            rejections += 1

    blocked_point_seconds = 0.0
    sparse_seconds = 0.0
    for row, later in zip(metrics, metrics[1:]):
        raw = int(row["raw_frontiers"])
        filtered = int(row["frontier_points_after_stuck_filter"])
        require(0 <= filtered <= raw and (not local or filtered == raw),
                "Frontier pool contradicts recovery scope")
        duration = float(later["elapsed_sim_s"]) - float(row["elapsed_sim_s"])
        blocked_point_seconds += (raw - filtered) * duration
        if raw > 0 and int(row["candidate_frontiers"]) <= 2:
            sparse_seconds += duration

    near = list(clearance_events(logs))
    threshold = float(metrics[-1]["near_collision_range_m"])
    require(all(row["clearance_m"] <= threshold + 1e-6 for row in near),
            "Logged clearance outside threshold")
    logged_nav = sum(row["motion"] == "navigation" for row in near)
    return dict(route_peer_retirements=peer_retirements, scope_target_rejections=rejections,
                blocked_frontier_point_seconds=blocked_point_seconds, sparse_frontier_seconds=sparse_seconds,
                logged_near_events=len(near), logged_near_navigation=logged_nav,
                logged_near_inspection=len(near) - logged_nav)
