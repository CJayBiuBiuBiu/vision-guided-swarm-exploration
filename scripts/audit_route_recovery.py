"""Audit reporter goal retirement, target cooldown and recovery command bounds."""

import math
import re


ROUTE_FIELDS = (
    "route_events", "route_goals_retired", "route_far_goals_retired",
    "route_target_rejections", "guarded_command_samples", "guarded_escape_samples",
    "guarded_forward_stops", "guarded_stale_stops",
)


def audit_route(run, events, logs, result, directory):
    from route_recovery import (
        RECOVERY_HEADING_MIN_RAD, RECOVERY_PAUSE_S, RECOVERY_TURN_MIN_S,
        TARGET_COOLDOWN_S, RouteFailureMemory,
    )

    mode = run["route_recovery_mode"]
    guarded = mode == "guarded"
    adaptive = run.get("recovery_motion_mode") == "displacement"
    pause_duration = RECOVERY_PAUSE_S
    if adaptive:
        from route_recovery import EXTENDED_RECOVERY_PAUSE_S, RECOVERY_RELEASE_DELAY_S

        pause_duration = EXTENDED_RECOVERY_PAUSE_S
    counts = dict.fromkeys(ROUTE_FIELDS, 0)

    def require(ok, reason):
        if not ok:
            raise ValueError(f"{reason} in {directory}")

    require(events[0]["route_recovery_mode"] == mode and all(
        f"[{robot}] route recovery mode={mode}" in log for robot, log in logs.items()),
        "Recovery configuration differs")
    offset = next(event["control_stamp"] - event["elapsed_sim_s"] for event in events
                  if "control_stamp" in event and "elapsed_sim_s" in event)
    active, pending, pauses, reports = {}, {}, {}, {}
    memory = RouteFailureMemory()
    for event in events[1:]:
        kind, robot = event["event"], event.get("robot")
        if kind == "assigned":
            now = event["elapsed_sim_s"] + offset
            if guarded:
                require(now >= pauses.get(robot, -math.inf) - 0.05,
                        "New goal assigned during recovery pause")
                require(memory.available(robot, event["selected_xy"], now + 0.05),
                        "Failed target reassigned during cooldown")
            active[robot] = (event["attempt_id"], event["selected_frontier_id"], event["selected_xy"])
        elif kind == "passage_continuation_decision" and event["target_xy"] is not None:
            attempt, fid, _ = active[robot]
            if guarded:
                require(memory.available(robot, event["target_xy"], event["control_stamp"] + 1e-6),
                        "Continuation enters failed target region")
            active[robot] = (attempt, fid, event["target_xy"])
        elif kind == "route_recovery":
            require(guarded and robot not in pending, "Unexpected recovery event")
            now = event["control_stamp"]
            require(event["report_stamp"] <= now + 1e-6
                    and abs(event["pause_until"] - now - pause_duration) < 1e-6
                    and abs(event["memory_until"] - now - TARGET_COOLDOWN_S) < 1e-6,
                    "Recovery timing differs")
            goal, fid = event["goal_xy"], event["goal_id"]
            if robot in active:
                attempt, old_id, old_goal = active[robot]
                require(goal is not None and fid == old_id and math.dist(goal, old_goal) < 1e-6
                        and abs(event["goal_distance_m"] - math.dist(event["stuck_xy"], goal)) < 1e-6,
                        "Recovery did not record the active target")
                pending[robot] = attempt
                counts["route_goals_retired"] += 1
                counts["route_far_goals_retired"] += event["goal_distance_m"] >= 1.8
            else:
                require(goal is None and fid is None, "Untraced recovery target")
            pauses[robot] = event["pause_until"]
            reports[robot] = event["report_stamp"]
            memory.record(robot, event["stuck_xy"], goal, now)
            counts["route_events"] += 1
        elif kind == "route_recovery_resume":
            now = event["control_stamp"]
            require(adaptive and robot in reports
                    and reports[robot] == event["report_stamp"]
                    and event["report_stamp"] <= event["result_stamp"] <= now < pauses[robot]
                    and event["pause_until"] == min(pauses[robot], now + RECOVERY_RELEASE_DELAY_S),
                    "Navigation resumed outside the recovery window")
            pauses[robot] = event["pause_until"]
            del reports[robot]
        elif kind == "route_target_rejected":
            require(guarded and not memory.available(
                robot, event["target_xy"], event["control_stamp"] - 1e-6),
                "Unexpected target rejection")
            counts["route_target_rejections"] += 1
        elif kind == "outcome":
            attempt, _, _ = active.pop(robot)
            require(attempt == event["attempt_id"], "Mismatched attempt outcome")
            if robot in pending:
                require(event["outcome"] == "stuck" and pending.pop(robot) == attempt,
                        "Reporting robot's failed goal was not retired")
    require(not pending and not active, "Unclosed recovery or goal")
    require(counts["route_events"] == (result["stuck_events"] if guarded else 0),
            "Recovery events differ from recorded stuck events")

    pattern = re.compile(
        r"guarded recovery phase=(\w+) front=(\S+) side=(\S+) "
        r"elapsed=([-\d.]+) heading=([-\d.]+) forward=([-\d.]+) turn=([-\d.]+)")
    for log in logs.values():
        for match in pattern.finditer(log):
            require(guarded, "Guarded commands in legacy condition")
            phase, front, side, elapsed, heading, forward, turn = match.groups()
            elapsed, heading, forward, turn = map(float, (elapsed, heading, forward, turn))
            require(-1e-6 <= forward <= 0.1501 and abs(turn) <= 0.8001,
                    "Recovery command exceeds limits")
            counts["guarded_command_samples"] += 1
            if phase == "stale":
                require(front == side == "None" and forward == turn == 0, "Motion without fresh scan")
                counts["guarded_stale_stops"] += 1
            else:
                front, side = float(front), float(side)

                def scale(clearance):
                    return min(1.0, max(0.0, (clearance - 0.20) / 0.15))

                if phase == "escape":
                    require(elapsed >= RECOVERY_TURN_MIN_S - 0.0001
                            and heading >= RECOVERY_HEADING_MIN_RAD - 0.0001
                            and front > 0.20 and turn == 0
                            and abs(forward - 0.15 * scale(front)) <= 0.0001,
                            "Forward escape lacks verified turn or clearance")
                    counts["guarded_escape_samples"] += 1
                else:
                    require(phase == "turn" and forward == 0
                            and abs(abs(turn) - 0.80 * scale(side)) <= 0.0001,
                            "Turn command differs from side clearance gate")
            counts["guarded_forward_stops"] += abs(forward) <= 1e-6
    return counts
