"""Associate each guarded recovery with the measured motion and return to navigation."""

from collections import Counter, defaultdict
import re


OUTCOME = re.compile(
    r"recovery outcome=(\w+) start=([\d.]+) end=([\d.]+) distance=([\d.]+)")
RECOVERY_TIMESTAMP_SKEW_S = 0.15


def recovery_outcomes(logs):
    for robot, log in logs.items():
        for match in OUTCOME.finditer(log):
            reason, start, end, distance = match.groups()
            yield dict(robot=robot, reason=reason, start_s=float(start), end_s=float(end),
                       displacement_m=float(distance))


def audit_displacement(run, events, metrics, logs, directory):
    from route_recovery import (
        EXTENDED_RECOVERY_DURATION_S, FAILED_ROUTE_RADIUS_M, GUARDED_RECOVERY_DURATION_S,
        MIN_RECOVERY_PROGRESS_M,
    )

    mode = run["recovery_motion_mode"]
    adaptive = mode == "displacement"
    limit = EXTENDED_RECOVERY_DURATION_S if adaptive else GUARDED_RECOVERY_DURATION_S

    def require(condition, message):
        if not condition:
            raise ValueError(f"{message} in {directory}")

    require(events[0]["recovery_motion_mode"] == mode and all(
        f"[{robot}] recovery motion mode={mode}" in log for robot, log in logs.items()),
        "Recovery motion configuration differs")
    pending = defaultdict(list)
    resumed = {}
    for event in events:
        if event["event"] == "route_recovery":
            pending[event["robot"]].append(event)
        elif event["event"] == "route_recovery_resume":
            resumed[(event["robot"], event["report_stamp"])] = event

    counts = Counter()
    outcomes = list(recovery_outcomes(logs))
    distance_sum = 0.0
    for outcome in outcomes:
        robot, reason = outcome["robot"], outcome["reason"]
        start, end, distance = (outcome[key] for key in ("start_s", "end_s", "displacement_m"))
        require(pending[robot], "Recovery outcome has no matching stuck report")
        report = pending[robot].pop(0)
        # The explorer and coordinator can observe adjacent /clock updates.
        require(abs(report["report_stamp"] - start) <= RECOVERY_TIMESTAMP_SKEW_S,
                "Logged recovery start differs from the stuck report")
        duration = end - start
        require(0 < duration <= limit + 0.20, "Recovery duration exceeded the run limit")
        if reason == "fixed_duration":
            require(not adaptive and duration >= limit - 1e-5, "Invalid fixed recovery outcome")
        elif reason == "cleared_region":
            require(adaptive and distance >= FAILED_ROUTE_RADIUS_M - 1e-6,
                    "Recovery ended without clearing the failed-route radius")
        elif reason == "insufficient_progress":
            require(adaptive and duration >= GUARDED_RECOVERY_DURATION_S - 1e-5
                    and distance <= MIN_RECOVERY_PROGRESS_M + 1e-6,
                    "Invalid early stop for insufficient progress")
        else:
            require(adaptive and reason == "time_limit" and duration >= limit - 1e-5,
                    "Unexpected recovery outcome")
        release = resumed.pop((robot, report["report_stamp"]), None)
        if release is not None:
            require(release["result_stamp"] >= end - RECOVERY_TIMESTAMP_SKEW_S
                    and release["pause_until"] >= end - 0.001
                    and abs(release["displacement_m"] - distance) <= 1e-5,
                    "Navigation release differs from the logged completion")
        counts[reason] += 1
        counts["outside_region"] += distance >= FAILED_ROUTE_RADIUS_M - 1e-6
        distance_sum += distance

    require(not resumed, "Navigation released with no logged recovery completion")
    for reports in pending.values():
        for report in reports:
            require(float(metrics[-1]["elapsed_sim_s"]) - report["elapsed_sim_s"] <= limit + 0.25,
                    "A completed recovery is missing its outcome")
    # A displacement published next to the horizon can reach one log before the other.
    require(abs(int(metrics[-1]["recovery_results"]) - len(outcomes)) <= 1,
            "Recovery outcome count differs from the displacement counter")
    return dict(motion_completions=len(outcomes), motion_logged_displacement_sum_m=distance_sum,
                motion_cleared_region=counts["outside_region"],
                motion_insufficient_progress=counts["insufficient_progress"],
                motion_time_limit=counts["time_limit"],
                motion_early_releases=sum(event["event"] == "route_recovery_resume" for event in events))
