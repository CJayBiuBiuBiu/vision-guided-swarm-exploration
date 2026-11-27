"""Verify tracker activation and the shared scan-based speed limit."""

from collections import Counter
import math

from run_policy_goal_retention import read_json_lines


def audit_tracking(run, logs, directory):
    from motion_safety import clearance_speed_scale
    from near_goal_tracking import NEAR_GOAL_RANGE_M, near_goal_command

    mode = run["local_tracking_mode"]
    counts = Counter()

    def require(ok, reason):
        if not ok:
            raise ValueError(f"{reason} in {directory}")

    for robot, log in logs.items():
        require(f"[{robot}] local tracking mode={mode}" in log,
                f"Tracking mode differs for {robot}")
        events = read_json_lines(directory / "local_tracking" / f"{robot}.jsonl")
        require(events, f"Missing tracking samples for {robot}")
        elapsed = -math.inf
        for event in events:
            stamp, scan_stamp = event["sim_s"], event["scan_s"]
            distance, bearing = event["distance_m"], event["bearing_rad"]
            requested, linear, angular = (event[key] for key in (
                "requested_mps", "linear_mps", "angular_rps"))
            require(all(math.isfinite(value) for value in (
                stamp, scan_stamp, distance, bearing, requested, linear, angular,
                event["min_laser_m"], event["front_laser_m"], *event["xy"]))
                and stamp >= elapsed and distance >= 0 and 0 <= requested <= 0.260001
                and abs(angular) <= 1.000001,
                f"Invalid local command or timestamp for {robot}")
            elapsed = stamp
            eligible = near_goal_command(distance, bearing, event["min_laser_m"], stamp - scan_stamp)
            expected = eligible if mode == "near_goal" else None
            require(event["source"] == ("near_goal" if expected is not None else "ddpg"),
                    f"Tracker gate differs for {robot}")
            if expected is not None:
                require(math.isclose(requested, expected[0], abs_tol=1e-7)
                        and math.isclose(angular, expected[1], abs_tol=1e-7),
                        f"Near-goal command differs for {robot}")
            guarded = requested * clearance_speed_scale(event["front_laser_m"], 0.20, 0.35)
            require(math.isclose(linear, guarded, abs_tol=1e-7),
                    f"Shared laser speed limit differs for {robot}")
            counts["tracking_samples"] += 1
            counts["eligible_samples"] += eligible is not None
            counts["near_goal_samples"] += expected is not None
            counts["short_target_samples"] += distance <= NEAR_GOAL_RANGE_M
            if distance <= NEAR_GOAL_RANGE_M and abs(bearing) >= math.pi / 3:
                counts["short_large_bearing_samples"] += 1
                counts["short_large_bearing_opposed_turns"] += bearing * angular < 0
    return {key: counts[key] for key in (
        "tracking_samples", "eligible_samples", "near_goal_samples", "short_target_samples",
        "short_large_bearing_samples", "short_large_bearing_opposed_turns")}
