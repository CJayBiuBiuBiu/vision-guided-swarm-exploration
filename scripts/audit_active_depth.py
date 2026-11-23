"""Audit interruptions, replacement assignments and their outcomes."""

from collections import Counter
import math


def audit_active_depth(events, mode, scene, directory, near_test_block):
    apply = mode == "recheck"
    flags = [event["applies"] for event in events if event["event"] == "depth_active_monitor"]
    if flags != [apply]:
        raise ValueError(f"Active monitor state differs from the schedule in {directory}")

    totals = Counter()
    active = {}
    previous = {}
    interrupts = {}
    replacements = {}
    outcomes = {}
    for event in events:
        kind = event["event"]
        if kind not in ("assigned", "outcome", "depth_active_check"):
            continue
        robot = event["robot"]
        if kind == "assigned":
            attempt = event["attempt_id"]
            active[robot] = (attempt, event["selected_frontier_id"])
            previous.pop(robot, None)
            identifier = event.get("depth_interruption_id")
            if identifier is not None:
                if (identifier not in interrupts or identifier in replacements
                        or interrupts[identifier]["robot"] != robot
                        or event["selection_source"] != "depth_recheck"):
                    raise ValueError(f"Unlinked replacement assignment in {directory}")
                replacements[identifier] = attempt
            continue
        if kind == "outcome":
            outcomes[event["attempt_id"]] = event["outcome"]
            active.pop(robot, None)
            previous.pop(robot, None)
            continue

        old_attempt, frontier = active[robot]
        stamp, scan, hit = event["depth_stamp"], event["laser_stamp"], event["depth_hit"]
        prior_stamp, prior_hit, prior_reported = previous.get(robot, (-math.inf, None, False))
        if (frontier != event["frontier_id"] or stamp <= prior_stamp
                or not 0 <= event["control_stamp"] - stamp <= 0.4000001
                or scan is not None and (abs(scan - stamp) > 0.22
                                         or not 0 <= event["control_stamp"] - scan <= 0.4000001)):
            raise ValueError(f"Invalid active waypoint, order or sample timing in {directory}")
        agrees = (hit is not None and prior_hit is not None and stamp - prior_stamp <= 0.5
                  and math.dist(hit["xy"], prior_hit["xy"]) <= 0.28)
        expected = agrees and not prior_reported
        if (event["confirmed"] != expected or event["applied"] != (expected and apply)
                or (event["previous_stamp"], event["previous_xy"]) !=
                ((prior_stamp, prior_hit["xy"]) if agrees else (None, None))):
            raise ValueError(f"Confirmed detection fails reconstruction in {directory}")
        previous[robot] = (stamp, hit, agrees)
        totals["checks"] += 1
        totals["paired"] += scan is not None
        totals["hits"] += hit is not None
        totals["confirmed"] += expected
        totals["depth_only_at_box"] += (scene == "low_blocks" and expected and scan is not None
                                        and event["laser_hit"] is None and near_test_block(hit))
        identifier = event["interruption_id"]
        if event["applied"]:
            if identifier != len(interrupts) + 1:
                raise ValueError(f"Invalid active interruption ID in {directory}")
            interrupts[identifier] = dict(robot=robot, attempt=old_attempt)
        elif identifier is not None:
            raise ValueError(f"Shadow check has an interruption ID in {directory}")

    blocked = {attempt for attempt, outcome in outcomes.items() if outcome == "depth_blocked"}
    if blocked != {event["attempt"] for event in interrupts.values()} or not totals["paired"]:
        raise ValueError(f"Missing paired active checks or unmatched interruptions in {directory}")
    return dict(active_checks=totals["checks"], active_paired=totals["paired"],
                active_hits=totals["hits"], active_confirmed=totals["confirmed"],
                active_depth_only_at_box=totals["depth_only_at_box"],
                active_interrupted=len(interrupts), active_replacements=len(replacements),
                active_replacement_arrivals=sum(outcomes[attempt] == "reached"
                                               for attempt in replacements.values()))
