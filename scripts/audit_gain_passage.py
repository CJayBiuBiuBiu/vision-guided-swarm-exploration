"""Replay gain-region decisions against the recorded scan-map snapshots."""

from collections import Counter
import math

import numpy as np


ADDITIONAL_FIELDS = (
    "gain_advantage_cells", "inspections_with_missing_ticks", "missing_region_ticks",
    "region_gain_depleted", "region_unavailable", "redirects_after_missing_region",
)


def audit_gain(run, events, directory):
    from active_passage import SCAN_COOLDOWN_S, SCAN_MAX_EXTRA_ROUTE_M, angle_delta, heading
    from passage_information import (
        MIN_GAIN_CELLS, REGION_BEARING_RAD, REGION_RADIUS_M, restore_map,
        select_gain, select_inspection, unknown_corridor_cells,
    )
    from rgb_passage import MAX_EXTRA_ROUTE_M, MAX_IMAGE_AGE_S

    if (events[0]["rgb_passage_strategy"] != "gain_region"
            or events[0]["rgb_passage_geometry"] != "projected_width"):
        raise ValueError(f"Unexpected gain configuration in {directory}")

    def require(condition, reason):
        if not condition:
            raise ValueError(f"{reason} in {directory}")

    def check_stamps(event, after=None):
        stamps = event["image_stamps"]
        require(stamps == sorted(set(stamps)) and len(stamps) <= 2
                and all(0 <= event["control_stamp"] - stamp <= MAX_IMAGE_AGE_S + 1e-6
                        and (after is None or stamp >= after) for stamp in stamps),
                "Invalid camera timestamps")

    def replay_rows(event):
        grid, xs, ys = restore_map(event["map"])
        rows = event["candidates"]
        require(len({row["index"] for row in rows}) == len(rows)
                and len({row["frontier_id"] for row in rows}) == len(rows), "Duplicate candidates")
        for row in rows:
            potential = unknown_corridor_cells(grid, xs, ys, event["origin_xy"], row["xy"])
            score = row["score"]
            expected = (None if score is None else unknown_corridor_cells(
                grid, xs, ys, event["origin_xy"], row["xy"], score))
            require(potential == row["potential_cells"] and expected == row["gain_cells"]
                    and (score is None or (len(event["image_stamps"]) == 2
                         and 0 <= expected <= potential)), "Gain arithmetic mismatch")
        return rows, grid, xs, ys

    def in_region(origin, anchor, target):
        return (math.dist(anchor, target) <= REGION_RADIUS_M + 1e-6
                and abs(angle_delta(heading(origin, anchor), heading(origin, target)))
                <= REGION_BEARING_RAD + 1e-6)

    counts = Counter()
    choices, inspections, inflight, last_scan = {}, {}, {}, {}
    for event in events[1:]:
        kind, robot = event["event"], event.get("robot")
        if kind in ("gain_passage_decision", "gain_inspection_start"):
            require(robot not in choices and robot not in inspections, "Concurrent gain choice")
            check_stamps(event)
            rows, _, _, _ = replay_rows(event)
            base = rows[0]
            require(base["frontier_id"] == event["preferred"], "Preferred candidate differs")
            by_index = {row["index"]: row for row in rows}
            direct = None
            if base["score"] is not None and base["route_m"] is not None:
                direct = select_gain(rows, base["index"], base["gain_cells"],
                                     base["route_m"] + MAX_EXTRA_ROUTE_M)
            counts["passage_decisions"] += 1
            counts["fresh_frame_pairs"] += len(event["image_stamps"]) == 2
            counts["preferred_scored"] += base["score"] is not None
            counts["any_direction_scored"] += any(row["score"] is not None for row in rows)
            counts["preferred_bottleneck"] += (
                base["score"] is not None and base["score"]["bottleneck_m"] is not None)
            if kind == "gain_inspection_start":
                require(direct is None and base["score"] is not None and base["route_m"] is not None,
                        "Inspection bypassed actual evidence")
                alternative = select_inspection(rows, base["index"], base["gain_cells"],
                                                base["route_m"] + SCAN_MAX_EXTRA_ROUTE_M,
                                                event["origin_xy"], event["yaw"])
                require(alternative is not None and
                        by_index[alternative]["frontier_id"] == event["alternative"]
                        and by_index[alternative]["xy"] == event["anchor_xy"]
                        and event["control_stamp"] - last_scan.get(robot, -math.inf) >= SCAN_COOLDOWN_S,
                        "Inspection candidate or cooldown differs")
                last_scan[robot] = event["control_stamp"]
                inspections[robot] = event
                counts["inspections_started"] += 1
            else:
                expected = base if direct is None else by_index[direct]
                require(expected["frontier_id"] == event["selected"], "Direct rank differs")
                choices[robot] = (event["selected"], "direct" if direct is not None else None)
                if direct is not None:
                    counts["applied_reranks"] += 1
                    counts["would_rerank"] += 1
                    counts["gain_advantage_cells"] += expected["gain_cells"] - base["gain_cells"]
        elif kind == "gain_inspection_result":
            start = inspections.pop(robot)
            check_stamps(event, start["control_stamp"])
            rows, grid, xs, ys = replay_rows(event)
            base = start["candidates"][0]
            duration = event["control_stamp"] - start["control_stamp"]
            require(0 <= duration <= 5.0 and abs(event["duration_s"] - duration) <= 1e-6
                    and event["original_origin_xy"] == start["origin_xy"]
                    and event["anchor_xy"] == start["anchor_xy"]
                    and event["preferred_anchor_xy"] == base["xy"]
                    and event["base_score"] == base["score"]
                    and event["base_route_m"] == base["route_m"]
                    and (event["preferred"], event["alternative"]) == (start["preferred"], start["alternative"])
                    and 0 <= event["missing_ticks"] <= event["ticks"], "Inspection anchor or timing differs")
            origin = event["original_origin_xy"]
            base_gain = unknown_corridor_cells(
                grid, xs, ys, origin, base["xy"], base["score"])
            potential = unknown_corridor_cells(grid, xs, ys, event["origin_xy"], event["anchor_xy"])
            require((base_gain, potential) == (event["base_gain_cells"], event["anchor_potential_cells"]),
                    "Inspected gain differs from current map")
            require(all(in_region(origin, event["anchor_xy"], row["xy"]) for row in rows),
                    "Candidate lies outside the fixed approach region")
            if event["preferred_resolved_xy"] is not None:
                require(in_region(origin, base["xy"], event["preferred_resolved_xy"]),
                        "Fallback lies outside the preferred region")
            preferred_index = next((row["index"] for row in rows
                                    if row["frontier_id"] == event["preferred_resolved"]), None)
            selected = select_gain(rows, preferred_index, base_gain, base["route_m"] + SCAN_MAX_EXTRA_ROUTE_M)
            expected = next((row for row in rows if row["index"] == selected), None)
            counts["inspections_finished"] += 1
            counts["inspections_scored"] += event["anchor_score"] is not None
            counts["inspections_with_missing_ticks"] += event["missing_ticks"] > 0
            counts["missing_region_ticks"] += event["missing_ticks"]
            counts["inspection_rebound"] += (
                event["selected"] not in (None, event["alternative"], event["preferred"]))
            if event["reason"] in ("region_gain_depleted", "region_unavailable"):
                counts[event["reason"]] += 1
            if expected is not None:
                require(event["reason"] == "redirected" and expected["frontier_id"] == event["selected"]
                        and event["anchor_score"] is not None and len(event["image_stamps"]) == 2,
                        "Rediscovered region rank differs")
                counts["inspection_redirects"] += 1
                counts["redirects_after_missing_region"] += event["missing_ticks"] > 0
                counts["gain_advantage_cells"] += expected["gain_cells"] - base_gain
            else:
                require(event["reason"] != "redirected" and (
                    event["selected"] is None or (event["selected"] == event["preferred_resolved"]
                                                  and event["fallback_route_m"] is not None)),
                        "Unexpected fallback selection")
            if event["selected"] is not None:
                choices[robot] = (event["selected"], "inspection" if expected is not None else None)
        elif kind == "passage_inspection_cancel":
            inspections.pop(robot)
            counts["inspections_cancelled"] += 1
        elif kind == "assigned":
            chosen, source = choices.pop(robot)
            require(chosen == event["selected_frontier_id"], "Issued goal differs from gain choice")
            inflight[robot] = (event["attempt_id"], source)
        elif kind == "outcome":
            attempt, source = inflight.pop(robot)
            require(attempt == event["attempt_id"], "Attempt differs")
            if event["outcome"] == "reached":
                counts["reranked_targets_reached"] += source == "direct"
                counts["inspection_redirects_reached"] += source == "inspection"
        elif kind == "passage_continuation_decision":
            require(robot in inflight and 0 <= event["step"] <= 2, "Unexpected continuation")
            check_stamps(event)
            counts["continuation_checks"] += 1
            counts["continuation_segments_arrived"] += event["step"] > 0
            if event["target_xy"] is not None:
                length = math.dist(event["origin_xy"], event["target_xy"])
                require(event["step"] < 2 and event["gain_cells"] >= MIN_GAIN_CELLS
                        and event["score"] is not None and len(event["image_stamps"]) == 2
                        and 0.65 < length <= min(1.2, event["score"]["clear_to_m"] - 0.25) + 1e-6,
                        "Unbounded or unsupported continuation")
                counts["continuations_issued"] += 1
        elif kind == "passage_continuation_wait":
            require(robot in inflight and (not event["keep"] or (
                0 <= event["age_s"] < 4.5 and event["clearance_m"] > 0.25)), "Invalid continuation hold")
            counts["continuation_waits_kept"] += event["keep"]
    require(not choices and not inflight and counts["fresh_frame_pairs"] > 0,
            "Incomplete gain trace or unavailable camera evidence")
    counts["inspections_open_at_horizon"] = len(inspections)
    return {key: counts[key] for key in (
        "passage_decisions", "fresh_frame_pairs", "preferred_scored", "any_direction_scored",
        "preferred_bottleneck", "would_rerank", "applied_reranks", "reranked_targets_reached",
        "inspections_started", "inspections_finished", "inspections_scored", "inspection_rebound",
        "inspection_redirects", "inspection_redirects_reached", "inspections_cancelled",
        "inspections_open_at_horizon", "continuation_checks", "continuations_issued",
        "continuation_segments_arrived", "continuation_waits_kept", *ADDITIONAL_FIELDS,
    )}
