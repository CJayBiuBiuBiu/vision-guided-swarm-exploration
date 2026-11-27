"""Replay initial waypoint geometry on archived scan-map snapshots."""

from collections import Counter
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

from exploration_coordinator import (
    MAX_WAYPOINT_DIST, astar_grid, bresenham_cells, waypoint_along_path,
)
from passage_information import restore_map
from rgb_passage import route_length
from route_recovery import RouteFailureMemory


def replay(formal, totals):
    for marker in sorted(formal.glob("*.complete.json")):
        run = json.loads(marker.read_text())
        raw = formal / run["directory"]
        with (raw / "decisions.jsonl").open() as stream:
            events = [json.loads(line) for line in stream]
        outcomes = {event["attempt_id"]: event["outcome"] for event in events
                    if event["event"] == "outcome"}
        pending = {}
        memory = RouteFailureMemory()
        for event in events:
            kind, robot = event["event"], event.get("robot")
            if kind == "route_recovery":
                memory.record(robot, event["stuck_xy"], event["goal_xy"], event["control_stamp"])
            if kind in ("gain_passage_decision", "gain_inspection_result"):
                pending[robot] = event
            if kind != "assigned":
                continue
            status = outcomes[event["attempt_id"]]
            totals["assignments"] += 1
            totals[f"all_outcomes_{status}"] += 1
            decision = pending.pop(robot, None)
            if (decision is None or event["selected_frontier_id"] != decision["selected"]
                    or abs(event["elapsed_sim_s"] - decision["elapsed_sim_s"]) > 0.05):
                continue
            changed = (decision["reason"] == "redirected" if decision["event"] == "gain_inspection_result"
                       else decision["selected"] != decision["preferred"])
            totals["traced_assignments"] += 1
            totals["visual_redirects"] += changed
            totals["visual_redirects_ending_stuck"] += changed and status == "stuck"
            if decision["event"] != "gain_passage_decision":
                continue
            info = next(row for row in decision["candidates"]
                        if row["frontier_id"] == event["selected_frontier_id"])
            if info["route_m"] is None:
                totals["no_logged_route"] += 1
                continue
            restored, xs, ys = restore_map(decision["map"])
            grid = restored.copy()
            resolution = decision["map"]["resolution"]

            def cell(xy):
                return (int(np.clip((xy[0] - xs[0] + resolution / 2) / resolution, 0, len(xs) - 1)),
                        int(np.clip((xy[1] - ys[0] + resolution / 2) / resolution, 0, len(ys) - 1)))

            origin = decision["origin_xy"]
            start, goal = cell(origin), cell(event["selected_xy"])
            if grid[goal[1], goal[0]] == 2:
                totals["occupied_goal_in_snapshot"] += 1
                continue
            grid[start[1], start[0]] = 1
            path = astar_grid(
                grid, start, goal, allow_unknown=True, unknown_cost=2,
                cell_penalty=memory.penalties(robot, xs, ys, decision["control_stamp"]))
            if (not path or not math.isclose(route_length(path, resolution), info["route_m"], abs_tol=1e-5)):
                totals["different_replayed_route"] += 1
                continue

            diagonal = any(before[0] != after[0] and before[1] != after[1]
                           and (grid[before[1], after[0]] == 2 or grid[after[1], before[0]] == 2)
                           for before, after in zip(path, path[1:]))
            path_xy = [(xs[x], ys[y]) for x, y in path]
            waypoint = waypoint_along_path(path_xy, origin, MAX_WAYPOINT_DIST)
            ray = list(bresenham_cells(start, cell(waypoint)))
            occupied = [(float(xs[x]), float(ys[y])) for x, y in ray[1:] if grid[y, x] == 2]
            guarded = waypoint
            if occupied:
                travelled = 0.0
                near_points = []
                for before, after in zip(path_xy, path_xy[1:]):
                    travelled += math.dist(before, after)
                    if travelled > MAX_WAYPOINT_DIST:
                        break
                    near_points.append(after)
                guarded = next((point for point in reversed(near_points) if cell(point) != start
                                and all(grid[y, x] != 2
                                        for x, y in bresenham_cells(start, cell(point)))), None)
                totals["occupied_ray_with_shorter_clear_waypoint"] += guarded is not None
                totals["occupied_ray_needing_replan"] += guarded is None
            totals["replayed_assignments"] += 1
            totals[f"replayed_outcome_{status}"] += 1
            totals["diagonal_with_occupied_flank"] += diagonal
            totals["waypoint_ray_through_occupied"] += bool(occupied)
            totals[f"occupied_ray_outcome_{status}"] += bool(occupied)
            yield dict(run=marker.stem, robot=robot, attempt=event["attempt_id"],
                       elapsed_s=event["elapsed_sim_s"], outcome=status, visual_redirect=changed,
                       route_m=info["route_m"], occupied_flank_of_diagonal=diagonal,
                       occupied_on_waypoint_ray=len(occupied), origin_x=origin[0], origin_y=origin[1],
                       waypoint_x=float(waypoint[0]), waypoint_y=float(waypoint[1]),
                       guarded_waypoint_x=None if guarded is None else float(guarded[0]),
                       guarded_waypoint_y=None if guarded is None else float(guarded[1]),
                       occupied_ray_xy=json.dumps(occupied))


if __name__ == "__main__":
    counts = Counter()
    rows = list(replay(Path(sys.argv[1]), counts))
    with Path(sys.argv[2]).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(counts, sort_keys=True))
