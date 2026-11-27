"""Replay commanded lookahead points and the scan maps used to check them."""

import numpy as np

from run_dynamic_passage_study import ARENA


def equal_point(recorded, reconstructed):
    if recorded is None or reconstructed is None:
        return recorded is None and reconstructed is None
    return np.allclose(recorded, reconstructed, rtol=0, atol=1e-7)


def audit_waypoints(run, events, directory):
    from exploration_coordinator import GRID_RES, MAX_WAYPOINT_DIST, waypoint_along_path
    from passage_information import restore_map
    from waypoint_visibility import visible_lookahead, waypoint_ray_blocked

    mode = run["waypoint_visibility_mode"]

    def require(ok, message):
        if not ok:
            raise ValueError(f"{message} in {directory}")

    require(events[0]["waypoint_visibility_mode"] == mode, "Waypoint mode differs")
    counts = dict(waypoint_checks=0, requested_blocked_waypoints=0, published_waypoints=0,
                  published_blocked_waypoints=0, shortened_waypoints=0, withheld_waypoints=0)
    for event in events:
        if event["event"] != "waypoint_visibility":
            continue
        grid, xs, ys = restore_map(event["map"])
        require(abs(event["map"]["resolution"] - GRID_RES) < 1e-7
                and np.allclose([xs[0], ys[0]], [ARENA[0] + GRID_RES / 2, ARENA[1] + GRID_RES / 2]),
                "Waypoint trace does not match the scan-map geometry")

        def cell(x, y):
            return (int(np.clip((x - ARENA[0]) / GRID_RES, 0, len(xs) - 1)),
                    int(np.clip((y - ARENA[1]) / GRID_RES, 0, len(ys) - 1)))

        pos = np.asarray(event["origin_xy"])
        path = event["path_xy"]
        if path:
            requested = waypoint_along_path(path, pos, MAX_WAYPOINT_DIST)
        else:
            vector = np.asarray(event["target_xy"]) - pos
            requested = pos + vector * min(1.0, MAX_WAYPOINT_DIST / max(np.linalg.norm(vector), 1e-9))
        visible = visible_lookahead(grid, pos, path, requested, cell, MAX_WAYPOINT_DIST)
        issued = event["published_xy"]
        require(equal_point(event["requested_xy"], requested)
                and equal_point(event["candidate_xy"], visible)
                and equal_point(issued, requested if mode == "observe" else visible),
                "Published navigation point differs from replay")
        start = cell(*pos)
        counts["waypoint_checks"] += 1
        counts["requested_blocked_waypoints"] += waypoint_ray_blocked(grid, start, cell(*requested))
        if issued is None:
            counts["withheld_waypoints"] += 1
            continue
        blocked = waypoint_ray_blocked(grid, start, cell(*issued))
        require(not blocked or mode == "observe", "Issued waypoint crosses occupied cells")
        counts["published_waypoints"] += 1
        counts["published_blocked_waypoints"] += blocked
        counts["shortened_waypoints"] += not equal_point(issued, requested)
    require(counts["waypoint_checks"] > 0, "No navigation waypoints in trace")
    return counts
