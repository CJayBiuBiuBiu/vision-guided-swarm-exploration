from __future__ import annotations

import numpy as np

from eavf_mappo.planning.astar import bfs_distance_field


DEFAULT_WEIGHTS = {
    "alpha": 1.0,
    "beta": 0.5,
    "gamma": 0.8,
    "delta": 0.6,
    "eta": 0.4,
    "lam": 1.2,
}


class ExecutionAwareVoronoi:
    def __init__(self, weights: dict | None, normalize: str = "minmax", grid=None):
        self.weights = {**DEFAULT_WEIGHTS, **(weights or {})}
        self.normalize = normalize
        self.grid = grid

    def compute_cost_matrix(self, robots, tokens_per_robot, grid=None) -> np.ndarray:
        grid = self.grid if grid is None else grid
        n = len(robots)
        k = max((len(tokens) for tokens in tokens_per_robot), default=0)
        raw = np.full((n, k, 6), np.nan, dtype=np.float32)
        invalid_mask = np.ones((n, k), dtype=bool)
        unreachable_mask = np.zeros((n, k), dtype=bool)

        for i, robot in enumerate(robots):
            dist_field = None
            if grid is not None:
                start_rc = grid.try_world_to_grid(robot.x, robot.y)
                if start_rc is not None:
                    dist_field = bfs_distance_field(grid, start_rc)

            for f, tok in enumerate(tokens_per_robot[i]):
                if not tok.valid:
                    continue
                invalid_mask[i, f] = False
                distance = self._distance_for_token(grid, dist_field, tok)
                if not np.isfinite(distance):
                    unreachable_mask[i, f] = True
                raw[i, f] = [
                    distance,
                    tok.obstacle_risk,
                    tok.collision_risk,
                    tok.overlap_risk,
                    tok.comm_uncertainty,
                    tok.information_gain,
                ]

        norm = normalize_per_column(raw, method=self.normalize)
        w = self.weights
        cost = (
            w["alpha"] * norm[..., 0]
            + w["beta"] * norm[..., 1]
            + w["gamma"] * norm[..., 2]
            + w["delta"] * norm[..., 3]
            + w["eta"] * norm[..., 4]
            - w["lam"] * norm[..., 5]
        )
        cost[invalid_mask | unreachable_mask] = np.inf
        return cost.astype(np.float32)

    def assign(self, cost_matrix) -> dict[int, int]:
        cost = np.asarray(cost_matrix, dtype=float)
        n, k = cost.shape
        assignment = {i: -1 for i in range(n)}
        used_robots: set[int] = set()
        used_frontiers: set[int] = set()

        candidates = [
            (cost[i, f], i, f)
            for i in range(n)
            for f in range(k)
            if np.isfinite(cost[i, f])
        ]
        candidates.sort(key=lambda item: item[0])
        for _, robot_idx, frontier_idx in candidates:
            if robot_idx in used_robots or frontier_idx in used_frontiers:
                continue
            assignment[robot_idx] = frontier_idx
            used_robots.add(robot_idx)
            used_frontiers.add(frontier_idx)
        return assignment

    def assign_unique_frontiers(self, cost_matrix, tokens_per_robot) -> dict[int, int]:
        cost = np.asarray(cost_matrix, dtype=float)
        n, k = cost.shape
        assignment = {i: -1 for i in range(n)}
        used_robots: set[int] = set()
        used_frontier_ids: set[int] = set()

        candidates = []
        for robot_idx in range(n):
            for frontier_idx in range(k):
                if frontier_idx >= len(tokens_per_robot[robot_idx]):
                    continue
                token = tokens_per_robot[robot_idx][frontier_idx]
                frontier_id = int(getattr(token, "frontier_id", -1))
                if frontier_id < 0 or not getattr(token, "valid", False):
                    continue
                if np.isfinite(cost[robot_idx, frontier_idx]):
                    candidates.append((cost[robot_idx, frontier_idx], robot_idx, frontier_idx, frontier_id))

        candidates.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
        for _, robot_idx, frontier_idx, frontier_id in candidates:
            if robot_idx in used_robots or frontier_id in used_frontier_ids:
                continue
            assignment[robot_idx] = frontier_idx
            used_robots.add(robot_idx)
            used_frontier_ids.add(frontier_id)
        return assignment

    def _distance_for_token(self, grid, dist_field, tok) -> float:
        if grid is None or dist_field is None:
            return float(tok.path_cost if np.isfinite(tok.path_cost) else tok.distance_to_robot)
        rc = grid.try_world_to_grid(tok.x, tok.y)
        if rc is None:
            return float("inf")
        return float(dist_field[rc])


def normalize_per_column(raw: np.ndarray, method: str = "minmax") -> np.ndarray:
    if method != "minmax":
        raise ValueError(f"unsupported normalization method: {method}")

    data = raw.astype(np.float32, copy=True)
    data[~np.isfinite(data)] = np.nan
    norm = np.ones_like(data, dtype=np.float32)

    for feature_idx in range(data.shape[-1]):
        feature = data[..., feature_idx]
        finite = np.isfinite(feature)
        if not finite.any():
            continue
        min_v = np.nanmin(feature)
        max_v = np.nanmax(feature)
        if np.isclose(max_v, min_v):
            norm[..., feature_idx][finite] = 0.0
        else:
            norm[..., feature_idx][finite] = (feature[finite] - min_v) / (max_v - min_v)
    return norm
