from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from eavf_mappo.communication.message import FrontierMessage
from eavf_mappo.exploration.frontier_token import FrontierToken
from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, UNKNOWN
from eavf_mappo.planning.astar import astar


@dataclass
class Cluster:
    cells: list[tuple[int, int]]
    frontier_id: int
    grid: object | None = None

    def center_grid(self) -> tuple[float, float]:
        arr = np.asarray(self.cells, dtype=np.float32)
        return float(arr[:, 0].mean()), float(arr[:, 1].mean())

    def center_world(self, grid=None) -> tuple[float, float]:
        grid = self.grid if grid is None else grid
        if grid is None:
            r, c = self.center_grid()
            return c, r
        points = [grid.grid_to_world(r, c) for r, c in self.cells]
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return float(np.mean(xs)), float(np.mean(ys))


class FrontierDetector:
    def __init__(
        self,
        sensor_range: float = 3.0,
        sigma_obs: float = 0.5,
        r_comm: float = 5.0,
        overlap_radius: float = 0.5,
        collision_sigma: float = 1.0,
    ):
        self.sensor_range = float(sensor_range)
        self.sigma_obs = float(sigma_obs)
        self.r_comm = float(r_comm)
        self.overlap_radius = float(overlap_radius)
        self.collision_sigma = float(collision_sigma)
        self._last_grid = None

    def detect_frontiers(self, grid) -> list[tuple[int, int]]:
        self._last_grid = grid
        free = grid.cells == FREE
        unknown = grid.cells == UNKNOWN
        neighbor_unknown = np.zeros_like(unknown)
        neighbor_unknown[1:, :] |= unknown[:-1, :]
        neighbor_unknown[:-1, :] |= unknown[1:, :]
        neighbor_unknown[:, 1:] |= unknown[:, :-1]
        neighbor_unknown[:, :-1] |= unknown[:, 1:]
        rows, cols = np.nonzero(free & neighbor_unknown)
        return list(zip(rows.tolist(), cols.tolist()))

    def cluster_frontiers(self, cells: list[tuple[int, int]]) -> list[Cluster]:
        cell_set = set(cells)
        components: list[list[tuple[int, int]]] = []
        while cell_set:
            start = cell_set.pop()
            comp = [start]
            queue = deque([start])
            while queue:
                r, c = queue.popleft()
                for nr in (r - 1, r, r + 1):
                    for nc in (c - 1, c, c + 1):
                        if (nr, nc) in cell_set:
                            cell_set.remove((nr, nc))
                            comp.append((nr, nc))
                            queue.append((nr, nc))
            components.append(comp)
        # Discovery order depends on set/hash iteration order, which can
        # change between calls even when most clusters are unchanged
        # (e.g. one extra cell discovered elsewhere reshuffles the hash
        # table). Sorting by a stable spatial key keeps frontier_id tied
        # to geometry instead of to incidental iteration order, so a robot
        # tracking the same target_frontier_id across steps keeps pointing
        # at (approximately) the same physical frontier.
        components.sort(key=min)
        return [
            Cluster(comp, frontier_id=idx, grid=self._last_grid)
            for idx, comp in enumerate(components)
        ]

    def build_frontier_tokens(
        self,
        robot,
        clusters: list[Cluster],
        grid,
        other_robots,
        messages: list[FrontierMessage],
        comm_aware: bool,
    ) -> list[FrontierToken]:
        tokens: list[FrontierToken] = []
        robot_rc = grid.try_world_to_grid(robot.x, robot.y)
        occupied_xy = self._occupied_world_points(grid)
        for cid, cluster in enumerate(clusters):
            cx, cy = cluster.center_world(grid)
            frontier_id = getattr(cluster, "frontier_id", cid)
            gain = self.estimate_information_gain(grid, (cx, cy), self.sensor_range, n_rays=36)
            d_obs = self.nearest_obstacle_distance(occupied_xy, (cx, cy))
            obstacle_risk = float(math.exp(-d_obs / max(self.sigma_obs, 1.0e-8)))

            if comm_aware:
                others_info = [m.position for m in messages]
            else:
                others_info = [(r.x, r.y) for r in other_robots]
            collision_risk = self.collision_risk_to_others((cx, cy), others_info)

            overlap_risk, assigned = self.overlap_risk(
                (cx, cy),
                frontier_id,
                messages if comm_aware else other_robots,
                comm_aware=comm_aware,
            )
            comm_uncertainty = self.comm_uncertainty((cx, cy), robot, others_info)

            goal_rc = self._reachable_goal_rc(grid, (cx, cy), cluster)
            path = astar(grid, robot_rc, goal_rc) if robot_rc is not None and goal_rc is not None else None
            path_cost = float(len(path) - 1) if path else float("inf")
            distance = math.hypot(robot.x - cx, robot.y - cy)

            tokens.append(
                FrontierToken(
                    frontier_id=frontier_id,
                    x=cx,
                    y=cy,
                    distance_to_robot=distance,
                    path_cost=path_cost,
                    information_gain=float(gain),
                    obstacle_risk=float(np.clip(obstacle_risk, 0.0, 1.0)),
                    collision_risk=float(np.clip(collision_risk, 0.0, 1.0)),
                    overlap_risk=float(np.clip(overlap_risk, 0.0, 1.0)),
                    comm_uncertainty=float(np.clip(comm_uncertainty, 0.0, 1.0)),
                    assigned_flag=int(assigned),
                    valid=True,
                )
            )
        return tokens

    def select_top_k(self, tokens: list[FrontierToken], k: int) -> list[FrontierToken]:
        if k < 0:
            raise ValueError("k must be non-negative")
        valid = [t for t in tokens if t.valid]
        valid.sort(key=lambda t: (not np.isfinite(t.path_cost), t.path_cost, t.frontier_id))
        selected = valid[:k]
        while len(selected) < k:
            selected.append(
                FrontierToken(
                    frontier_id=-1,
                    x=0.0,
                    y=0.0,
                    distance_to_robot=0.0,
                    path_cost=float("inf"),
                    information_gain=0.0,
                    obstacle_risk=0.0,
                    collision_risk=0.0,
                    overlap_risk=0.0,
                    comm_uncertainty=0.0,
                    assigned_flag=0,
                    valid=False,
                )
            )
        return selected

    def estimate_information_gain(self, grid, xy, sensor_range: float, n_rays: int = 36) -> int:
        x, y = xy
        unknown: set[tuple[int, int]] = set()
        step = grid.resolution
        for idx in range(n_rays):
            angle = 2.0 * math.pi * idx / n_rays
            distance = 0.0
            while distance <= sensor_range:
                rc = grid.try_world_to_grid(
                    x + math.cos(angle) * distance,
                    y + math.sin(angle) * distance,
                )
                if rc is None:
                    break
                value = grid.cells[rc]
                if value == OCCUPIED:
                    break
                if value == UNKNOWN:
                    unknown.add(rc)
                distance += step
        return len(unknown)

    @staticmethod
    def _occupied_world_points(grid) -> np.ndarray:
        rc = np.argwhere(grid.cells == OCCUPIED)
        if rc.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        rows = rc[:, 0].astype(np.float64)
        cols = rc[:, 1].astype(np.float64)
        xs = grid.origin + (cols + 0.5) * grid.resolution
        ys = grid.origin + (rows + 0.5) * grid.resolution
        return np.stack([xs, ys], axis=1)

    def nearest_obstacle_distance(self, occupied_xy: np.ndarray, xy) -> float:
        if occupied_xy.size == 0:
            return self.sensor_range
        x, y = xy
        dists = np.hypot(occupied_xy[:, 0] - x, occupied_xy[:, 1] - y)
        return float(min(float(dists.min()), self.sensor_range))

    def collision_risk_to_others(self, xy, others_info) -> float:
        if not others_info:
            return 0.0
        x, y = xy
        risks = []
        for ox, oy in others_info:
            dist = math.hypot(float(ox) - x, float(oy) - y)
            risks.append(math.exp(-dist / max(self.collision_sigma, 1.0e-8)))
        return float(max(risks))

    def overlap_risk(self, xy, frontier_id: int, peers, comm_aware: bool) -> tuple[float, bool]:
        if not peers:
            return 0.0, False
        x, y = xy
        best = 0.0
        assigned = False
        for peer in peers:
            if comm_aware:
                target_id = peer.target_frontier_id
                px, py = peer.frontier_position
            else:
                # Without communication we cannot see a peer's true target_frontier_id,
                # so overlap must be inferred from position alone, not ground truth.
                target_id = None
                px, py = getattr(peer, "x", x), getattr(peer, "y", y)
            dist = math.hypot(float(px) - x, float(py) - y)
            if (comm_aware and target_id == frontier_id) or dist <= self.overlap_radius:
                assigned = True
            best = max(best, math.exp(-dist / max(self.overlap_radius, 1.0e-8)))
        return best, assigned

    def comm_uncertainty(self, xy, robot, others_info) -> float:
        positions = [(robot.x, robot.y)] + list(others_info)
        if not positions:
            return 1.0
        x, y = xy
        min_dist = min(math.hypot(float(px) - x, float(py) - y) for px, py in positions)
        return min_dist / max(self.r_comm, 1.0e-8)

    def _reachable_goal_rc(self, grid, xy, cluster: Cluster) -> tuple[int, int] | None:
        center_rc = grid.try_world_to_grid(float(xy[0]), float(xy[1]))
        if center_rc is not None and grid.is_free(*center_rc):
            return center_rc
        free_cells = [cell for cell in cluster.cells if grid.is_free(*cell)]
        if not free_cells:
            return None
        if center_rc is None:
            return free_cells[0]
        return min(free_cells, key=lambda rc: abs(rc[0] - center_rc[0]) + abs(rc[1] - center_rc[1]))
