from __future__ import annotations

import math
from typing import Sequence

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 1


class OccupancyGrid:
    def __init__(self, size_m: float, resolution: float):
        if size_m <= 0:
            raise ValueError("size_m must be positive")
        if resolution <= 0:
            raise ValueError("resolution must be positive")

        self.size_m = float(size_m)
        self.resolution = float(resolution)
        self.width = int(round(self.size_m / self.resolution))
        self.height = self.width
        self.origin = -self.size_m / 2.0
        self.cells = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        c = int(math.floor((float(x) - self.origin) / self.resolution))
        r = int(math.floor((float(y) - self.origin) / self.resolution))
        if not self.in_bounds(r, c):
            raise ValueError(f"world coordinate {(x, y)} is outside the grid")
        return r, c

    def try_world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        c = int(math.floor((float(x) - self.origin) / self.resolution))
        r = int(math.floor((float(y) - self.origin) / self.resolution))
        if not self.in_bounds(r, c):
            return None
        return r, c

    def grid_to_world(self, r: int, c: int) -> tuple[float, float]:
        if not self.in_bounds(r, c):
            raise ValueError(f"grid coordinate {(r, c)} is outside the grid")
        x = self.origin + (c + 0.5) * self.resolution
        y = self.origin + (r + 0.5) * self.resolution
        return x, y

    def is_free(self, r: int, c: int) -> bool:
        return self.in_bounds(r, c) and self.cells[r, c] == FREE

    def set_free_disk(self, x: float, y: float, radius_m: float) -> None:
        center = self.try_world_to_grid(x, y)
        if center is None:
            return
        radius_cells = max(0, int(math.ceil(radius_m / self.resolution)))
        cr, cc = center
        for r in range(cr - radius_cells, cr + radius_cells + 1):
            for c in range(cc - radius_cells, cc + radius_cells + 1):
                if not self.in_bounds(r, c):
                    continue
                wx, wy = self.grid_to_world(r, c)
                if math.hypot(wx - x, wy - y) <= radius_m + self.resolution:
                    self.cells[r, c] = FREE

    def update_from_observation(self, pose, ranges) -> None:
        x, y, theta = float(pose[0]), float(pose[1]), float(pose[2])
        observations = self._normalize_ranges(ranges)
        start_rc = self.try_world_to_grid(x, y)
        if start_rc is not None:
            self.cells[start_rc] = FREE

        for rel_angle, distance, has_hit in observations:
            if distance <= 0:
                continue
            angle = theta + rel_angle
            end_x = x + math.cos(angle) * distance
            end_y = y + math.sin(angle) * distance
            self._mark_ray_free((x, y), (end_x, end_y))
            end_rc = self.try_world_to_grid(end_x, end_y)
            if has_hit and end_rc is not None:
                self.cells[end_rc] = OCCUPIED

    def _normalize_ranges(self, ranges) -> list[tuple[float, float, bool]]:
        if ranges is None:
            return []
        ranges = list(ranges)
        if not ranges:
            return []

        first = ranges[0]
        if isinstance(first, Sequence) and not isinstance(first, (str, bytes)):
            result = []
            for item in ranges:
                if len(item) == 2:
                    angle, distance = item
                    has_hit = math.isfinite(float(distance))
                else:
                    angle, distance, has_hit = item
                result.append((float(angle), float(distance), bool(has_hit)))
            return result

        count = len(ranges)
        result = []
        for idx, distance in enumerate(ranges):
            angle = 2.0 * math.pi * idx / count
            result.append((angle, float(distance), math.isfinite(float(distance))))
        return result

    def _mark_ray_free(self, start_xy: tuple[float, float], end_xy: tuple[float, float]) -> None:
        sx, sy = start_xy
        ex, ey = end_xy
        length = math.hypot(ex - sx, ey - sy)
        steps = max(1, int(math.ceil(length / (self.resolution * 0.5))))
        for step in range(steps):
            t = step / steps
            rc = self.try_world_to_grid(sx + (ex - sx) * t, sy + (ey - sy) * t)
            if rc is not None:
                self.cells[rc] = FREE

    def get_local_bev(self, pose, size: tuple[int, int] = (64, 64), target_xy=None) -> np.ndarray:
        h, w = size
        bev = np.zeros((5, h, w), dtype=np.float32)
        robot_rc = self.try_world_to_grid(float(pose[0]), float(pose[1]))
        if robot_rc is None:
            return bev

        rr0, cc0 = robot_rc
        half_h, half_w = h // 2, w // 2
        gr0, gc0 = rr0 - half_h, cc0 - half_w

        local = np.full((h, w), UNKNOWN, dtype=np.int8)
        src_r0, src_r1 = max(0, gr0), min(self.height, gr0 + h)
        src_c0, src_c1 = max(0, gc0), min(self.width, gc0 + w)
        if src_r0 < src_r1 and src_c0 < src_c1:
            dst_r0, dst_c0 = src_r0 - gr0, src_c0 - gc0
            local[dst_r0 : dst_r0 + (src_r1 - src_r0), dst_c0 : dst_c0 + (src_c1 - src_c0)] = (
                self.cells[src_r0:src_r1, src_c0:src_c1]
            )

        bev[0] = (local == FREE).astype(np.float32)
        bev[1] = (local == OCCUPIED).astype(np.float32)
        bev[2] = (local == UNKNOWN).astype(np.float32)
        bev[3, half_h, half_w] = 1.0
        if target_xy is not None:
            target_rc = self.try_world_to_grid(float(target_xy[0]), float(target_xy[1]))
            if target_rc is not None:
                tr = target_rc[0] - rr0 + half_h
                tc = target_rc[1] - cc0 + half_w
                if 0 <= tr < h and 0 <= tc < w:
                    bev[4, tr, tc] = 1.0
        return bev

    def get_global_bev(self, robot_states=None, size: tuple[int, int] = (64, 64)) -> np.ndarray:
        h, w = size
        rows = np.linspace(0, self.height - 1, h).astype(np.int32)
        cols = np.linspace(0, self.width - 1, w).astype(np.int32)
        sampled = self.cells[np.ix_(rows, cols)]
        bev = np.zeros((5, h, w), dtype=np.float32)
        bev[0] = (sampled == FREE).astype(np.float32)
        bev[1] = (sampled == OCCUPIED).astype(np.float32)
        bev[2] = (sampled == UNKNOWN).astype(np.float32)

        for robot in robot_states or []:
            rc = self.try_world_to_grid(robot.x, robot.y)
            if rc is not None:
                br = min(h - 1, max(0, int(round(rc[0] * (h - 1) / max(1, self.height - 1)))))
                bc = min(w - 1, max(0, int(round(rc[1] * (w - 1) / max(1, self.width - 1)))))
                bev[3, br, bc] = 1.0
            if getattr(robot, "path", None):
                target = robot.path[-1]
                br = min(h - 1, max(0, int(round(target[0] * (h - 1) / max(1, self.height - 1)))))
                bc = min(w - 1, max(0, int(round(target[1] * (w - 1) / max(1, self.width - 1)))))
                bev[4, br, bc] = 1.0
        return bev

    def coverage_ratio(self) -> float:
        known = np.count_nonzero(self.cells != UNKNOWN)
        return float(known / self.cells.size)
