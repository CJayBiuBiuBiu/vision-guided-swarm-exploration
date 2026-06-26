from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Iterable

import numpy as np


def astar(grid, start_rc, goal_rc, obstacle_cost=None) -> list[tuple[int, int]] | None:
    start = tuple(start_rc)
    goal = tuple(goal_rc)
    if not _is_free(grid, *start) or not _is_free(grid, *goal):
        return None

    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}
    visited: set[tuple[int, int]] = set()

    while frontier:
        _, current = heapq.heappop(frontier)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return _reconstruct_path(came_from, current)

        for nxt in _neighbors(grid, current):
            if nxt in visited:
                continue
            step_cost = 1.0
            if obstacle_cost is not None:
                step_cost += float(obstacle_cost[nxt])
            new_cost = cost_so_far[current] + step_cost
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + _manhattan(nxt, goal)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    return None


def bfs_distance_field(grid, start_rc) -> np.ndarray:
    height, width = _shape(grid)
    dist = np.full((height, width), np.inf, dtype=np.float32)
    start = tuple(start_rc)
    if not _is_free(grid, *start):
        return dist

    queue: deque[tuple[int, int]] = deque([start])
    dist[start] = 0.0
    while queue:
        current = queue.popleft()
        for nxt in _neighbors(grid, current):
            if np.isfinite(dist[nxt]):
                continue
            dist[nxt] = dist[current] + 1.0
            queue.append(nxt)
    return dist


def _shape(grid) -> tuple[int, int]:
    if hasattr(grid, "cells"):
        return grid.cells.shape
    return np.asarray(grid).shape


def _is_free(grid, r: int, c: int) -> bool:
    if hasattr(grid, "is_free"):
        return bool(grid.is_free(r, c))
    arr = np.asarray(grid)
    return 0 <= r < arr.shape[0] and 0 <= c < arr.shape[1] and arr[r, c] == 0


def _neighbors(grid, rc: tuple[int, int]) -> Iterable[tuple[int, int]]:
    r, c = rc
    for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
        if _is_free(grid, nr, nc):
            yield nr, nc


def _manhattan(a: tuple[int, int], b: tuple[int, int]) -> float:
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))


def _reconstruct_path(came_from, current) -> list[tuple[int, int]]:
    path = [current]
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
