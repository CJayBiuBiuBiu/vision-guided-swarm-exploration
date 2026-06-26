from __future__ import annotations

import math

import numpy as np

from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED


def generate_true_world(grid, map_type: str = "open_room", seed: int | None = None) -> np.ndarray:
    rng = np.random.default_rng(seed)
    world = np.full(grid.cells.shape, FREE, dtype=np.int8)
    world[0, :] = OCCUPIED
    world[-1, :] = OCCUPIED
    world[:, 0] = OCCUPIED
    world[:, -1] = OCCUPIED

    map_type = map_type.lower()
    if map_type == "open_room":
        return world
    if map_type == "corridor":
        _add_corridor_walls(world, rng)
    elif map_type == "maze":
        _add_maze_walls(world, rng)
    elif map_type == "office":
        _add_office_blocks(world, rng)
    elif map_type == "cluttered":
        _add_clutter(world, rng, density=0.08)
    else:
        raise ValueError(f"unknown map_type: {map_type}")
    return world


def clear_world_disk(world: np.ndarray, grid, x: float, y: float, radius_m: float) -> None:
    center = grid.try_world_to_grid(x, y)
    if center is None:
        return
    radius_cells = max(0, int(math.ceil(radius_m / grid.resolution)))
    cr, cc = center
    for r in range(cr - radius_cells, cr + radius_cells + 1):
        for c in range(cc - radius_cells, cc + radius_cells + 1):
            if not grid.in_bounds(r, c):
                continue
            wx, wy = grid.grid_to_world(r, c)
            if math.hypot(wx - x, wy - y) <= radius_m + grid.resolution:
                world[r, c] = FREE


def raycast_ranges(
    world: np.ndarray,
    grid,
    pose: tuple[float, float, float],
    max_range: float,
    n_rays: int,
) -> list[tuple[float, float, bool]]:
    if n_rays <= 0:
        raise ValueError("n_rays must be positive")
    x, y, theta = float(pose[0]), float(pose[1]), float(pose[2])
    step = max(grid.resolution * 0.25, 1.0e-3)
    ranges: list[tuple[float, float, bool]] = []
    for idx in range(n_rays):
        rel_angle = 2.0 * math.pi * idx / n_rays
        angle = theta + rel_angle
        distance = 0.0
        hit = False
        while distance <= max_range:
            px = x + math.cos(angle) * distance
            py = y + math.sin(angle) * distance
            rc = grid.try_world_to_grid(px, py)
            if rc is None:
                hit = True
                break
            if world[rc] == OCCUPIED:
                hit = True
                break
            distance += step
        ranges.append((rel_angle, min(distance, max_range), hit))
    return ranges


def is_world_free(world: np.ndarray, grid, x: float, y: float) -> bool:
    rc = grid.try_world_to_grid(x, y)
    return rc is not None and world[rc] == FREE


def _add_corridor_walls(world: np.ndarray, rng) -> None:
    h, w = world.shape
    if h <= 6 or w <= 6:
        return

    jitter = max(1, h // 12)
    upper = int(np.clip(h // 3 + rng.integers(-jitter, jitter + 1), 2, h - 4))
    lower = int(np.clip(2 * h // 3 + rng.integers(-jitter, jitter + 1), upper + 2, h - 3))
    gap_width = int(rng.integers(2, max(3, min(5, w // 3))))

    for row in [upper, lower]:
        gap_center = int(rng.integers(3, max(4, w - 3)))
        gap_start = max(2, gap_center - gap_width // 2)
        gap_end = min(w - 2, gap_start + gap_width)
        world[row, 2 : w - 2] = OCCUPIED
        world[row, gap_start:gap_end] = FREE


def _add_maze_walls(world: np.ndarray, rng) -> None:
    h, w = world.shape
    for r in range(3, h - 3, 4):
        world[r, 2 : w - 2] = OCCUPIED
        gap = int(rng.integers(2, w - 2))
        world[r, max(2, gap - 1) : min(w - 2, gap + 2)] = FREE
    for c in range(5, w - 5, 6):
        world[2 : h - 2, c] = OCCUPIED
        gap = int(rng.integers(2, h - 2))
        world[max(2, gap - 1) : min(h - 2, gap + 2), c] = FREE


def _add_office_blocks(world: np.ndarray, rng) -> None:
    h, w = world.shape
    if h <= 6 or w <= 6:
        return

    block_h = max(2, h // 8)
    block_w = max(2, w // 8)
    row_step = max(3, block_h * 2 + int(rng.integers(0, 3)))
    col_step = max(3, block_w * 2 + int(rng.integers(0, 3)))
    start_r = int(np.clip(h // 4 + rng.integers(-1, 2), 2, h - 3))
    start_c = int(np.clip(w // 4 + rng.integers(-1, 2), 2, w - 3))
    end_r = max(start_r + 1, min(h - 2, 3 * h // 4 + int(rng.integers(-1, 2))))
    end_c = max(start_c + 1, min(w - 2, 3 * w // 4 + int(rng.integers(-1, 2))))

    placed = False
    for r in range(start_r, end_r, row_step):
        for c in range(start_c, end_c, col_step):
            if rng.random() < 0.25:
                continue
            rr = int(np.clip(r + rng.integers(-1, 2), 1, h - 2))
            cc = int(np.clip(c + rng.integers(-1, 2), 1, w - 2))
            bh = max(2, block_h + int(rng.integers(-1, 2)))
            bw = max(2, block_w + int(rng.integers(-1, 2)))
            world[rr : min(h - 1, rr + bh), cc : min(w - 1, cc + bw)] = OCCUPIED
            placed = True

    if not placed:
        rr = int(np.clip(h // 2 + rng.integers(-1, 2), 1, h - 3))
        cc = int(np.clip(w // 2 + rng.integers(-1, 2), 1, w - 3))
        world[rr : min(h - 1, rr + block_h), cc : min(w - 1, cc + block_w)] = OCCUPIED


def _add_clutter(world: np.ndarray, rng, density: float) -> None:
    h, w = world.shape
    interior = (slice(2, h - 2), slice(2, w - 2))
    mask = rng.random((h - 4, w - 4)) < density
    world[interior][mask] = OCCUPIED
