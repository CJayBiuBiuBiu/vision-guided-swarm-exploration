import math
import unittest

import numpy as np

from eavf_mappo.envs.grid_world import (
    clear_world_disk,
    generate_true_world,
    raycast_ranges,
)
from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, OccupancyGrid


class GridWorldTests(unittest.TestCase):
    def test_generated_world_has_occupied_border_and_free_interior(self):
        grid = OccupancyGrid(size_m=6.0, resolution=0.5)

        world = generate_true_world(grid, map_type="open_room", seed=0)

        self.assertEqual(world.shape, grid.cells.shape)
        self.assertTrue(np.all(world[0, :] == OCCUPIED))
        self.assertTrue(np.all(world[-1, :] == OCCUPIED))
        self.assertGreater(np.count_nonzero(world == FREE), 0)

    def test_structured_maps_vary_across_different_seeds(self):
        grid = OccupancyGrid(size_m=8.0, resolution=0.5)

        for map_type in ["office", "corridor"]:
            with self.subTest(map_type=map_type):
                seed0 = generate_true_world(grid, map_type=map_type, seed=0)
                seed1 = generate_true_world(grid, map_type=map_type, seed=1)

                self.assertFalse(np.array_equal(seed0, seed1))

    def test_clear_world_disk_opens_spawn_area(self):
        grid = OccupancyGrid(size_m=4.0, resolution=0.5)
        world = np.full(grid.cells.shape, OCCUPIED, dtype=np.int8)

        clear_world_disk(world, grid, 0.0, 0.0, 0.6)

        center = grid.world_to_grid(0.0, 0.0)
        self.assertEqual(world[center], FREE)

    def test_raycast_reports_hit_distance_for_true_obstacle(self):
        grid = OccupancyGrid(size_m=4.0, resolution=0.5)
        world = np.full(grid.cells.shape, FREE, dtype=np.int8)
        obstacle_rc = grid.world_to_grid(1.0, 0.0)
        world[obstacle_rc] = OCCUPIED

        ranges = raycast_ranges(
            world,
            grid,
            pose=(0.0, 0.0, 0.0),
            max_range=2.0,
            n_rays=4,
        )

        angle, distance, has_hit = ranges[0]
        self.assertAlmostEqual(angle, 0.0)
        self.assertTrue(has_hit)
        self.assertLess(distance, 1.5)


if __name__ == "__main__":
    unittest.main()
