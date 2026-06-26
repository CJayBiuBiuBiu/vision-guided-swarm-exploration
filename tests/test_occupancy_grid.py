import unittest

import numpy as np

from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid


class OccupancyGridTests(unittest.TestCase):
    def test_world_grid_round_trip_stays_within_one_cell(self):
        grid = OccupancyGrid(size_m=4.0, resolution=0.5)

        for x, y in [(-1.75, -1.75), (-0.2, 0.2), (1.2, -0.8)]:
            r, c = grid.world_to_grid(x, y)
            x2, y2 = grid.grid_to_world(r, c)
            self.assertLessEqual(abs(x - x2), grid.resolution)
            self.assertLessEqual(abs(y - y2), grid.resolution)

    def test_coverage_ratio_counts_known_cells(self):
        grid = OccupancyGrid(size_m=2.0, resolution=1.0)
        grid.cells[0, 0] = FREE
        grid.cells[0, 1] = OCCUPIED

        self.assertAlmostEqual(grid.coverage_ratio(), 0.5)

    def test_local_bev_has_fixed_channels_and_robot_center(self):
        grid = OccupancyGrid(size_m=4.0, resolution=0.25)
        center = grid.world_to_grid(0.0, 0.0)
        grid.cells[center] = FREE

        bev = grid.get_local_bev((0.0, 0.0, 0.0))

        self.assertEqual(bev.shape, (5, 64, 64))
        self.assertEqual(bev.dtype, np.float32)
        self.assertEqual(bev[3, 32, 32], 1.0)
        self.assertGreater(bev[0].sum(), 0.0)
        self.assertGreater(bev[2].sum(), 0.0)

    def test_ray_observation_marks_free_and_hit_cells(self):
        grid = OccupancyGrid(size_m=4.0, resolution=0.5)

        grid.update_from_observation((0.0, 0.0, 0.0), [(0.0, 1.0)])
        hit_rc = grid.world_to_grid(1.0, 0.0)

        self.assertEqual(grid.cells[hit_rc], OCCUPIED)
        self.assertGreater(np.count_nonzero(grid.cells == FREE), 0)
        self.assertGreater(np.count_nonzero(grid.cells == UNKNOWN), 0)


if __name__ == "__main__":
    unittest.main()
