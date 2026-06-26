import unittest

import numpy as np

from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from eavf_mappo.planning.astar import astar, bfs_distance_field


class AstarTests(unittest.TestCase):
    def test_astar_finds_shortest_four_connected_path(self):
        grid = OccupancyGrid(size_m=5.0, resolution=1.0)
        grid.cells[:, :] = FREE
        grid.cells[2, 1:4] = OCCUPIED
        grid.cells[2, 2] = FREE

        path = astar(grid, (0, 0), (4, 4))

        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))
        self.assertEqual(len(path) - 1, 8)

    def test_astar_returns_none_when_goal_is_unreachable(self):
        grid = OccupancyGrid(size_m=3.0, resolution=1.0)
        grid.cells[:, :] = FREE
        grid.cells[1, :] = OCCUPIED

        self.assertIsNone(astar(grid, (0, 0), (2, 2)))

    def test_bfs_distance_field_uses_inf_for_unreachable_cells(self):
        grid = OccupancyGrid(size_m=3.0, resolution=1.0)
        grid.cells[:, :] = FREE
        grid.cells[1, :] = OCCUPIED

        dist = bfs_distance_field(grid, (0, 0))

        self.assertEqual(dist[0, 0], 0.0)
        self.assertTrue(np.isinf(dist[2, 2]))


if __name__ == "__main__":
    unittest.main()
