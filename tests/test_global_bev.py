import unittest

import numpy as np

from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from eavf_mappo.robots.robot_state import RobotState


class GlobalBevTests(unittest.TestCase):
    def test_global_bev_has_fixed_shape_and_channels(self):
        grid = OccupancyGrid(size_m=8.0, resolution=0.5)
        grid.cells[:, :] = FREE
        grid.cells[0, :] = OCCUPIED
        robot = RobotState(0, 0.0, 0.0, 0.0, 0.0, 0.0, 3, [(8, 8)], False, 0, 0)

        bev = grid.get_global_bev(robot_states=[robot], size=(64, 64))

        self.assertEqual(bev.shape, (5, 64, 64))
        self.assertEqual(bev.dtype, np.float32)
        self.assertGreater(bev[0].sum(), 0.0)
        self.assertGreater(bev[1].sum(), 0.0)
        self.assertGreater(bev[3].sum(), 0.0)
        self.assertGreater(bev[4].sum(), 0.0)


if __name__ == "__main__":
    unittest.main()
