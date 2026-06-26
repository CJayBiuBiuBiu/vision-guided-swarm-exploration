import unittest

from eavf_mappo.communication.message import FrontierMessage
from eavf_mappo.exploration.frontier_detector import FrontierDetector
from eavf_mappo.mapping.occupancy_grid import FREE, OccupancyGrid
from eavf_mappo.robots.robot_state import RobotState


class FrontierDetectorTests(unittest.TestCase):
    def _grid_with_free_patch(self):
        grid = OccupancyGrid(size_m=5.0, resolution=1.0)
        grid.cells[1:4, 1:4] = FREE
        return grid

    def test_detects_free_unknown_boundary_cells(self):
        grid = self._grid_with_free_patch()
        detector = FrontierDetector(sensor_range=3.0)

        cells = detector.detect_frontiers(grid)
        clusters = detector.cluster_frontiers(cells)

        self.assertEqual(len(cells), 8)
        self.assertEqual(len(clusters), 1)

    def test_build_tokens_uses_messages_when_comm_aware(self):
        grid = self._grid_with_free_patch()
        detector = FrontierDetector(sensor_range=3.0, overlap_radius=1.5)
        clusters = detector.cluster_frontiers(detector.detect_frontiers(grid))
        robot = RobotState(0, 0.0, 0.0, 0.0, 0.0, 0.0, None, [], False, 0, 0)
        other = RobotState(1, 10.0, 10.0, 0.0, 0.0, 0.0, 3, [], False, 0, 0)
        cx, cy = clusters[0].center_world()
        msg = FrontierMessage(1, (cx, cy), 3, (cx, cy), 0.2, 0.1, "exploring", 7)

        tokens = detector.build_frontier_tokens(
            robot, clusters, grid, [other], [msg], comm_aware=True
        )

        self.assertEqual(len(tokens), 1)
        self.assertEqual(tokens[0].assigned_flag, 1)
        self.assertGreater(tokens[0].overlap_risk, 0.5)
        self.assertGreater(tokens[0].collision_risk, 0.5)

    def test_top_k_pads_invalid_tokens(self):
        grid = self._grid_with_free_patch()
        detector = FrontierDetector(sensor_range=3.0)
        clusters = detector.cluster_frontiers(detector.detect_frontiers(grid))
        robot = RobotState(0, 0.0, 0.0, 0.0, 0.0, 0.0, None, [], False, 0, 0)
        tokens = detector.build_frontier_tokens(robot, clusters, grid, [], [], comm_aware=False)

        selected = detector.select_top_k(tokens, 4)

        self.assertEqual(len(selected), 4)
        self.assertTrue(selected[0].valid)
        self.assertFalse(selected[-1].valid)
        self.assertEqual(selected[-1].to_vector().sum(), 0.0)


if __name__ == "__main__":
    unittest.main()
