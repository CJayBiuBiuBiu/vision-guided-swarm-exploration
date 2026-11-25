"""Exercise the ROS coordinator's active passage decisions without Gazebo."""

import argparse
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np


def suite(package):
    sys.path.insert(0, str(package / "scripts" / "voronoi_exploration"))
    from exploration_coordinator import Coordinator
    from rgb_passage import BEARINGS_DEG, WIDTH_DISTANCES_M

    class CoordinatorPassageTests(unittest.TestCase):
        def setUp(self):
            self.robot = "robot1"
            self.coord = Coordinator.__new__(Coordinator)
            self.coord._control_now = Mock(return_value=10.0)
            self.coord._frontier_trace_snapshot = Mock(return_value=(10.0, 12.0))
            self.coord.frontier_trace = Mock()
            self.coord.xs = self.coord.ys = np.arange(-2.0, 2.01, 0.2)
            self.coord.grid = np.ones((21, 21), dtype=np.uint8)
            self.coord._cell_index = lambda x, y: (int(round((x + 2) / 0.2)), int(round((y + 2) / 0.2)))
            self.coord.passage_inspections = {}
            self.coord.passage_continuations = {}
            self.coord.current_goal = {self.robot: (0.0, 0.0)}
            self.coord.current_goal_id = {self.robot: 11}
            self.coord.current_path = {}
            self.coord.goal_set_time = {}
            self.coord.clearance_telemetry = Mock()
            self.coord.clearance_telemetry.recent_clearance.return_value = 0.8
            self.coord.near_collision_clear_range = 0.25
            self.coord._plan_path = Mock(return_value=[(10, 10), (10, 11), (10, 12)])
            self.coord._retire_policy_waypoint = Mock()
            self.widths = np.ones((len(BEARINGS_DEG), len(WIDTH_DISTANCES_M)))

        def frames(self, position, yaw, now):
            frame = (np.array([*position, 0.0]), yaw, self.widths)
            self.coord.passage_width_frames = {self.robot: [(now - 0.2, frame), (now - 0.1, frame)]}

        def test_aligned_inspection_redirects_to_rebound_candidate(self):
            origin = np.array([0.0, 0.0])
            self.coord.passage_inspections[self.robot] = dict(
                start=8.0, origin=origin, preferred=10, alternative=11,
                preferred_xy=np.array([1.0, 0.0]), target=np.array([0.0, 1.0]),
                score={"bottleneck_m": 2.0, "clear_to_m": 1.75}, route_m=1.0)
            self.frames(origin, math.pi / 2, 10.0)
            frontiers = np.array([[1.0, 0.0], [0.0, 1.2]])
            waiting, decision = self.coord._advance_passage_inspection(
                self.robot, origin, frontiers, [10, 12], np.array([0, 0]), 0)
            self.assertFalse(waiting)
            self.assertEqual(decision[0], 1)
            self.assertNotIn(self.robot, self.coord.passage_inspections)
            np.testing.assert_allclose(self.coord.passage_continuations[self.robot]["direction"], [0, 1])
            self.assertEqual(self.coord.frontier_trace.record.call_args.kwargs["reason"], "redirected")

        def test_safe_continuation_and_wait_recheck(self):
            position = np.array([0.0, 0.0])
            self.coord._start_passage_continuation(self.robot, position, np.array([1.0, 0.0]))
            self.frames(position, 0.0, 10.0)
            goal = self.coord._continue_passage(self.robot, position)
            np.testing.assert_allclose(goal, [1.2, 0.0])
            self.assertEqual(self.coord.passage_continuations[self.robot]["steps"], 1)
            self.assertEqual(self.coord.current_goal[self.robot], goal)
            self.assertTrue(self.coord.current_path[self.robot])
            self.assertTrue(self.coord._keep_passage_on_wait(self.robot, position, goal))
            self.coord.grid[10, 13] = 2
            self.assertFalse(self.coord._keep_passage_on_wait(self.robot, position, goal))

        def test_unknown_corridor_and_step_budget_stop_extension(self):
            position = np.array([0.0, 0.0])
            self.coord._start_passage_continuation(self.robot, position, np.array([1.0, 0.0]))
            self.frames(position, 0.0, 10.0)
            self.coord.grid[:, 12:] = 0
            self.assertIsNone(self.coord._continue_passage(self.robot, position))
            self.assertNotIn(self.robot, self.coord.passage_continuations)
            self.coord.grid[:] = 1
            self.coord._start_passage_continuation(self.robot, position, np.array([1.0, 0.0]))
            self.coord.passage_continuations[self.robot]["steps"] = 2
            self.assertIsNone(self.coord._continue_passage(self.robot, position))
            self.assertNotIn(self.robot, self.coord.passage_continuations)

    return unittest.defaultTestLoader.loadTestsFromTestCase(CoordinatorPassageTests)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    args = parser.parse_args()
    result = unittest.TextTestRunner(verbosity=2).run(suite(args.package.resolve()))
    sys.exit(not result.wasSuccessful())
