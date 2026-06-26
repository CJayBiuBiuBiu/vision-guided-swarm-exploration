import unittest

import numpy as np

from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.exploration.frontier_token import FrontierToken
from eavf_mappo.mapping.occupancy_grid import FREE, OccupancyGrid
from eavf_mappo.robots.robot_state import RobotState


class VoronoiTests(unittest.TestCase):
    def test_cost_matrix_normalizes_features_and_masks_invalid(self):
        grid = OccupancyGrid(size_m=5.0, resolution=1.0)
        grid.cells[:, :] = FREE
        robots = [
            RobotState(0, -1.5, -1.5, 0.0, 0.0, 0.0, None, [], False, 0, 0),
            RobotState(1, 1.5, 1.5, 0.0, 0.0, 0.0, None, [], False, 0, 0),
        ]
        tokens = [
            [
                FrontierToken(0, -0.5, -0.5, 1.0, 2.0, 5.0, 0.1, 0.1, 0.0, 0.0, 0),
                FrontierToken(1, 1.5, 1.5, 4.0, 6.0, 1.0, 0.5, 0.4, 0.2, 0.2, 0),
            ],
            [
                FrontierToken(0, -0.5, -0.5, 4.0, 6.0, 1.0, 0.5, 0.4, 0.2, 0.2, 0),
                FrontierToken(-1, 0.0, 0.0, 0.0, float("inf"), 0.0, 0.0, 0.0, 0.0, 0.0, 0, valid=False),
            ],
        ]
        weights = {"alpha": 1.0, "beta": 0.5, "gamma": 0.8, "delta": 0.6, "eta": 0.4, "lam": 1.2}

        cost = ExecutionAwareVoronoi(weights, grid=grid).compute_cost_matrix(robots, tokens)

        self.assertEqual(cost.shape, (2, 2))
        self.assertTrue(np.isfinite(cost[0, 0]))
        self.assertTrue(np.isinf(cost[1, 1]))

    def test_assignment_prefers_lowest_unique_costs(self):
        cost = np.array([[1.0, 5.0], [0.5, 0.8]])

        assignment = ExecutionAwareVoronoi({}).assign(cost)

        self.assertEqual(assignment[1], 0)
        self.assertEqual(assignment[0], 1)


if __name__ == "__main__":
    unittest.main()
