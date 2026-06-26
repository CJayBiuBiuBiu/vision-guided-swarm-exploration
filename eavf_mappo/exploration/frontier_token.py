from __future__ import annotations

from dataclasses import dataclass

import numpy as np


TOKEN_DIM = 10


@dataclass
class FrontierToken:
    frontier_id: int
    x: float
    y: float
    distance_to_robot: float
    path_cost: float
    information_gain: float
    obstacle_risk: float
    collision_risk: float
    overlap_risk: float
    comm_uncertainty: float
    assigned_flag: int
    valid: bool = True

    def to_vector(self) -> np.ndarray:
        if not self.valid:
            return np.zeros(TOKEN_DIM, dtype=np.float32)

        path_cost = self.path_cost if np.isfinite(self.path_cost) else 1.0e6
        return np.asarray(
            [
                self.x,
                self.y,
                self.distance_to_robot,
                path_cost,
                self.information_gain,
                self.obstacle_risk,
                self.collision_risk,
                self.overlap_risk,
                self.comm_uncertainty,
                float(self.assigned_flag),
            ],
            dtype=np.float32,
        )
