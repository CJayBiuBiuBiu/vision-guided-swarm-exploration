from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RobotState:
    robot_id: int
    x: float
    y: float
    theta: float
    v: float
    omega: float
    target_frontier_id: int | None
    path: list[tuple[int, int]]
    in_collision: bool
    stuck_counter: int
    steps_since_high_level: int

    @property
    def position(self) -> tuple[float, float]:
        return (self.x, self.y)
