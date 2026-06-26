from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FrontierMessage:
    robot_id: int
    position: tuple[float, float]
    target_frontier_id: int
    frontier_position: tuple[float, float]
    frontier_score: float
    risk: float
    intent: str
    timestamp: int
