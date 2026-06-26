from __future__ import annotations

import math

from eavf_mappo.planning.astar import astar


class LocalController:
    def __init__(
        self,
        wheel_base_L: float = 0.3,
        wheel_radius_r: float = 0.05,
        v_max: float = 0.5,
        omega_max: float = 1.5,
    ):
        self.wheel_base_L = float(wheel_base_L)
        self.wheel_radius_r = float(wheel_radius_r)
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)

    def plan_path(self, robot, target_xy, grid) -> list[tuple[int, int]]:
        start = grid.try_world_to_grid(robot.x, robot.y)
        goal = grid.try_world_to_grid(float(target_xy[0]), float(target_xy[1]))
        if start is None or goal is None:
            return []
        path = astar(grid, start, goal)
        return path or []

    def compute_command(
        self, robot, path, grid, others, in_collision: bool | None = None
    ) -> tuple[float, float]:
        if in_collision is None:
            in_collision = self.is_colliding(robot, others)
        if in_collision:
            turn_direction = -1.0 if robot.stuck_counter % 2 else 1.0
            return float(-0.5 * self.v_max), float(turn_direction * 0.75 * self.omega_max)

        if not path:
            return 0.0, 0.0
        waypoint = path[min(1, len(path) - 1)]
        wx, wy = grid.grid_to_world(*waypoint)
        desired = math.atan2(wy - robot.y, wx - robot.x)
        angle_error = wrap_angle(desired - robot.theta)
        omega = max(-self.omega_max, min(self.omega_max, 2.0 * angle_error))
        v = self.v_max * max(0.0, 1.0 - abs(angle_error) / math.pi)

        return float(v), float(omega)

    def is_colliding(self, robot, others, threshold: float = 0.4) -> bool:
        return any(math.hypot(other.x - robot.x, other.y - robot.y) < threshold for other in others)

    def to_wheel_speeds(self, v: float, omega: float) -> tuple[float, float]:
        wl = (v - omega * self.wheel_base_L / 2.0) / self.wheel_radius_r
        wr = (v + omega * self.wheel_base_L / 2.0) / self.wheel_radius_r
        return float(wl), float(wr)


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
