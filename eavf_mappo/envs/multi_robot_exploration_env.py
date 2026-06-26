from __future__ import annotations

import math

import numpy as np

from eavf_mappo.control.local_controller import LocalController, wrap_angle
from eavf_mappo.envs.grid_world import (
    clear_world_disk,
    generate_true_world,
    is_world_free,
    raycast_ranges,
)
from eavf_mappo.exploration.frontier_detector import FrontierDetector
from eavf_mappo.mapping.occupancy_grid import OccupancyGrid
from eavf_mappo.robots.robot_state import RobotState
from eavf_mappo.utils.config import merge_config
from eavf_mappo.utils.seed import set_seed


# Part of the env's own action contract (any caller passing raw integer
# actions to step() can use these), so they live here rather than in
# eavf_mappo.marl.action_adapter, which depends on the env's contract
# rather than the other way around.
CONTINUE_ACTION = -1
RELEASE_ACTION = -2


DEFAULT_CONFIG = {
    "env": {
        "map_size_m": 10.0,
        "resolution": 0.05,
        "n_robots": 2,
        "max_episode_steps": 400,
        "coverage_done_thresh": 0.95,
        "sensor_range_m": 3.0,
        "sensor_n_rays": 36,
        "map_type": "open_room",
    },
    "high_level": {
        "arrival_thresh_m": 0.3,
        "stuck_timeout": 30,
        "max_high_level_interval": 50,
        "frontier_K": 8,
    },
    "reward": {
        # w_coverage is scaled up relative to w_collision: per-step
        # delta_coverage is naturally tiny (a fraction of the whole map),
        # while w_collision's penalty fires as a full -5 in one step. Left
        # at 1.0 they put a "do nothing" policy at a steep advantage over
        # exploring. w_collision itself stays as-is since it encodes a real
        # safety preference we want to keep once this runs on hardware.
        "w_coverage": 10.0,
        "w_path": 0.05,
        "w_collision": 5.0,
        "w_overlap": 1.0,
        "w_comm": 0.2,
        "w_frontier": 2.0,
        "time_penalty": 0.01,
        "charge_target_assignment_comm": False,
    },
}


class MultiRobotExplorationEnv:
    def __init__(self, config: dict | None = None):
        self.cfg = merge_config(DEFAULT_CONFIG, config or {})
        self.detector = FrontierDetector()
        self.controller = LocalController()
        self.grid: OccupancyGrid | None = None
        self.true_cells: np.ndarray | None = None
        self.robots: list[RobotState] = []
        self.current_step = 0
        self._last_coverage = 0.0
        self._cluster_cache: tuple[int, list] | None = None

    def reset(self, seed: int | None = None):
        if seed is not None:
            set_seed(seed)
        env_cfg = self.cfg["env"]
        self.grid = OccupancyGrid(env_cfg["map_size_m"], env_cfg["resolution"])
        self.true_cells = generate_true_world(self.grid, env_cfg.get("map_type", "open_room"), seed=seed)
        self.current_step = 0
        self._cluster_cache = None
        self.robots = []

        n = int(env_cfg["n_robots"])
        spacing = min(0.5, env_cfg["map_size_m"] / max(4, n + 1))
        offset = -spacing * (n - 1) / 2.0
        for idx in range(n):
            x = offset + idx * spacing
            y = 0.0
            robot = RobotState(idx, x, y, 0.0, 0.0, 0.0, None, [], False, 0, 0)
            self.robots.append(robot)
            clear_world_disk(self.true_cells, self.grid, x, y, radius_m=max(0.35, self.grid.resolution * 2.0))

        self._observe_all_robots()
        self._last_coverage = self.grid.coverage_ratio()
        return self._observations(), self._infos(high_level={r.robot_id: True for r in self.robots})

    def current_clusters(self) -> list:
        """Frontier clusters for the grid as it stands right now.

        Cached per `current_step` so a caller that inspects clusters to pick
        an action (e.g. an evaluation policy) and `step()` itself don't each
        pay for their own full-grid frontier detection on the same state.
        """
        assert self.grid is not None
        if self._cluster_cache is not None and self._cluster_cache[0] == self.current_step:
            return self._cluster_cache[1]
        cells = self.detector.detect_frontiers(self.grid)
        clusters = self.detector.cluster_frontiers(cells)
        self._cluster_cache = (self.current_step, clusters)
        return clusters

    def step(self, actions: dict[str, int]):
        if self.grid is None:
            raise RuntimeError("reset must be called before step")

        clusters = self.current_clusters()
        cluster_by_id = {c.frontier_id: c for c in clusters}

        high_level: dict[int, bool] = {}
        comm_count: dict[int, int] = {}
        for robot in self.robots:
            target_exists = (
                robot.target_frontier_id in cluster_by_id
                if robot.target_frontier_id is not None
                else True
            )
            triggered = self.should_trigger_high_level(robot, target_exists=target_exists)
            high_level[robot.robot_id] = triggered
            comm_count[robot.robot_id] = 0
            if triggered:
                action = actions.get(self._agent_id(robot.robot_id))
                if isinstance(action, (int, np.integer)) and action >= 0:
                    robot.target_frontier_id = int(action)
                    if self.cfg["reward"].get("charge_target_assignment_comm", False):
                        comm_count[robot.robot_id] = 1
                elif action == RELEASE_ACTION:
                    robot.target_frontier_id = None
                    robot.path = []
                robot.steps_since_high_level = 0
                if action != CONTINUE_ACTION or not robot.path:
                    robot.path = self._plan_path_to_target(robot, cluster_by_id)
            else:
                robot.steps_since_high_level += 1

        was_in_collision = {robot.robot_id: robot.in_collision for robot in self.robots}
        frontier_bonus, path_cost, coverage_gain = self._simulate_low_level_step(cluster_by_id)
        collision_event = self._collision_events(was_in_collision)
        overlap = self._overlap_penalties()
        coverage = self.grid.coverage_ratio()
        self._last_coverage = coverage
        self.current_step += 1

        rewards = {
            self._agent_id(robot.robot_id): self.aggregate_reward(
                delta_coverage=coverage_gain[robot.robot_id],
                path_cost=path_cost[robot.robot_id],
                collision=collision_event[robot.robot_id],
                overlap=overlap[robot.robot_id],
                comm_count=comm_count[robot.robot_id],
                frontier_bonus=frontier_bonus[robot.robot_id],
            )
            for robot in self.robots
        }
        terminated_all = coverage >= self.cfg["env"]["coverage_done_thresh"]
        truncated_all = self.current_step >= self.cfg["env"]["max_episode_steps"]
        terminated = {self._agent_id(r.robot_id): terminated_all for r in self.robots}
        truncated = {self._agent_id(r.robot_id): truncated_all for r in self.robots}
        terminated["__all__"] = terminated_all
        truncated["__all__"] = truncated_all
        return self._observations(), rewards, terminated, truncated, self._infos(high_level)

    def should_trigger_high_level(self, robot: RobotState, target_exists: bool = True) -> bool:
        high_cfg = self.cfg["high_level"]
        if robot.target_frontier_id is None:
            return True
        if not target_exists:
            return True
        if robot.stuck_counter >= high_cfg["stuck_timeout"]:
            return True
        if robot.steps_since_high_level >= high_cfg["max_high_level_interval"]:
            return True
        return False

    def aggregate_reward(
        self,
        delta_coverage: float,
        path_cost: float,
        collision: bool,
        overlap: float,
        comm_count: int,
        frontier_bonus: float = 0.0,
    ) -> float:
        rew = self.cfg["reward"]
        return float(
            rew["w_coverage"] * delta_coverage
            - rew["w_path"] * path_cost
            - rew["w_collision"] * float(collision)
            - rew["w_overlap"] * overlap
            - rew["w_comm"] * comm_count
            + rew["w_frontier"] * frontier_bonus
            - rew["time_penalty"]
        )

    def _plan_path_to_target(self, robot: RobotState, cluster_by_id: dict) -> list[tuple[int, int]]:
        assert self.grid is not None
        if robot.target_frontier_id is None:
            return []
        cluster = cluster_by_id.get(robot.target_frontier_id)
        if cluster is None:
            return []
        free_cells = [cell for cell in cluster.cells if self.grid.is_free(*cell)]
        if not free_cells:
            return []
        robot_rc = self.grid.try_world_to_grid(robot.x, robot.y)
        if robot_rc is None:
            return []
        target_rc = min(
            free_cells,
            key=lambda rc: abs(rc[0] - robot_rc[0]) + abs(rc[1] - robot_rc[1]),
        )
        target_xy = self.grid.grid_to_world(*target_rc)
        return self.controller.plan_path(robot, target_xy, self.grid)

    def _overlap_penalties(self) -> dict[int, float]:
        penalties = {}
        for robot in self.robots:
            shared = robot.target_frontier_id is not None and any(
                other.robot_id != robot.robot_id and other.target_frontier_id == robot.target_frontier_id
                for other in self.robots
            )
            penalties[robot.robot_id] = 1.0 if shared else 0.0
        return penalties

    def _collision_events(self, was_in_collision: dict[int, bool]) -> dict[int, bool]:
        return {
            robot.robot_id: bool(robot.in_collision and not was_in_collision.get(robot.robot_id, False))
            for robot in self.robots
        }

    def _simulate_low_level_step(
        self, cluster_by_id: dict
    ) -> tuple[dict[int, float], dict[int, float], dict[int, float]]:
        assert self.grid is not None
        assert self.true_cells is not None
        dt = 0.1
        bound = -self.grid.origin - self.grid.resolution
        bonus: dict[int, float] = {}
        path_cost: dict[int, float] = {}
        coverage_gain: dict[int, float] = {}
        for robot in self.robots:
            others = [r for r in self.robots if r.robot_id != robot.robot_id]
            robot.in_collision = self.controller.is_colliding(robot, others) or not is_world_free(
                self.true_cells, self.grid, robot.x, robot.y
            )
            v, omega = self.controller.compute_command(
                robot, robot.path, self.grid, others, in_collision=robot.in_collision
            )
            robot.v, robot.omega = v, omega
            old_x, old_y = robot.x, robot.y
            next_x = float(np.clip(robot.x + v * math.cos(robot.theta) * dt, self.grid.origin, bound))
            next_y = float(np.clip(robot.y + v * math.sin(robot.theta) * dt, self.grid.origin, bound))
            if is_world_free(self.true_cells, self.grid, next_x, next_y):
                robot.x = next_x
                robot.y = next_y
                robot.theta = wrap_angle(robot.theta + omega * dt)
                robot.in_collision = self.controller.is_colliding(robot, others)
            else:
                robot.v = 0.0
                robot.in_collision = True
                robot.theta = wrap_angle(robot.theta + omega * dt)
            robot.stuck_counter = robot.stuck_counter + 1 if robot.in_collision else 0
            path_cost[robot.robot_id] = math.hypot(robot.x - old_x, robot.y - old_y)
            before_observe_coverage = self.grid.coverage_ratio()
            self._observe_robot(robot)
            coverage_gain[robot.robot_id] = max(0.0, self.grid.coverage_ratio() - before_observe_coverage)

            # Drop waypoints the robot has already reached so len(robot.path)
            # tracks remaining distance-to-go (what w_path is meant to cost)
            # instead of staying pinned at the full planned length for every
            # step until the next replan. Keep at least the final waypoint so
            # the arrival check below still has a goal to compare against.
            while len(robot.path) > 1:
                wx, wy = self.grid.grid_to_world(*robot.path[0])
                if math.hypot(wx - robot.x, wy - robot.y) > self.grid.resolution:
                    break
                robot.path.pop(0)

            bonus[robot.robot_id] = 0.0
            cluster = cluster_by_id.get(robot.target_frontier_id) if robot.target_frontier_id is not None else None
            if cluster is not None:
                if robot.path:
                    tx, ty = self.grid.grid_to_world(*robot.path[-1])
                else:
                    tx, ty = cluster.center_world(self.grid)
                if math.hypot(tx - robot.x, ty - robot.y) <= self.cfg["high_level"]["arrival_thresh_m"]:
                    bonus[robot.robot_id] = 1.0
                    robot.target_frontier_id = None
                    robot.path = []
        return bonus, path_cost, coverage_gain

    def _observe_all_robots(self) -> None:
        for robot in self.robots:
            self._observe_robot(robot)

    def _observe_robot(self, robot: RobotState) -> None:
        assert self.grid is not None
        assert self.true_cells is not None
        env_cfg = self.cfg["env"]
        ranges = raycast_ranges(
            self.true_cells,
            self.grid,
            (robot.x, robot.y, robot.theta),
            max_range=env_cfg["sensor_range_m"],
            n_rays=env_cfg["sensor_n_rays"],
        )
        self.grid.update_from_observation((robot.x, robot.y, robot.theta), ranges)

    def _observations(self) -> dict[str, dict]:
        assert self.grid is not None
        obs = {}
        for robot in self.robots:
            obs[self._agent_id(robot.robot_id)] = {
                "bev": self.grid.get_local_bev((robot.x, robot.y, robot.theta)),
                "robot_state": robot,
            }
        return obs

    def _infos(self, high_level: dict[int, bool]) -> dict[str, dict]:
        return {
            self._agent_id(robot.robot_id): {
                "coverage": self._last_coverage,
                "high_level_triggered": bool(high_level.get(robot.robot_id, False)),
            }
            for robot in self.robots
        }

    @staticmethod
    def _agent_id(robot_id: int) -> str:
        return f"robot_{robot_id}"
