import unittest

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv, RELEASE_ACTION
from eavf_mappo.evaluation.policies import choose_actions


class EnvStepTests(unittest.TestCase):
    def test_reset_and_step_use_multi_agent_dicts(self):
        env = MultiRobotExplorationEnv({"env": {"n_robots": 2, "map_size_m": 4.0, "resolution": 0.5}})

        obs, infos = env.reset(seed=123)
        next_obs, rewards, terminated, truncated, next_infos = env.step({"robot_0": 0, "robot_1": 1})

        self.assertEqual(set(obs.keys()), {"robot_0", "robot_1"})
        self.assertEqual(set(next_obs.keys()), {"robot_0", "robot_1"})
        self.assertEqual(set(rewards.keys()), {"robot_0", "robot_1"})
        self.assertIn("__all__", terminated)
        self.assertIn("__all__", truncated)
        self.assertIn("high_level_triggered", next_infos["robot_0"])
        self.assertIsInstance(rewards["robot_0"], float)

    def test_high_level_trigger_checks_interval_and_stuck(self):
        env = MultiRobotExplorationEnv({"env": {"n_robots": 1}, "high_level": {"max_high_level_interval": 3, "stuck_timeout": 2}})
        env.reset(seed=0)
        robot = env.robots[0]

        robot.steps_since_high_level = 3
        self.assertTrue(env.should_trigger_high_level(robot))

        robot.steps_since_high_level = 0
        robot.stuck_counter = 2
        self.assertTrue(env.should_trigger_high_level(robot))

    def test_sensor_loop_increases_coverage_as_robot_moves(self):
        env = MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 1,
                    "map_size_m": 6.0,
                    "resolution": 0.5,
                    "max_episode_steps": 40,
                    "sensor_range_m": 1.2,
                    "sensor_n_rays": 24,
                    "map_type": "open_room",
                },
                "high_level": {
                    "max_high_level_interval": 1,
                    "stuck_timeout": 5,
                },
            }
        )
        _, infos = env.reset(seed=2)
        start_coverage = infos["robot_0"]["coverage"]
        coverages = [start_coverage]

        for step in range(30):
            actions = choose_actions(env, "nearest_frontier", step=step)
            _, _, _, _, infos = env.step(actions)
            coverages.append(infos["robot_0"]["coverage"])

        self.assertGreater(max(coverages), start_coverage + 0.02)

    def test_frontier_action_creates_reachable_path(self):
        env = MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 1,
                    "map_size_m": 8.0,
                    "resolution": 0.5,
                    "sensor_range_m": 1.5,
                    "sensor_n_rays": 36,
                    "map_type": "open_room",
                }
            }
        )
        env.reset(seed=0)

        actions = choose_actions(env, "nearest_frontier", step=0)
        env.step(actions)

        self.assertGreater(len(env.robots[0].path), 0)

    def test_collision_penalty_applies_only_on_collision_entry(self):
        env = MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 1,
                    "map_size_m": 4.0,
                    "resolution": 0.5,
                    "sensor_range_m": 1.0,
                    "sensor_n_rays": 12,
                    "map_type": "open_room",
                },
                "reward": {
                    "w_coverage": 0.0,
                    "w_path": 0.0,
                    "w_overlap": 0.0,
                    "w_comm": 0.0,
                    "w_frontier": 0.0,
                    "time_penalty": 0.0,
                },
            }
        )
        env.reset(seed=0)
        robot = env.robots[0]
        rc = env.grid.try_world_to_grid(robot.x, robot.y)
        self.assertIsNotNone(rc)
        rr, cc = rc
        env.true_cells[rr - 1 : rr + 2, cc - 1 : cc + 2] = 1
        robot.in_collision = False

        _, rewards_1, _, _, _ = env.step({})
        _, rewards_2, _, _, _ = env.step({})

        self.assertLess(rewards_1["robot_0"], -4.0)
        self.assertGreater(rewards_2["robot_0"], rewards_1["robot_0"] + 4.0)
        self.assertEqual(rewards_2["robot_0"], 0.0)

    def test_reward_prefers_frontier_progress_over_release(self):
        config = {
            "env": {
                "n_robots": 2,
                "map_size_m": 8.0,
                "resolution": 0.5,
                "sensor_range_m": 1.5,
                "sensor_n_rays": 36,
                "map_type": "office",
                "max_episode_steps": 80,
            },
            "high_level": {
                "max_high_level_interval": 3,
                "stuck_timeout": 30,
            },
        }

        release_return, release_coverage = self._run_policy_episode(config, "release")
        frontier_return, frontier_coverage = self._run_policy_episode(config, "nearest_frontier")

        self.assertGreater(frontier_coverage, release_coverage + 0.1)
        self.assertGreater(frontier_return, release_return)

    def test_stationary_robot_does_not_receive_teammate_coverage_reward(self):
        env = MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 2,
                    "map_size_m": 8.0,
                    "resolution": 0.5,
                    "sensor_range_m": 1.5,
                    "sensor_n_rays": 36,
                    "map_type": "office",
                    "max_episode_steps": 20,
                },
                "high_level": {"max_high_level_interval": 1},
                "reward": {
                    "w_path": 0.0,
                    "w_collision": 0.0,
                    "w_overlap": 0.0,
                    "w_comm": 0.0,
                    "w_frontier": 0.0,
                    "time_penalty": 0.0,
                },
            }
        )
        env.reset(seed=0)
        start_coverage = env.grid.coverage_ratio()
        rewards_by_robot = {"robot_0": 0.0, "robot_1": 0.0}

        for step in range(12):
            actions = choose_actions(env, "nearest_frontier", step=step)
            actions["robot_0"] = RELEASE_ACTION
            _, rewards, terminated, truncated, _ = env.step(actions)
            for agent_id, reward in rewards.items():
                rewards_by_robot[agent_id] += reward
            if terminated["__all__"] or truncated["__all__"]:
                break

        self.assertGreater(env.grid.coverage_ratio(), start_coverage + 0.01)
        self.assertEqual(rewards_by_robot["robot_0"], 0.0)
        self.assertGreater(rewards_by_robot["robot_1"], 0.0)

    def _run_policy_episode(self, config, policy_name: str, max_steps: int = 80):
        env = MultiRobotExplorationEnv(config)
        env.reset(seed=0)
        total_return = 0.0
        coverage = env.grid.coverage_ratio()
        for step in range(max_steps):
            if policy_name == "release":
                actions = {env._agent_id(robot.robot_id): RELEASE_ACTION for robot in env.robots}
            else:
                actions = choose_actions(env, policy_name, step=step)
            _, rewards, terminated, truncated, infos = env.step(actions)
            total_return += sum(rewards.values())
            coverage = next(iter(infos.values()))["coverage"]
            if terminated["__all__"] or truncated["__all__"]:
                break
        return total_return, coverage


if __name__ == "__main__":
    unittest.main()
