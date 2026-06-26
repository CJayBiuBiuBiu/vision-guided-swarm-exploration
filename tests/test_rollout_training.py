import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import torch
from torch import nn

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.marl.actor import ResidualFrontierActor
from eavf_mappo.marl.critic import CentralizedCritic
from eavf_mappo.marl.mappo import MAPPOConfig, update
from eavf_mappo.marl.rollout import build_decision_inputs, build_joint_decision_inputs, collect_rollout


class FrontierOnlyActor(nn.Module):
    def forward(self, bev, tokens, c_analytic, mask):
        batch, k = c_analytic.shape
        return torch.zeros(batch, k), torch.full((batch, 2), -100.0)


class RolloutTrainingTests(unittest.TestCase):
    def _env(self):
        return MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 2,
                    "map_size_m": 6.0,
                    "resolution": 0.5,
                    "sensor_range_m": 1.2,
                    "sensor_n_rays": 24,
                    "map_type": "office",
                    "max_episode_steps": 20,
                },
                "high_level": {"frontier_K": 4, "max_high_level_interval": 2},
            }
        )

    def test_decision_inputs_use_real_voronoi_cost(self):
        env = self._env()
        env.reset(seed=0)
        voronoi = ExecutionAwareVoronoi({}, grid=env.grid)

        decision = build_decision_inputs(env, env.robots[0], voronoi, k=4)

        self.assertEqual(decision.tokens.shape, (4, 10))
        self.assertEqual(decision.c_analytic.shape, (4,))
        self.assertEqual(decision.mask.shape, (4,))
        self.assertTrue(torch.isfinite(decision.c_analytic[decision.mask]).all())
        self.assertGreater(decision.mask.sum().item(), 0)
        self.assertEqual(decision.special_mask.tolist(), [False, False])

        env.robots[0].target_frontier_id = 0
        decision_with_target = build_decision_inputs(env, env.robots[0], voronoi, k=4)
        self.assertEqual(decision_with_target.special_mask.tolist(), [True, True])

    def test_joint_decision_inputs_bias_robots_toward_unique_frontiers(self):
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
                "high_level": {"frontier_K": 4, "max_high_level_interval": 2},
            }
        )
        env.reset(seed=0)
        voronoi = ExecutionAwareVoronoi({}, grid=env.grid)

        decisions = build_joint_decision_inputs(env, env.robots, voronoi, k=4)
        preferred_frontiers = []
        for decision in decisions:
            costs = decision.c_analytic.masked_fill(~decision.mask, float("inf"))
            best_idx = int(torch.argmin(costs).item())
            preferred_frontiers.append(decision.topk_tokens[best_idx].frontier_id)

        self.assertEqual(len(preferred_frontiers), 2)
        self.assertEqual(len(set(preferred_frontiers)), 2)

    def test_collect_rollout_returns_one_time_ordered_buffer_per_agent_and_update_runs(self):
        env = self._env()
        env.reset(seed=0)
        actor = ResidualFrontierActor(d_token=10, d_model=16, tau=0.5)
        critic = CentralizedCritic(d_model=16)
        voronoi = ExecutionAwareVoronoi({}, grid=env.grid)

        buffers = collect_rollout(env, actor, critic, voronoi, rollout_length=2, k=4)

        self.assertEqual(len(buffers), 2)
        self.assertTrue(all(0 < len(buffer.items) <= 2 for buffer in buffers))
        opt = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=1.0e-3)
        stats = update(
            buffers,
            actor,
            critic,
            opt,
            MAPPOConfig(update_epochs=1, minibatch_size=4),
        )

        self.assertTrue(stats)
        self.assertTrue(all(torch.isfinite(torch.tensor(item["loss"])) for item in stats))

    def test_collect_rollout_records_only_executed_high_level_decisions(self):
        env = MultiRobotExplorationEnv(
            {
                "env": {
                    "n_robots": 2,
                    "map_size_m": 6.0,
                    "resolution": 0.5,
                    "sensor_range_m": 1.2,
                    "sensor_n_rays": 24,
                    "map_type": "office",
                    "max_episode_steps": 20,
                },
                "high_level": {"frontier_K": 4, "max_high_level_interval": 99},
            }
        )
        env.reset(seed=0)
        actor = FrontierOnlyActor()
        critic = CentralizedCritic(d_model=16)
        voronoi = ExecutionAwareVoronoi({}, grid=env.grid)

        buffers = collect_rollout(env, actor, critic, voronoi, rollout_length=4, k=4, training=False)

        self.assertTrue(all(len(buffer.items) == 1 for buffer in buffers))
        self.assertTrue(all(buffer.items[0].rewards.item() != 0.0 for buffer in buffers))

    def test_train_script_smoke_writes_checkpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/train.py",
                    "--iterations",
                    "1",
                    "--rollout-length",
                    "2",
                    "--n-robots",
                    "2",
                    "--map-size",
                    "6",
                    "--sensor-range",
                    "1.2",
                    "--d-model",
                    "16",
                    "--output-dir",
                    tmp,
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=60,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((Path(tmp) / "checkpoint.pt").exists())
            self.assertTrue((Path(tmp) / "train_log.csv").exists())


if __name__ == "__main__":
    unittest.main()
