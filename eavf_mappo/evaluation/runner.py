from __future__ import annotations

from collections import Counter

import numpy as np

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.evaluation.policies import choose_actions
from eavf_mappo.utils.config import merge_config


DEFAULT_EVAL_CONFIG = {
    "env": {
        "n_robots": 2,
        "map_size_m": 8.0,
        "resolution": 0.5,
        "max_episode_steps": 50,
        "sensor_range_m": 1.5,
        "sensor_n_rays": 36,
        "map_type": "open_room",
    },
    "high_level": {
        "max_high_level_interval": 10,
        "stuck_timeout": 5,
    },
}


def run_evaluation(
    policies: list[str],
    seeds: list[int],
    max_steps: int = 50,
    env_config: dict | None = None,
) -> list[dict]:
    rows: list[dict] = []
    for policy in policies:
        for seed in seeds:
            rows.extend(run_episode(policy, seed, max_steps=max_steps, env_config=env_config))
    return rows


def run_episode(
    policy_name: str,
    seed: int,
    max_steps: int = 50,
    env_config: dict | None = None,
) -> list[dict]:
    config = merge_config(DEFAULT_EVAL_CONFIG, env_config or {})
    config["env"]["max_episode_steps"] = min(int(max_steps), int(config["env"]["max_episode_steps"]))
    env = MultiRobotExplorationEnv(config)
    env.reset(seed=seed)

    run_id = f"{policy_name}_seed{seed}"
    rows: list[dict] = []
    cumulative_return = 0.0
    for step in range(max_steps):
        actions = choose_actions(env, policy_name, step=step)
        _, rewards, terminated, truncated, infos = env.step(actions)
        step_return = float(sum(rewards.values()))
        cumulative_return += step_return
        coverage = float(next(iter(infos.values()))["coverage"])
        row = {
            "run_id": run_id,
            "seed": int(seed),
            "policy": policy_name,
            "step": int(step),
            "coverage": coverage,
            "mean_reward": float(np.mean(list(rewards.values()))) if rewards else 0.0,
            "step_return": step_return,
            "episode_return": cumulative_return,
            "collision_count": _collision_count(env),
            "overlap_count": _overlap_count(env),
            "path_length": _path_length(env),
            "high_level_count": sum(1 for info in infos.values() if info.get("high_level_triggered")),
            "terminated": bool(terminated["__all__"]),
            "truncated": bool(truncated["__all__"]),
        }
        rows.append(row)
        if terminated["__all__"] or truncated["__all__"]:
            break
    return rows


def _collision_count(env) -> int:
    return sum(1 for robot in env.robots if robot.in_collision)


def _overlap_count(env) -> int:
    targets = [
        robot.target_frontier_id
        for robot in env.robots
        if robot.target_frontier_id is not None
    ]
    counts = Counter(targets)
    return sum(count - 1 for count in counts.values() if count > 1)


def _path_length(env) -> int:
    return sum(len(robot.path) for robot in env.robots)
