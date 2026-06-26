from __future__ import annotations

from collections import Counter
from pathlib import Path

import torch

from eavf_mappo.envs.multi_robot_exploration_env import CONTINUE_ACTION, RELEASE_ACTION
from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.marl.action_adapter import decode_action
from eavf_mappo.marl.actor import ResidualFrontierActor, select_action
from eavf_mappo.marl.rollout import build_joint_decision_inputs


def load_actor_from_checkpoint(checkpoint_path: str | Path, device=None):
    device = device or torch.device("cpu")
    checkpoint = torch.load(checkpoint_path, map_location=device)
    args = checkpoint.get("args", {})
    actor = ResidualFrontierActor(
        d_token=10,
        d_model=int(args.get("d_model", 64)),
        tau=float(args.get("tau", 0.5)),
    )
    actor.load_state_dict(checkpoint["actor"])
    actor.to(device)
    actor.eval()
    return actor, args


def run_checkpoint_evaluation(
    checkpoint_path: str | Path,
    seeds: list[int],
    map_types: list[str],
    max_steps: int,
    env_config: dict,
    frontier_k: int,
    stochastic: bool = False,
    device=None,
) -> tuple[list[dict], list[dict]]:
    device = device or torch.device("cpu")
    actor, _ = load_actor_from_checkpoint(checkpoint_path, device=device)
    checkpoint_name = Path(checkpoint_path).name

    step_rows: list[dict] = []
    summary_rows: list[dict] = []
    for map_type in map_types:
        for seed in seeds:
            rows = run_checkpoint_episode(
                actor=actor,
                checkpoint_name=checkpoint_name,
                seed=int(seed),
                map_type=map_type,
                max_steps=max_steps,
                env_config=env_config,
                frontier_k=frontier_k,
                stochastic=stochastic,
                device=device,
            )
            step_rows.extend(rows)
            summary_rows.append(summarize_checkpoint_episode(rows))
    return step_rows, summary_rows


def run_checkpoint_episode(
    actor,
    checkpoint_name: str,
    seed: int,
    map_type: str,
    max_steps: int,
    env_config: dict,
    frontier_k: int,
    stochastic: bool = False,
    device=None,
) -> list[dict]:
    device = device or torch.device("cpu")
    config = _with_map_type(env_config, map_type, max_steps)
    env = MultiRobotExplorationEnv(config)
    env.reset(seed=seed)
    voronoi = ExecutionAwareVoronoi({}, grid=env.grid)

    run_id = f"{Path(checkpoint_name).stem}_{map_type}_seed{seed}"
    rows: list[dict] = []
    episode_return = 0.0
    action_idx_counts: Counter[int] = Counter()
    env_action_counts: Counter[int] = Counter()

    for step in range(max_steps):
        env_actions, chosen_indices = _actor_actions(
            env,
            actor,
            voronoi,
            frontier_k=frontier_k,
            stochastic=stochastic,
            device=device,
        )
        action_idx_counts.update(chosen_indices)
        env_action_counts.update(env_actions.values())

        _, rewards, terminated, truncated, infos = env.step(env_actions)
        step_return = float(sum(rewards.values()))
        episode_return += step_return
        coverage = float(next(iter(infos.values()))["coverage"])
        row = {
            "checkpoint": checkpoint_name,
            "run_id": run_id,
            "map_type": map_type,
            "seed": int(seed),
            "step": int(step),
            "coverage": coverage,
            "step_return": step_return,
            "episode_return": episode_return,
            "collision_count": _collision_count(env),
            "overlap_count": _overlap_count(env),
            "path_length": _path_length(env),
            "high_level_count": len(chosen_indices),
            "frontier_action_count": sum(1 for action in env_actions.values() if action >= 0),
            "continue_action_count": sum(1 for action in env_actions.values() if action == CONTINUE_ACTION),
            "release_action_count": sum(1 for action in env_actions.values() if action == RELEASE_ACTION),
            "terminated": bool(terminated["__all__"]),
            "truncated": bool(truncated["__all__"]),
            "action_idx_distribution": _format_counter(action_idx_counts),
            "env_action_distribution": _format_counter(env_action_counts),
        }
        rows.append(row)
        if terminated["__all__"] or truncated["__all__"]:
            break
    return rows


def summarize_checkpoint_episode(rows: list[dict]) -> dict:
    if not rows:
        raise ValueError("cannot summarize an empty episode")
    last = rows[-1]
    return {
        "checkpoint": last["checkpoint"],
        "run_id": last["run_id"],
        "map_type": last["map_type"],
        "seed": last["seed"],
        "steps": len(rows),
        "final_coverage": last["coverage"],
        "total_return": last["episode_return"],
        "collision_total": sum(int(row["collision_count"]) for row in rows),
        "overlap_total": sum(int(row["overlap_count"]) for row in rows),
        "high_level_total": sum(int(row["high_level_count"]) for row in rows),
        "frontier_action_total": sum(int(row["frontier_action_count"]) for row in rows),
        "continue_action_total": sum(int(row["continue_action_count"]) for row in rows),
        "release_action_total": sum(int(row["release_action_count"]) for row in rows),
        "terminated": last["terminated"],
        "truncated": last["truncated"],
        "action_idx_distribution": last["action_idx_distribution"],
        "env_action_distribution": last["env_action_distribution"],
    }


def _actor_actions(env, actor, voronoi, frontier_k: int, stochastic: bool, device) -> tuple[dict[str, int], list[int]]:
    env_actions = {env._agent_id(robot.robot_id): CONTINUE_ACTION for robot in env.robots}
    triggered = _high_level_indices(env)
    if not triggered:
        return env_actions, []

    robots = [env.robots[idx] for idx in triggered]
    decisions = build_joint_decision_inputs(env, robots, voronoi, frontier_k, device=device)
    chosen_indices: list[int] = []
    for local_idx, robot_idx in enumerate(triggered):
        robot = env.robots[robot_idx]
        decision = decisions[local_idx]
        with torch.no_grad():
            action, _, _ = select_action(
                actor,
                decision.bev.unsqueeze(0),
                decision.tokens.unsqueeze(0),
                decision.c_analytic.unsqueeze(0),
                decision.mask.unsqueeze(0),
                special_mask=decision.special_mask.unsqueeze(0),
                training=stochastic,
            )
        action_idx = int(action.item())
        chosen_indices.append(action_idx)
        env_actions[env._agent_id(robot.robot_id)] = decode_action(action_idx, decision.topk_tokens)
    return env_actions, chosen_indices


def _high_level_indices(env) -> list[int]:
    clusters = env.current_clusters()
    cluster_by_id = {cluster.frontier_id: cluster for cluster in clusters}
    indices: list[int] = []
    for idx, robot in enumerate(env.robots):
        target_exists = (
            robot.target_frontier_id in cluster_by_id
            if robot.target_frontier_id is not None
            else True
        )
        if env.should_trigger_high_level(robot, target_exists=target_exists):
            indices.append(idx)
    return indices


def _with_map_type(env_config: dict, map_type: str, max_steps: int) -> dict:
    config = {
        key: (value.copy() if isinstance(value, dict) else value)
        for key, value in env_config.items()
    }
    config.setdefault("env", {})
    config["env"]["map_type"] = map_type
    config["env"]["max_episode_steps"] = max_steps
    return config


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


def _format_counter(counter: Counter[int]) -> str:
    total = sum(counter.values())
    if total == 0:
        return ""
    return ";".join(f"{key}:{counter[key]}" for key in sorted(counter))
