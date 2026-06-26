from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from eavf_mappo.envs.multi_robot_exploration_env import CONTINUE_ACTION
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.exploration.frontier_token import TOKEN_DIM
from eavf_mappo.marl.action_adapter import decode_action
from eavf_mappo.marl.actor import select_action
from eavf_mappo.marl.rollout_buffer import RolloutBatch, RolloutBuffer


@dataclass
class DecisionInputs:
    bev: torch.Tensor
    tokens: torch.Tensor
    c_analytic: torch.Tensor
    mask: torch.Tensor
    special_mask: torch.Tensor
    topk_tokens: list
    global_bev: torch.Tensor
    all_poses: torch.Tensor
    all_targets: torch.Tensor
    coverage: torch.Tensor
    collision: torch.Tensor
    comm_feat: torch.Tensor


def build_decision_inputs(env, robot, voronoi: ExecutionAwareVoronoi, k: int, device=None) -> DecisionInputs:
    assert env.grid is not None
    device = device or torch.device("cpu")
    clusters = env.current_clusters()
    other_robots = [other for other in env.robots if other.robot_id != robot.robot_id]
    tokens = env.detector.build_frontier_tokens(
        robot,
        clusters,
        env.grid,
        other_robots,
        messages=[],
        comm_aware=False,
    )
    topk_tokens = env.detector.select_top_k(tokens, k)
    cost = voronoi.compute_cost_matrix([robot], [topk_tokens], grid=env.grid)[0]
    mask_np = np.asarray([tok.valid and np.isfinite(cost[idx]) for idx, tok in enumerate(topk_tokens)], dtype=bool)
    cost_np = np.asarray(cost, dtype=np.float32)
    cost_np[~np.isfinite(cost_np)] = 0.0
    special_mask_np = _special_action_mask(robot, has_valid_frontier=bool(mask_np.any()))

    token_np = np.stack([tok.to_vector() for tok in topk_tokens], axis=0).astype(np.float32)
    bev_np = env.grid.get_local_bev((robot.x, robot.y, robot.theta)).astype(np.float32)
    global_bev_np = env.grid.get_global_bev(env.robots).astype(np.float32)

    return DecisionInputs(
        bev=torch.as_tensor(bev_np, dtype=torch.float32, device=device),
        tokens=torch.as_tensor(token_np, dtype=torch.float32, device=device),
        c_analytic=torch.as_tensor(cost_np, dtype=torch.float32, device=device),
        mask=torch.as_tensor(mask_np, dtype=torch.bool, device=device),
        special_mask=torch.as_tensor(special_mask_np, dtype=torch.bool, device=device),
        topk_tokens=topk_tokens,
        global_bev=torch.as_tensor(global_bev_np, dtype=torch.float32, device=device),
        all_poses=_all_poses(env, device),
        all_targets=_all_targets(env, device),
        coverage=torch.as_tensor([env.grid.coverage_ratio()], dtype=torch.float32, device=device),
        collision=torch.as_tensor([float(r.in_collision) for r in env.robots], dtype=torch.float32, device=device),
        comm_feat=torch.as_tensor([0.0], dtype=torch.float32, device=device),
    )


def build_joint_decision_inputs(
    env,
    robots,
    voronoi: ExecutionAwareVoronoi,
    k: int,
    device=None,
    assignment_penalty: float = 2.0,
) -> list[DecisionInputs]:
    robots = list(robots)
    decisions = [build_decision_inputs(env, robot, voronoi, k, device=device) for robot in robots]
    if len(decisions) <= 1:
        return decisions

    cost_matrix = voronoi.compute_cost_matrix(
        robots,
        [decision.topk_tokens for decision in decisions],
        grid=env.grid,
    )
    assignment = voronoi.assign_unique_frontiers(
        cost_matrix,
        [decision.topk_tokens for decision in decisions],
    )
    for idx, decision in enumerate(decisions):
        shaped_cost = np.asarray(cost_matrix[idx], dtype=np.float32).copy()
        assigned_idx = assignment.get(idx, -1)
        if assigned_idx >= 0:
            for token_idx, token in enumerate(decision.topk_tokens):
                if token_idx == assigned_idx or not getattr(token, "valid", False):
                    continue
                if np.isfinite(shaped_cost[token_idx]):
                    shaped_cost[token_idx] += float(assignment_penalty)
        decision.c_analytic = torch.as_tensor(shaped_cost, dtype=torch.float32, device=decision.c_analytic.device)
    return decisions


def collect_rollout(
    env,
    actor,
    critic,
    voronoi: ExecutionAwareVoronoi,
    rollout_length: int,
    k: int,
    device=None,
    training: bool = True,
) -> list[RolloutBuffer]:
    device = device or torch.device("cpu")
    buffers = [RolloutBuffer() for _ in env.robots]
    pending: dict[int, dict] = {}

    for _ in range(rollout_length):
        high_level = _high_level_triggers(env)
        env_actions = {env._agent_id(robot.robot_id): CONTINUE_ACTION for robot in env.robots}
        triggered_indices = [
            idx for idx, robot in enumerate(env.robots) if high_level.get(robot.robot_id, False)
        ]

        for idx in triggered_indices:
            _flush_pending_transition(buffers, pending, idx, done=False, device=device)

        if triggered_indices:
            triggered_robots = [env.robots[idx] for idx in triggered_indices]
            decisions = build_joint_decision_inputs(env, triggered_robots, voronoi, k, device=device)
            bev = torch.stack([d.bev for d in decisions], dim=0)
            tokens = torch.stack([d.tokens for d in decisions], dim=0)
            c_analytic = torch.stack([d.c_analytic for d in decisions], dim=0)
            mask = torch.stack([d.mask for d in decisions], dim=0)
            special_mask = torch.stack([d.special_mask for d in decisions], dim=0)
            global_bev = torch.stack([d.global_bev for d in decisions], dim=0)
            all_poses = torch.stack([d.all_poses for d in decisions], dim=0)
            all_targets = torch.stack([d.all_targets for d in decisions], dim=0)
            coverage = torch.stack([d.coverage for d in decisions], dim=0)
            collision = torch.stack([d.collision for d in decisions], dim=0)
            comm_feat = torch.stack([d.comm_feat for d in decisions], dim=0)

            with torch.no_grad():
                actions, logp, _ = select_action(
                    actor,
                    bev,
                    tokens,
                    c_analytic,
                    mask,
                    special_mask=special_mask,
                    training=training,
                )
                values = critic(global_bev, all_poses, all_targets, coverage, collision, comm_feat)

            for local_idx, robot_idx in enumerate(triggered_indices):
                robot = env.robots[robot_idx]
                env_actions[env._agent_id(robot.robot_id)] = decode_action(
                    actions[local_idx].item(),
                    decisions[local_idx].topk_tokens,
                )
                pending[robot_idx] = {
                    "reward": 0.0,
                    "batch": RolloutBatch(
                        bev=bev[local_idx : local_idx + 1].detach(),
                        tokens=tokens[local_idx : local_idx + 1].detach(),
                        c_analytic=c_analytic[local_idx : local_idx + 1].detach(),
                        mask=mask[local_idx : local_idx + 1].detach(),
                        actions=actions[local_idx : local_idx + 1].detach(),
                        old_logp=logp[local_idx : local_idx + 1].detach(),
                        rewards=torch.zeros(1, dtype=torch.float32, device=device),
                        dones=torch.zeros(1, dtype=torch.float32, device=device),
                        values=values[local_idx : local_idx + 1].detach(),
                        special_mask=special_mask[local_idx : local_idx + 1].detach(),
                        global_bev=global_bev[local_idx : local_idx + 1].detach(),
                        all_poses=all_poses[local_idx : local_idx + 1].detach(),
                        all_targets=all_targets[local_idx : local_idx + 1].detach(),
                        coverage=coverage[local_idx : local_idx + 1].detach(),
                        collision=collision[local_idx : local_idx + 1].detach(),
                        comm_feat=comm_feat[local_idx : local_idx + 1].detach(),
                    ),
                }

        _, rewards, terminated, truncated, _ = env.step(env_actions)
        done_all = bool(terminated["__all__"] or truncated["__all__"])

        for idx, robot in enumerate(env.robots):
            agent_id = env._agent_id(robot.robot_id)
            if idx in pending:
                pending[idx]["reward"] += float(rewards[agent_id])

        if done_all:
            for idx in list(pending):
                _flush_pending_transition(buffers, pending, idx, done=True, device=device)
            env.reset()
            voronoi.grid = env.grid
            pending.clear()

    for idx in list(pending):
        _flush_pending_transition(buffers, pending, idx, done=False, device=device)
    return buffers


def _high_level_triggers(env) -> dict[int, bool]:
    clusters = env.current_clusters()
    cluster_by_id = {cluster.frontier_id: cluster for cluster in clusters}
    triggers: dict[int, bool] = {}
    for robot in env.robots:
        target_exists = (
            robot.target_frontier_id in cluster_by_id
            if robot.target_frontier_id is not None
            else True
        )
        triggers[robot.robot_id] = env.should_trigger_high_level(robot, target_exists=target_exists)
    return triggers


def _special_action_mask(robot, has_valid_frontier: bool) -> np.ndarray:
    has_active_target = robot.target_frontier_id is not None or bool(robot.path)
    if has_active_target or not has_valid_frontier:
        return np.asarray([True, True], dtype=bool)
    return np.asarray([False, False], dtype=bool)


def _flush_pending_transition(
    buffers: list[RolloutBuffer],
    pending: dict[int, dict],
    idx: int,
    done: bool,
    device,
) -> None:
    item = pending.pop(idx, None)
    if item is None:
        return
    batch = item["batch"]
    batch.rewards = torch.as_tensor([item["reward"]], dtype=torch.float32, device=device)
    batch.dones = torch.as_tensor([float(done)], dtype=torch.float32, device=device)
    buffers[idx].add(batch)


def _all_poses(env, device) -> torch.Tensor:
    values = []
    for robot in env.robots:
        values.extend([robot.x, robot.y, robot.theta])
    return torch.as_tensor(values, dtype=torch.float32, device=device)


def _all_targets(env, device) -> torch.Tensor:
    values = []
    for robot in env.robots:
        values.append(-1.0 if robot.target_frontier_id is None else float(robot.target_frontier_id))
    return torch.as_tensor(values, dtype=torch.float32, device=device)
