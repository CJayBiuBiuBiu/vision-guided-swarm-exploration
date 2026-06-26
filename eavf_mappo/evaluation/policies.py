from __future__ import annotations

import math


def choose_actions(env, policy_name: str, step: int = 0) -> dict[str, int]:
    """Choose frontier ids for the current lightweight env state."""
    policy_name = policy_name.lower()
    clusters = _current_clusters(env)
    if not clusters:
        return {env._agent_id(robot.robot_id): -1 for robot in env.robots}

    if policy_name == "constant":
        return {env._agent_id(robot.robot_id): clusters[0].frontier_id for robot in env.robots}
    if policy_name == "round_robin":
        return {
            env._agent_id(robot.robot_id): clusters[(robot.robot_id + step) % len(clusters)].frontier_id
            for robot in env.robots
        }
    if policy_name == "nearest_frontier":
        return _nearest_unique_actions(env, clusters)

    raise ValueError(f"unknown evaluation policy: {policy_name}")


def _current_clusters(env):
    return env.current_clusters()


def _nearest_unique_actions(env, clusters) -> dict[str, int]:
    unused = list(clusters)
    actions: dict[str, int] = {}
    for robot in env.robots:
        if unused:
            cluster = min(unused, key=lambda c: _distance_to_cluster(robot, c, env.grid))
            unused.remove(cluster)
        else:
            cluster = min(clusters, key=lambda c: _distance_to_cluster(robot, c, env.grid))
        actions[env._agent_id(robot.robot_id)] = cluster.frontier_id
    return actions


def _distance_to_cluster(robot, cluster, grid) -> float:
    cx, cy = cluster.center_world(grid)
    return math.hypot(robot.x - cx, robot.y - cy)
