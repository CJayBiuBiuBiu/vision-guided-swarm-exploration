from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from eavf_mappo.envs.multi_robot_exploration_env import CONTINUE_ACTION, MultiRobotExplorationEnv
from eavf_mappo.evaluation.policies import choose_actions
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.marl.action_adapter import decode_action
from eavf_mappo.marl.actor import ResidualFrontierActor, select_action
from eavf_mappo.marl.rollout import build_joint_decision_inputs
from eavf_mappo.mapping.occupancy_grid import FREE, OCCUPIED, UNKNOWN


def render_frame(env, ax) -> None:
    import matplotlib.colors as mcolors

    grid = env.grid
    if grid is None:
        raise RuntimeError("env must be reset before rendering")

    ax.clear()
    display = np.zeros_like(grid.cells, dtype=np.int8)
    display[grid.cells == UNKNOWN] = 0
    display[grid.cells == FREE] = 1
    display[grid.cells == OCCUPIED] = 2
    cmap = mcolors.ListedColormap(["#8a8f98", "#f5f7fb", "#20242c"])
    extent = (grid.origin, -grid.origin, grid.origin, -grid.origin)
    ax.imshow(display, origin="lower", extent=extent, cmap=cmap, vmin=0, vmax=2)

    clusters = {cluster.frontier_id: cluster for cluster in env.current_clusters()}
    for robot in env.robots:
        color = f"C{robot.robot_id % 10}"
        if robot.path:
            pts = [grid.grid_to_world(r, c) for r, c in robot.path]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.75)

        if robot.target_frontier_id is not None and robot.target_frontier_id in clusters:
            cluster = clusters[robot.target_frontier_id]
            pts = [grid.grid_to_world(r, c) for r, c in cluster.cells]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.scatter(xs, ys, color=color, marker="s", s=10, alpha=0.35)

        ax.scatter([robot.x], [robot.y], color=color, edgecolor="black", s=55, zorder=4)
        heading_len = max(grid.resolution * 1.5, 0.2)
        ax.arrow(
            robot.x,
            robot.y,
            heading_len * np.cos(robot.theta),
            heading_len * np.sin(robot.theta),
            color=color,
            width=0.015,
            head_width=0.12,
            length_includes_head=True,
            zorder=5,
        )
        ax.text(robot.x, robot.y, str(robot.robot_id), color="black", ha="center", va="center", fontsize=7, zorder=6)

    ax.set_xlim(grid.origin, -grid.origin)
    ax.set_ylim(grid.origin, -grid.origin)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"step={env.current_step} coverage={grid.coverage_ratio():.3f}")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def record_episode(
    env,
    policy_name: str,
    max_steps: int,
    output_path: str | Path,
    fps: int = 6,
    seed: int | None = None,
    checkpoint: str | Path | None = None,
    frontier_k: int = 8,
    frame_stride: int = 1,
    device=None,
) -> None:
    if frame_stride < 1:
        raise ValueError("frame_stride must be >= 1")

    with _plotting_context() as plt:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        env.reset(seed=seed)
        actor = None
        voronoi = ExecutionAwareVoronoi({}, grid=env.grid)
        if checkpoint is not None:
            actor = _load_actor(checkpoint, device=device)
            policy_name = "trained"

        frames = []
        fig, ax = plt.subplots(figsize=(6, 6))
        render_frame(env, ax)
        frames.append(_frame_image(fig))
        for step in range(max_steps):
            if actor is None:
                actions = choose_actions(env, policy_name, step=step)
            else:
                actions = _trained_actions(env, actor, voronoi, frontier_k=frontier_k, device=device)
            _, _, terminated, truncated, _ = env.step(actions)
            should_stop = terminated["__all__"] or truncated["__all__"]
            step_number = step + 1
            if step_number % frame_stride == 0 or should_stop:
                render_frame(env, ax)
                frames.append(_frame_image(fig))
            if should_stop:
                break
        plt.close(fig)

        if _can_write_gif():
            _write_gif(frames, output_path, fps=fps)
        else:
            _write_png_frames(frames, output_path)


def _trained_actions(env, actor, voronoi, frontier_k: int, device=None) -> dict[str, int]:
    actions = {env._agent_id(robot.robot_id): CONTINUE_ACTION for robot in env.robots}
    triggered = _high_level_indices(env)
    if not triggered:
        return actions
    decisions = build_joint_decision_inputs(env, [env.robots[idx] for idx in triggered], voronoi, frontier_k, device=device)
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
                training=False,
            )
        actions[env._agent_id(robot.robot_id)] = decode_action(int(action.item()), decision.topk_tokens)
    return actions


def _high_level_indices(env) -> list[int]:
    clusters = env.current_clusters()
    cluster_by_id = {cluster.frontier_id: cluster for cluster in clusters}
    indices = []
    for idx, robot in enumerate(env.robots):
        target_exists = (
            robot.target_frontier_id in cluster_by_id
            if robot.target_frontier_id is not None
            else True
        )
        if env.should_trigger_high_level(robot, target_exists=target_exists):
            indices.append(idx)
    return indices


def _load_actor(checkpoint: str | Path, device=None):
    checkpoint_data = torch.load(checkpoint, map_location=device or "cpu")
    args = checkpoint_data.get("args", {})
    actor = ResidualFrontierActor(
        d_token=10,
        d_model=int(args.get("d_model", 64)),
        tau=float(args.get("tau", 0.5)),
    )
    actor.load_state_dict(checkpoint_data["actor"])
    actor.eval()
    return actor


def _frame_image(fig) -> np.ndarray:
    fig.canvas.draw()
    width, height = fig.canvas.get_width_height()
    return np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(height, width, 3).copy()


def _can_write_gif() -> bool:
    try:
        from matplotlib.animation import PillowWriter  # noqa: F401
        import PIL  # noqa: F401
    except Exception:
        return False
    return True


def _write_gif(frames: list[np.ndarray], output_path: Path, fps: int) -> None:
    from PIL import Image

    images = [Image.fromarray(frame) for frame in frames]
    duration_ms = max(1, int(1000 / max(fps, 1)))
    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=0,
    )


def _write_png_frames(frames: list[np.ndarray], output_path: Path) -> None:
    from matplotlib import pyplot as plt

    stem_dir = output_path.parent / output_path.stem
    stem_dir.mkdir(parents=True, exist_ok=True)
    for idx, frame in enumerate(frames):
        plt.imsave(stem_dir / f"frame_{idx:04d}.png", frame)


class _plotting_context:
    def __enter__(self):
        import warnings

        self._warnings = warnings.catch_warnings()
        self._warnings.__enter__()
        warnings.filterwarnings("ignore", message=".*deprecated.*", category=Warning)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt

    def __exit__(self, exc_type, exc, tb):
        return self._warnings.__exit__(exc_type, exc, tb)
