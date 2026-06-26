from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import torch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.marl.actor import ResidualFrontierActor
from eavf_mappo.marl.critic import CentralizedCritic
from eavf_mappo.marl.mappo import MAPPOConfig, update
from eavf_mappo.marl.rollout import collect_rollout
from eavf_mappo.utils.seed import set_seed


def main() -> None:
    args = parse_args()
    set_seed(args.seed)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    env = MultiRobotExplorationEnv(
        {
            "env": {
                "n_robots": args.n_robots,
                "map_size_m": args.map_size,
                "resolution": args.resolution,
                "sensor_range_m": args.sensor_range,
                "sensor_n_rays": args.sensor_rays,
                "map_type": args.map_type,
                "max_episode_steps": args.max_episode_steps,
            },
            "high_level": {
                "frontier_K": args.frontier_k,
                "max_high_level_interval": args.max_high_level_interval,
            },
        }
    )
    env.reset(seed=args.seed)
    actor = ResidualFrontierActor(d_token=10, d_model=args.d_model, tau=args.tau)
    critic = CentralizedCritic(d_model=args.d_model)
    voronoi = ExecutionAwareVoronoi({}, grid=env.grid)
    opt = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=args.lr)
    cfg = MAPPOConfig(
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_eps=args.clip_eps,
        entropy_coef=args.entropy_coef,
        value_coef=args.value_coef,
        residual_reg=args.residual_reg,
        update_epochs=args.update_epochs,
        minibatch_size=args.minibatch_size,
        max_grad_norm=args.max_grad_norm,
        normalize_returns=not args.no_normalize_returns,
        value_clip_eps=None if args.value_clip_eps <= 0.0 else args.value_clip_eps,
    )

    lines = [
        "EA-VF-MAPPO training diagnostics",
        f"seed={args.seed}",
        f"normalize_returns={cfg.normalize_returns}",
        f"value_clip_eps={cfg.value_clip_eps}",
    ]
    action_counts: Counter[int] = Counter()

    for iteration in range(args.iterations):
        buffers = collect_rollout(
            env,
            actor,
            critic,
            voronoi,
            rollout_length=args.rollout_length,
            k=args.frontier_k,
            training=True,
        )
        for buffer in buffers:
            for item in buffer.items:
                action_counts.update(int(action) for action in item.actions.detach().cpu().reshape(-1).tolist())

        stats = update(buffers, actor, critic, opt, cfg)
        last = stats[-1] if stats else {}
        lines.append(
            " ".join(
                [
                    f"iter={iteration}",
                    f"coverage={env.grid.coverage_ratio():.6f}",
                    f"value_loss={last.get('value_loss', 0.0):.6f}",
                    f"actor_loss={last.get('actor_loss', 0.0):.6f}",
                    f"entropy={last.get('entropy', 0.0):.6f}",
                    f"return_mean={last.get('return_mean', 0.0):.6f}",
                    f"return_std={last.get('return_std', 0.0):.6f}",
                    f"value_pred_mean={last.get('value_pred_mean', 0.0):.6f}",
                    f"approx_kl={last.get('approx_kl', 0.0):.6f}",
                ]
            )
        )

    total_actions = sum(action_counts.values())
    if total_actions:
        action_summary = ", ".join(
            f"{action}:{count / total_actions:.3f}" for action, count in sorted(action_counts.items())
        )
        lines.append(f"action_distribution={action_summary}")
    else:
        lines.append("action_distribution=empty")

    report = out_dir / "diagnostics.txt"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {report}")


def parse_args():
    parser = argparse.ArgumentParser(description="Run a short EA-VF-MAPPO stability diagnostic.")
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--rollout-length", type=int, default=4)
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--map-size", type=float, default=6.0)
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--sensor-range", type=float, default=1.2)
    parser.add_argument("--sensor-rays", type=int, default=36)
    parser.add_argument("--map-type", default="office", choices=["open_room", "corridor", "maze", "office", "cluttered"])
    parser.add_argument("--max-episode-steps", type=int, default=60)
    parser.add_argument("--frontier-k", type=int, default=8)
    parser.add_argument("--max-high-level-interval", type=int, default=3)
    parser.add_argument("--d-model", type=int, default=16)
    parser.add_argument("--tau", type=float, default=0.5)
    parser.add_argument("--lr", type=float, default=3.0e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-eps", type=float, default=0.2)
    parser.add_argument("--entropy-coef", type=float, default=0.05)
    parser.add_argument("--value-coef", type=float, default=0.5)
    parser.add_argument("--residual-reg", type=float, default=0.01)
    parser.add_argument("--update-epochs", type=int, default=1)
    parser.add_argument("--minibatch-size", type=int, default=32)
    parser.add_argument("--max-grad-norm", type=float, default=0.5)
    parser.add_argument("--no-normalize-returns", action="store_true")
    parser.add_argument("--value-clip-eps", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", default="results/diagnostics_latest")
    return parser.parse_args()


if __name__ == "__main__":
    main()
