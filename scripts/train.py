from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import torch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.exploration.execution_aware_voronoi import ExecutionAwareVoronoi
from eavf_mappo.evaluation.reporting import plot_training_curves
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

    log_rows = []
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
        stats = update(buffers, actor, critic, opt, cfg)
        last = stats[-1] if stats else {"loss": 0.0, "actor_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}
        row = {
            "iteration": iteration,
            "loss": last["loss"],
            "actor_loss": last["actor_loss"],
            "value_loss": last["value_loss"],
            "entropy": last["entropy"],
            "coverage": env.grid.coverage_ratio(),
            "return_mean": last.get("return_mean", 0.0),
            "return_std": last.get("return_std", 0.0),
            "value_pred_mean": last.get("value_pred_mean", 0.0),
            "approx_kl": last.get("approx_kl", 0.0),
        }
        log_rows.append(row)
        print(
            f"iter={iteration} loss={row['loss']:.4f} "
            f"actor={row['actor_loss']:.4f} value={row['value_loss']:.4f} "
            f"coverage={row['coverage']:.4f} ret_std={row['return_std']:.4f} "
            f"kl={row['approx_kl']:.6f}"
        )

    checkpoint = out_dir / "checkpoint.pt"
    torch.save(
        {
            "actor": actor.state_dict(),
            "critic": critic.state_dict(),
            "optimizer": opt.state_dict(),
            "args": vars(args),
        },
        checkpoint,
    )
    train_log_path = out_dir / "train_log.csv"
    curves_path = out_dir / "training_curves.png"
    write_log(train_log_path, log_rows)
    plot_training_curves(train_log_path, curves_path)
    print(f"wrote {checkpoint}")
    print(f"wrote {train_log_path}")
    print(f"wrote {curves_path}")


def write_log(path: Path, rows: list[dict]) -> None:
    fieldnames = [
        "iteration",
        "loss",
        "actor_loss",
        "value_loss",
        "entropy",
        "coverage",
        "return_mean",
        "return_std",
        "value_pred_mean",
        "approx_kl",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_args():
    parser = argparse.ArgumentParser(description="Train the lightweight EA-VF-MAPPO actor/critic.")
    parser.add_argument("--iterations", type=int, default=2)
    parser.add_argument("--rollout-length", type=int, default=8)
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--map-size", type=float, default=8.0)
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--sensor-range", type=float, default=1.5)
    parser.add_argument("--sensor-rays", type=int, default=36)
    parser.add_argument("--map-type", default="office", choices=["open_room", "corridor", "maze", "office", "cluttered"])
    parser.add_argument("--max-episode-steps", type=int, default=100)
    parser.add_argument("--frontier-k", type=int, default=8)
    parser.add_argument("--max-high-level-interval", type=int, default=3)
    parser.add_argument("--d-model", type=int, default=64)
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
    parser.add_argument("--output-dir", default="results/train_latest")
    return parser.parse_args()


if __name__ == "__main__":
    main()
