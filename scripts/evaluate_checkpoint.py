from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.evaluation.checkpoint_runner import run_checkpoint_evaluation
from eavf_mappo.evaluation.reporting import write_csv


def main() -> None:
    args = parse_args()
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    ckpt_args = checkpoint.get("args", {})
    out_dir = Path(args.output_dir)

    env_config = {
        "env": {
            "n_robots": _arg_or_checkpoint(args.n_robots, ckpt_args, "n_robots", 2, int),
            "map_size_m": _arg_or_checkpoint(args.map_size, ckpt_args, "map_size", 8.0, float),
            "resolution": _arg_or_checkpoint(args.resolution, ckpt_args, "resolution", 0.5, float),
            "sensor_range_m": _arg_or_checkpoint(args.sensor_range, ckpt_args, "sensor_range", 1.5, float),
            "sensor_n_rays": _arg_or_checkpoint(args.sensor_rays, ckpt_args, "sensor_rays", 36, int),
            "max_episode_steps": args.max_steps,
        },
        "high_level": {
            "frontier_K": _arg_or_checkpoint(args.frontier_k, ckpt_args, "frontier_k", 8, int),
            "max_high_level_interval": _arg_or_checkpoint(
                args.max_high_level_interval,
                ckpt_args,
                "max_high_level_interval",
                3,
                int,
            ),
        },
    }
    frontier_k = env_config["high_level"]["frontier_K"]
    step_rows, summary_rows = run_checkpoint_evaluation(
        checkpoint_path=args.checkpoint,
        seeds=args.seeds,
        map_types=args.map_types,
        max_steps=args.max_steps,
        env_config=env_config,
        frontier_k=frontier_k,
        stochastic=args.stochastic,
        device=torch.device(args.device),
    )

    steps_path = out_dir / "checkpoint_eval_steps.csv"
    summary_path = out_dir / "checkpoint_eval_summary.csv"
    write_csv(steps_path, step_rows)
    write_csv(summary_path, summary_rows)
    print(f"wrote {steps_path}")
    print(f"wrote {summary_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate a trained EA-VF-MAPPO checkpoint.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    parser.add_argument(
        "--map-types",
        nargs="+",
        default=["office"],
        choices=["open_room", "corridor", "maze", "office", "cluttered"],
    )
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--n-robots", type=int, default=None)
    parser.add_argument("--map-size", type=float, default=None)
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--sensor-range", type=float, default=None)
    parser.add_argument("--sensor-rays", type=int, default=None)
    parser.add_argument("--frontier-k", type=int, default=None)
    parser.add_argument("--max-high-level-interval", type=int, default=None)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--output-dir", default="results/checkpoint_eval_latest")
    return parser.parse_args()


def _arg_or_checkpoint(value, ckpt_args: dict, key: str, default, cast):
    if value is not None:
        return cast(value)
    return cast(ckpt_args.get(key, default))


if __name__ == "__main__":
    main()
