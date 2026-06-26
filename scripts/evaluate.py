from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.evaluation.reporting import plot_step_metrics, summarize_steps, write_csv
from eavf_mappo.evaluation.runner import run_evaluation


def main() -> None:
    args = parse_args()
    out_dir = Path(args.output_dir)
    env_config = {
        "env": {
            "n_robots": args.n_robots,
            "map_size_m": args.map_size,
            "resolution": args.resolution,
            "max_episode_steps": args.max_steps,
            "sensor_range_m": args.sensor_range,
            "sensor_n_rays": args.sensor_rays,
            "map_type": args.map_type,
        }
    }
    rows = run_evaluation(
        policies=args.policies,
        seeds=args.seeds,
        max_steps=args.max_steps,
        env_config=env_config,
    )
    summary = summarize_steps(rows)

    step_csv = out_dir / "eval_steps.csv"
    summary_csv = out_dir / "eval_summary.csv"
    coverage_png = out_dir / "coverage_curve.png"
    reward_png = out_dir / "reward_curve.png"
    write_csv(step_csv, rows)
    write_csv(summary_csv, summary)
    plot_step_metrics(step_csv, coverage_png, reward_png)

    print(f"wrote {step_csv}")
    print(f"wrote {summary_csv}")
    print(f"wrote {coverage_png}")
    print(f"wrote {reward_png}")


def parse_args():
    parser = argparse.ArgumentParser(description="Run lightweight EA-VF-MAPPO evaluation rollouts.")
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    parser.add_argument(
        "--policies",
        nargs="+",
        default=["nearest_frontier", "round_robin", "constant"],
        choices=["nearest_frontier", "round_robin", "constant"],
    )
    parser.add_argument("--max-steps", type=int, default=50)
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--map-size", type=float, default=8.0)
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--sensor-range", type=float, default=1.5)
    parser.add_argument("--sensor-rays", type=int, default=36)
    parser.add_argument(
        "--map-type",
        default="open_room",
        choices=["open_room", "corridor", "maze", "office", "cluttered"],
    )
    parser.add_argument("--output-dir", default="results/latest")
    return parser.parse_args()


if __name__ == "__main__":
    main()
