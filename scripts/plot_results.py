from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.evaluation.reporting import plot_step_metrics


def main() -> None:
    args = parse_args()
    out_dir = Path(args.output_dir)
    plot_step_metrics(
        args.step_csv,
        out_dir / "coverage_curve.png",
        out_dir / "reward_curve.png",
    )
    print(f"wrote {out_dir / 'coverage_curve.png'}")
    print(f"wrote {out_dir / 'reward_curve.png'}")


def parse_args():
    parser = argparse.ArgumentParser(description="Regenerate evaluation plots from eval_steps.csv.")
    parser.add_argument("step_csv")
    parser.add_argument("--output-dir", default="results/latest")
    return parser.parse_args()


if __name__ == "__main__":
    main()
