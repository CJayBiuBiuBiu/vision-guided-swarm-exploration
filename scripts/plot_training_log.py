from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.evaluation.reporting import plot_training_curves


def main() -> None:
    args = parse_args()
    output = Path(args.output)
    plot_training_curves(args.train_log_csv, output)
    print(f"wrote {output}")


def parse_args():
    parser = argparse.ArgumentParser(description="Plot training curves from scripts/train.py train_log.csv.")
    parser.add_argument("train_log_csv")
    parser.add_argument("--output", default="training_curves.png")
    return parser.parse_args()


if __name__ == "__main__":
    main()
