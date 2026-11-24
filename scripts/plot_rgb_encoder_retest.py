"""Plot paired coverage and clearance distortions in the RGB checkpoint retest."""

import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows
from run_rgb_encoder_retest import METHODS


LABELS = ("LiDAR", "Legacy RGB", "Raw robust", "Aligned robust")
COLOURS = ("#2A5C84", "#937040", "#4B7560", "#805990")
LAYOUT_LABELS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    groups = defaultdict(dict)
    for row in csv_rows(args.runs):
        groups[row["layout"], row["environment_seed"]][row["method"]] = row
    if (len(groups) != 4 or any(set(group) != set(METHODS) for group in groups.values())
            or len({layout for layout, _ in groups}) != len(LAYOUT_LABELS)):
        raise ValueError("The complete four-block study is required")

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": False, "pdf.fonttype": 42})
    figure, axes = plt.subplots(1, 2, figsize=(9, 3.7), sharey=True)
    for axis, label in zip(axes, LAYOUT_LABELS.values()):
        axis.set_title(label, loc="left", fontweight="bold")
        axis.set_ylim(0, 102)
        axis.set_xticks(range(4))
        axis.set_xticklabels(LABELS, rotation=20, ha="right")
    for index, ((layout, seed), group) in enumerate(sorted(groups.items())):
        axis = axes[list(LAYOUT_LABELS).index(layout)]
        values = [float(group[method]["coverage_pct"]) for method in METHODS]
        axis.plot(range(4), values, marker="o", color=COLOURS[index], label=f"Seed {seed}")
    for axis in axes:
        axis.legend(loc="lower right", frameon=False)
    axes[0].set_ylabel("Final coverage at 90 s (%)")
    figure.tight_layout()
    args.destination.mkdir(parents=True, exist_ok=True)
    for suffix in ("pdf", "png"):
        with (args.destination / f"paired_coverage.{suffix}").open("xb") as stream:
            figure.savefig(stream, format=suffix, dpi=220)
    plt.close(figure)


if __name__ == "__main__":
    main()
