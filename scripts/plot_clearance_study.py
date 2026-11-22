"""Plot seed-matched Gazebo coverage differences from a clearance study."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator


METRICS = (
    ("coverage_pct_difference", "Final coverage"),
    ("mean_coverage_pct_difference", "Time-averaged coverage"),
)
LAYOUTS = {"hu_aligned": ("Hu-aligned", "#235C86"), "hu_layout_c": ("Layout C", "#AD5433")}
CONDITIONS = {"lidar": "Direct LiDAR", "latest_fusion": "Latest-only fusion",
              "buffered_fusion": "Buffered fusion"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paired_differences", type=Path)
    parser.add_argument("output", type=Path, help="A new PNG, PDF or SVG file")
    args = parser.parse_args()
    with args.paired_differences.open(newline="") as stream:
        rows = sorted(csv.DictReader(stream), key=lambda row: (row["layout"], int(row["seed"])))
    if not rows or len({(row["layout"], row["seed"]) for row in rows}) != len(rows):
        raise ValueError("Expected one row per complete layout and seed pair")
    protocols = {(row["reference"], row["treatment"]) for row in rows}
    if len(protocols) != 1:
        raise ValueError("Expected a single condition pair")
    (reference, treatment), = protocols

    differences = [[float(row[key]) for row in rows] for key, _ in METRICS]
    if any(not math.isfinite(value) for values in differences for value in values):
        raise ValueError("Coverage differences must be finite")
    limit = max(5, math.ceil(1.1 * max(abs(value) for values in differences for value in values) / 5) * 5)
    offsets = list(reversed(range(len(rows))))
    colours = [LAYOUTS[row["layout"]][1] for row in rows]
    labels = [f"{LAYOUTS[row['layout']][0]} · {row['seed']}" for row in rows]

    figure, axes = plt.subplots(1, 2, sharex=True, sharey=True,
                                figsize=(8.4, max(3.0, 0.31 * len(rows) + 1.1)))
    for axis, (values, (_, title)) in zip(axes, zip(differences, METRICS)):
        axis.set_title(title, loc="left", pad=12, fontsize=11, fontweight="semibold")
        axis.axvline(0, linewidth=0.8, color="#707980")
        axis.hlines(offsets, [min(0, value) for value in values],
                    [max(0, value) for value in values], colors=colours, alpha=0.55, linewidth=1.2)
        axis.scatter(values, offsets, color=colours, s=29, zorder=3)
        axis.set_xlim(-limit, limit)
        axis.set_ylim(-0.65, len(rows) - 0.35)
        axis.xaxis.set_major_locator(MaxNLocator(nbins=5, symmetric=True, integer=True))
        axis.set_yticks(offsets)
        axis.tick_params(axis="y", length=0, pad=8)
        for side in ("top", "left", "right"):
            axis.spines[side].set_visible(False)
    axes[0].set_yticklabels(labels, fontsize=9)
    figure.text(0.545, 0.032,
                f"{CONDITIONS[treatment]} − {CONDITIONS[reference]} (percentage points)",
                ha="center", fontsize=9)
    figure.tight_layout(rect=(0, 0.095, 1, 1), w_pad=2.5)
    with args.output.open("xb") as stream:
        figure.savefig(stream, format=args.output.suffix[1:].lower(), dpi=240)
    plt.close(figure)


if __name__ == "__main__":
    main()
