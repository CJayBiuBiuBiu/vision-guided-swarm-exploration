"""Plot the two decision cues for each matched seed."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


LAYOUTS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}
LINES = (
    ("LiDAR", "#235C86", "o", ("lidar_lidar", "lidar_fusion")),
    ("RGB–LiDAR", "#A64B2A", "s", ("fusion_lidar", "fusion_fusion")),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seed_contrasts", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.seed_contrasts.open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if row["metric"] == "coverage_pct"]
    if not rows or len({(row["layout"], row["seed"]) for row in rows}) != len(rows):
        raise ValueError("Expected one coverage contrast per layout and seed")
    rows.sort(key=lambda row: (row["layout"], int(row["seed"])))
    values = [float(row[field]) for row in rows for _, _, _, fields in LINES for field in fields]
    if not all(0 <= value <= 100 for value in values):
        raise ValueError("Expected finite coverage percentages")

    figure, axes = plt.subplots((len(rows) + 1) // 2, 2, sharey=True, squeeze=False,
                                figsize=(7.8, 1.9 * ((len(rows) + 1) // 2) + 0.85))
    for axis, row in zip(axes.flat, rows):
        axis.set_title(f"{LAYOUTS[row['layout']]} · {row['seed']}", loc="left", fontsize=10)
        for name, colour, marker, fields in LINES:
            axis.plot([0, 1], [float(row[field]) for field in fields], label=name,
                      color=colour, marker=marker, linewidth=1.55, markersize=5)
        axis.set_xticks([0, 1])
        axis.set_xticklabels(["LiDAR", "RGB–LiDAR"])
        axis.set_xlim(-0.22, 1.22)
        axis.set_ylim(max(0, 10 * math.floor((min(values) - 4) / 10)), 104)
        axis.yaxis.set_major_locator(MultipleLocator(10))
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    for axis in list(axes.flat)[len(rows):]:
        axis.set_visible(False)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, title="MAPPO cue", loc="upper center",
                  frameon=False, ncol=2, fontsize=9)
    figure.text(.5, .025, "Frontier rules cue", ha="center", fontsize=10)
    figure.text(.026, .50, "Final coverage (%)", va="center", rotation="vertical", fontsize=10)
    figure.tight_layout(rect=(.05, .07, 1, .88), h_pad=1.2)
    with args.output.open("xb") as stream:
        figure.savefig(stream, format=args.output.suffix[1:].lower(), dpi=240)
    plt.close(figure)


if __name__ == "__main__":
    main()
