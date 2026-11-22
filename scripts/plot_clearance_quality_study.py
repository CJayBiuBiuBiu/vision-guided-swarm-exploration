"""Plot fixed and adaptive clearance across the paired camera conditions."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


LAYOUTS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}
LINES = (
    ("lidar", "LiDAR", "#235C86", "o"),
    ("fixed_fusion", "Fixed fusion", "#A64B2A", "s"),
    ("adaptive_fusion", "Adaptive fusion", "#19826F", "D"),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paired_differences", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    blocks = defaultdict(dict)
    with args.paired_differences.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["metric"] == "coverage_pct":
                block = blocks[row["layout"], row["seed"]]
                if row["camera_profile"] in block:
                    raise ValueError("Duplicate camera condition")
                block[row["camera_profile"]] = row
    if not blocks or any(set(block) != {"clean", "dark"} for block in blocks.values()):
        raise ValueError("Each seed needs clean and darkened coverage")
    values = [float(row[key]) for block in blocks.values() for row in block.values()
              for key, *_ in LINES]
    if not all(0 <= value <= 100 for value in values):
        raise ValueError("Expected finite coverage percentages")
    bottom = max(0, 10 * math.floor((min(values) - 4) / 10))
    figure, axes = plt.subplots(1, len(blocks), sharey=True, squeeze=False,
                                figsize=(max(4.4, 3.9 * len(blocks)), 3.45))
    for axis, ((layout, seed), block) in zip(axes[0], sorted(blocks.items())):
        axis.set_title(f"{LAYOUTS[layout]} · {seed}", loc="left", fontsize=10)
        for key, name, colour, marker in LINES:
            axis.plot([0, 1], [float(block[state][key]) for state in ("clean", "dark")],
                      label=name, color=colour, marker=marker, linewidth=1.6, markersize=5)
        axis.set_xticks([0, 1])
        axis.set_xticklabels(["Clean RGB", "Darkened RGB"])
        axis.set_xlim(-0.23, 1.23)
        axis.set_ylim(bottom, 104)
        axis.yaxis.set_major_locator(MultipleLocator(10))
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    axes[0, 0].set_ylabel("Final coverage (%)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", frameon=False, ncol=3, fontsize=9)
    figure.tight_layout(rect=(0, 0, 1, .88), w_pad=2)
    with args.output.open("xb") as stream:
        figure.savefig(stream, format=args.output.suffix[1:].lower(), dpi=240)
    plt.close(figure)


if __name__ == "__main__":
    main()
