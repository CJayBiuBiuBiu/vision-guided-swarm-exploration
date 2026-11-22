"""Plot adaptive RGB use and coverage contrasts for a mixed-exposure study."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator


LAYOUTS = {"hu_aligned": ("Hu-aligned", "#235C86"), "hu_layout_c": ("Layout C", "#A64B2A")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--horizon", type=int, default=300)
    args = parser.parse_args()
    edges = [*range(0, args.horizon, 60), args.horizon]
    if not 0 < args.horizon <= 300:
        parser.error("Expected a mixed-exposure horizon between 1 and 300 seconds")

    adaptive = defaultdict(dict)
    with (args.summary / "exposure_windows.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["cue"] == "adaptive":
                key = row["layout"], int(row["seed"])
                index = int(row["window"])
                if index in adaptive[key]:
                    raise ValueError("Duplicate exposure window")
                adaptive[key][index] = float(row["rgb_robot_tick_fraction"]) * 100
    with (args.summary / "paired_camera_differences.csv").open(newline="") as stream:
        differences = {(row["layout"], int(row["seed"])): row for row in csv.DictReader(stream)
                       if row["metric"] == "coverage_pct" and row["camera_profile"] == "mixed"}
    if not adaptive or set(adaptive) != set(differences):
        raise ValueError("Coverage and exposure windows must include the same runs")

    figure, (usage, coverage) = plt.subplots(1, 2, figsize=(9.4, 3.6))
    seed_counts = defaultdict(int)
    blocks = sorted(adaptive)
    for index, start in enumerate(edges[:-1]):
        if index % 2:
            usage.axvspan(start, edges[index + 1], color="#dde3e9", alpha=.8, linewidth=0)
    for world, seed in blocks:
        label, colour = LAYOUTS[world]
        windows = adaptive[world, seed]
        if set(windows) != set(range(len(edges) - 1)):
            raise ValueError(f"Missing exposure windows for {world}/{seed}")
        rates = [windows[index] for index in range(len(edges) - 1)]
        if not all(0 <= rate <= 100 for rate in rates):
            raise ValueError(f"Invalid RGB usage for {world}/{seed}")
        style = ("-", "--", ":", "-.")[seed_counts[world] % 4]
        seed_counts[world] += 1
        usage.step(edges, [*rates, rates[-1]], where="post", color=colour, linestyle=style,
                   linewidth=1.6, label=f"{label} · {seed}")
    usage.set_title("Adaptive RGB use", loc="left", fontsize=11)
    usage.set(xlabel="Gazebo time (s)", ylabel="Robot control ticks (%)",
              xlim=(0, args.horizon), ylim=(-4, 105))
    usage.set_xticks(edges)
    usage.legend(frameon=False, fontsize=7.5, loc="upper left", bbox_to_anchor=(0, -.31), ncol=2)

    offsets = list(reversed(range(len(blocks))))
    entries = (("adaptive_minus_lidar", "vs LiDAR", "#235C86", "o", .13),
               ("adaptive_minus_fixed", "vs fixed fusion", "#A64B2A", "D", -.13))
    all_values = []
    for key, label, colour, marker, shift in entries:
        values = [float(differences[block][key]) for block in blocks]
        coverage.scatter(values, [index + shift for index in offsets],
                         color=colour, marker=marker, s=28, label=label, zorder=3)
        all_values.extend(values)
    limit = max(5, math.ceil(1.12 * max(map(abs, all_values)) / 5) * 5)
    coverage.axvline(0, color="#707980", linewidth=.8)
    coverage.set_title("Paired final coverage", loc="left", fontsize=11)
    coverage.set(xlabel="Adaptive difference (percentage points)", xlim=(-limit, limit),
                 ylim=(-.55, len(blocks) - .45))
    coverage.set_yticks(offsets)
    coverage.set_yticklabels([f"{LAYOUTS[world][0]} · {seed}" for world, seed in blocks], fontsize=8)
    coverage.xaxis.set_major_locator(MaxNLocator(nbins=5, symmetric=True, integer=True))
    coverage.legend(frameon=False, fontsize=7.5, loc="upper right", bbox_to_anchor=(1, -.31), ncol=2)
    for axis in (usage, coverage):
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    figure.subplots_adjust(left=.085, right=.985, bottom=.28, top=.87, wspace=.50)
    with args.output.open("xb") as stream:
        figure.savefig(stream, format=args.output.suffix[1:].lower(), dpi=240)
    plt.close(figure)


if __name__ == "__main__":
    main()
