"""Report reached fixed targets and draw example tracked trajectories."""

import argparse
from collections import defaultdict
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows, write_csv


MODES = ("ddpg", "near_goal")


def grouped(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[("all", "all", row["mode"])].append(row)
        distance = "near" if float(row["distance_m"]) <= 1.0 else "long"
        groups[("all", distance, row["mode"])].append(row)
        groups[(row["layout"], distance, row["mode"])].append(row)
    for (layout, distance, mode), runs in groups.items():
        reached = [row for row in runs if row["reached"] == "1"]
        yield dict(layout=layout, distance_group=distance, mode=mode, trials=len(runs),
                   reached=len(reached), threshold_events=sum(int(row["laser_threshold_events"])
                                                             for row in runs),
                   median_success_s=median(float(row["elapsed_s"]) for row in reached) if reached else None,
                   median_final_distance_m=median(float(row["final_distance_m"]) for row in runs))


def paired(rows):
    cases = defaultdict(dict)
    for row in rows:
        cases[(row["layout"], row["name"])][row["mode"]] = row
    for (layout, name), arms in cases.items():
        old, new = (arms[mode] for mode in MODES)
        yield dict(layout=layout, target=name, distance_m=old["distance_m"], bearing_deg=old["bearing_deg"],
                   ddpg_reached=old["reached"], near_goal_reached=new["reached"],
                   ddpg_final_distance_m=old["final_distance_m"], near_goal_final_distance_m=new["final_distance_m"],
                   ddpg_time_s=old["elapsed_s"], near_goal_time_s=new["elapsed_s"])


def plot(directory, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                        "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 4.3))
    for axis, layout in zip(axes, ("open_floor", "offset_posts")):
        axis.set_title(layout.replace("_", " ").capitalize())
        for mode, color in zip(MODES, ("#345D85", "#50816C")):
            rows = csv_rows(directory / layout / f"left_060_{mode}.csv")
            axis.plot([float(row["x_m"]) for row in rows], [float(row["y_m"]) for row in rows],
                      color=color, label=("Frozen DDPG" if mode == "ddpg" else "Near-goal controller"))
        axis.scatter([0], [0], color="black", marker="o", s=20, label="Start")
        axis.scatter([0], [0.60], color="#B44C35", marker="x", s=65, label="Target")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_aspect("equal", adjustable="datalim")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.85))
    for suffix in ("png", "pdf"):
        with (output / f"fixed_waypoint_trajectories.{suffix}").open("xb") as stream:
            fig.savefig(stream, format=suffix, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.benchmark / "summary.csv")
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "group_summary.csv", list(grouped(rows)))
    write_csv(args.output / "paired_results.csv", list(paired(rows)))
    plot(args.benchmark, args.output)


if __name__ == "__main__":
    main()
