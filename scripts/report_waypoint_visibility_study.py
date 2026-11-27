"""Summarize matched waypoint visibility runs and draw a no-grid figure."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from run_matched_vision_training import csv_rows, write_csv
from run_waypoint_visibility_study import MODES, schedule


PAIR_FIELDS = ("coverage_pct", "path_m", "stuck_events", "near_collision_events", "robots_crossing",
               "reached", "published_blocked_waypoints", "published_blocked_pct")


def groups(rows):
    for group in dict.fromkeys(["all", *(row["layout_group"] for row in rows)]):
        for mode in MODES:
            selected = [row for row in rows if row["waypoint_visibility_mode"] == mode
                        and (group == "all" or row["layout_group"] == group)]
            path = sum(float(row["path_m"]) for row in selected)
            total = {field: sum(int(row[field]) for row in selected) for field in (
                "waypoint_checks", "published_waypoints", "requested_blocked_waypoints",
                "published_blocked_waypoints", "shortened_waypoints", "withheld_waypoints",
                "stuck_events", "near_collision_events", "assignments", "reached")}
            yield dict(layout_group=group, mode=mode, runs=len(selected),
                       mean_coverage_pct=mean(float(row["coverage_pct"]) for row in selected),
                       mean_path_m=path / len(selected),
                       crossing_runs=sum(int(row["robots_crossing"]) > 0 for row in selected),
                       reached_90=sum(bool(row["time_to_90_s"]) for row in selected),
                       reached_97=sum(bool(row["time_to_97_s"]) for row in selected),
                       stuck_per_10m=10.0 * total["stuck_events"] / path,
                       near_per_10m=10.0 * total["near_collision_events"] / path,
                       published_blocked_pct=100.0 * total["published_blocked_waypoints"]
                       / total["published_waypoints"], **total)


def pairs(rows):
    grouped = defaultdict(dict)
    for row in rows:
        grouped[(row["scene"], row["environment_seed"], row["layout_group"])][
            row["waypoint_visibility_mode"]] = row
    for (scene, seed, group), arms in grouped.items():
        before, after = (arms[mode] for mode in MODES)
        yield dict(scene=scene, seed=seed, layout_group=group, **{
            f"{field}_delta": round(float(after[field]) - float(before[field]), 6)
            for field in PAIR_FIELDS})


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    fields = ("coverage_pct", "published_blocked_pct", "stuck_events")
    for i, mode in enumerate(MODES):
        arm = [row for row in rows if row["waypoint_visibility_mode"] == mode]
        positions = [j + (i - 0.5) * 0.3 for j in range(len(arm))]
        for axis, field in zip(axes, fields):
            axis.bar(positions, [float(row[field]) for row in arm], width=0.28,
                     color=("#345D85", "#50816C")[i],
                     label=("Standard lookahead", "Scan-visible lookahead")[i])
    seeds = [row["environment_seed"] + ("*" if row["layout_group"] == "holdout" else "")
             for row in rows if row["waypoint_visibility_mode"] == MODES[0]]
    for axis, label in zip(axes, ("Final coverage (%)", "Published points crossing occupied cells (%)",
                                  "Stuck reports")):
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds, rotation=45, ha="right")
        axis.set_xlabel("Matched seed" + (" (* holdout)" if any("*" in seed for seed in seeds) else ""))
        axis.set_ylabel(label)
    axes[0].set_ylim(0, 100)
    axes[2].yaxis.set_major_locator(MaxNLocator(integer=True))
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    for suffix in ("png", "pdf"):
        with (output / f"waypoint_visibility_comparison.{suffix}").open("xb") as stream:
            fig.savefig(stream, format=suffix, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", choices=("smoke", "formal"), default="formal")
    args = parser.parse_args()
    manifest = json.loads((args.runs / "experiment_manifest.json").read_text())
    planned, _ = schedule(args.stage)
    rows = csv_rows(args.runs / "summary.csv")
    if (manifest["runs"] != planned or len(rows) != len(planned)
            or any(any(row[key] != str(value) for key, value in condition.items())
                   for row, condition in zip(rows, planned))):
        raise ValueError("Summary does not match the locked schedule")
    for row in rows:
        row["published_blocked_pct"] = (100.0 * int(row["published_blocked_waypoints"])
                                         / int(row["published_waypoints"]))
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "derived_metrics.csv", rows)
    write_csv(args.output / "group_summary.csv", list(groups(rows)))
    write_csv(args.output / "paired_deltas.csv", list(pairs(rows)))
    plot(rows, args.output)


if __name__ == "__main__":
    main()
