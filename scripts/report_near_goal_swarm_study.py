"""Summarize matched exploration and the behavior of the near-goal tracker."""

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
from run_near_goal_swarm_study import MODES, schedule
from run_policy_goal_retention import read_json_lines


PAIR_FIELDS = ("coverage_pct", "path_m", "robots_crossing", "stuck_events", "near_collision_events",
               "near_goal_samples", "stuck_per_10m", "wait_endings_within_3s")


def per_run(directory, rows):
    for row in rows:
        name = f"{row['scene']}_{row['environment_seed']}_{row['local_tracking_mode']}"
        marker = json.loads((directory / f"{name}.complete.json").read_text())
        events = read_json_lines(directory / marker["directory"] / "decisions.jsonl")
        assigned = {event["attempt_id"]: event["elapsed_sim_s"] for event in events
                    if event["event"] == "assigned"}
        waits = [event["elapsed_sim_s"] - assigned[event["attempt_id"]] for event in events
                 if event["event"] == "outcome" and event["outcome"] == "wait"]
        yield dict(**row, wait_endings=len(waits), wait_endings_within_3s=sum(age <= 3.0 for age in waits),
                   stuck_per_10m=10.0 * int(row["stuck_events"]) / float(row["path_m"]))


def summarize(rows):
    for group in dict.fromkeys(["all", *(row["layout_group"] for row in rows)]):
        for mode in MODES:
            selected = [row for row in rows if row["local_tracking_mode"] == mode
                        and (group == "all" or row["layout_group"] == group)]
            distance = sum(float(row["path_m"]) for row in selected)
            total = {key: sum(int(row[key]) for row in selected) for key in (
                "assignments", "reached", "stuck_events", "near_collision_events",
                "tracking_samples", "near_goal_samples", "eligible_samples", "short_target_samples",
                "short_large_bearing_samples", "short_large_bearing_opposed_turns",
                "wait_endings", "wait_endings_within_3s", "withheld_waypoints",
                "shortened_waypoints", "published_blocked_waypoints")}
            yield dict(layout_group=group, mode=mode, runs=len(selected),
                       coverage_mean_pct=mean(float(row["coverage_pct"]) for row in selected),
                       path_mean_m=distance / len(selected),
                       crossed_runs=sum(int(row["robots_crossing"]) > 0 for row in selected),
                       runs_90=sum(bool(row["time_to_90_s"]) for row in selected),
                       runs_97=sum(bool(row["time_to_97_s"]) for row in selected),
                       stuck_per_10m=10.0 * total["stuck_events"] / distance,
                       near_per_10m=10.0 * total["near_collision_events"] / distance,
                       near_goal_sample_pct=100.0 * total["near_goal_samples"] / total["tracking_samples"],
                       **total)


def paired(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[(row["scene"], row["environment_seed"], row["layout_group"])][
            row["local_tracking_mode"]] = row
    for (scene, seed, group), arms in groups.items():
        old, new = (arms[mode] for mode in MODES)
        yield dict(scene=scene, seed=seed, layout_group=group, **{
            f"{name}_delta": round(float(new[name]) - float(old[name]), 6) for name in PAIR_FIELDS})


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                        "axes.spines.right": False, "pdf.fonttype": 42})
    figure, axes = plt.subplots(1, 3, figsize=(11.8, 3.8))
    fields = ("coverage_pct", "stuck_events", "near_collision_events")
    for i, mode in enumerate(MODES):
        arm = [row for row in rows if row["local_tracking_mode"] == mode]
        positions = [j + (i - 0.5) * 0.30 for j in range(len(arm))]
        for axis, field in zip(axes, fields):
            axis.bar(positions, [float(row[field]) for row in arm], width=0.28,
                     color=("#345D85", "#50816C")[i],
                     label=("Frozen DDPG", "DDPG with near-goal tracking")[i])
    seeds = [row["environment_seed"] + ("*" if row["layout_group"] == "holdout" else "")
             for row in rows if row["local_tracking_mode"] == MODES[0]]
    for axis, label in zip(axes, ("Final coverage (%)", "Stuck reports", "Near-collision threshold events")):
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds, rotation=45, ha="right")
        axis.set_xlabel("Matched seed" + (" (* holdout)" if any("*" in seed for seed in seeds) else ""))
        axis.set_ylabel(label)
    axes[0].set_ylim(0, 100)
    for axis in axes[1:]:
        axis.yaxis.set_major_locator(MaxNLocator(integer=True))
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    figure.tight_layout(rect=(0, 0, 1, 0.89))
    for suffix in ("png", "pdf"):
        with (output / f"near_goal_swarm_comparison.{suffix}").open("xb") as stream:
            figure.savefig(stream, format=suffix, dpi=220)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", choices=("smoke", "formal"), default="formal")
    args = parser.parse_args()
    rows = csv_rows(args.runs / "summary.csv")
    planned, _ = schedule(args.stage)
    manifest = json.loads((args.runs / "experiment_manifest.json").read_text())
    if (manifest["runs"] != planned or len(rows) != len(planned)
            or any(any(row[key] != str(value) for key, value in condition.items())
                   for row, condition in zip(rows, planned))):
        raise ValueError("Summary does not match the locked schedule")
    derived = list(per_run(args.runs, rows))
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "derived_metrics.csv", derived)
    write_csv(args.output / "group_summary.csv", list(summarize(derived)))
    write_csv(args.output / "paired_deltas.csv", list(paired(derived)))
    plot(derived, args.output)


if __name__ == "__main__":
    main()
