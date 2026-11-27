"""Summarize matched waypoint visibility runs and draw a no-grid figure."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines
from run_waypoint_visibility_study import MODES, schedule


PAIR_FIELDS = ("coverage_pct", "path_m", "stuck_events", "near_collision_events", "robots_crossing",
               "reached", "published_blocked_waypoints", "published_blocked_pct")


def attempt_evidence(directory, row):
    scope = row["waypoint_visibility_mode"]
    name = f"{row['scene']}_{row['environment_seed']}_{scope}"
    marker = json.loads((directory / f"{name}.complete.json").read_text())
    trace = read_json_lines(directory / marker["directory"] / "decisions.jsonl")
    active = {}
    for event in trace:
        kind = event["event"]
        if kind == "assigned":
            active[event["robot"]] = dict(scene=row["scene"], seed=row["environment_seed"],
                                          mode=scope, robot=event["robot"],
                                          attempt_id=event["attempt_id"], checks=0,
                                          blocked_requests=0, blocked_published=0,
                                          shortened=0, withheld=0, distances=[])
        elif kind == "waypoint_visibility":
            attempt = active[event["robot"]]
            requested, candidate, issued = (event[key] for key in (
                "requested_xy", "candidate_xy", "published_xy"))
            blocked = candidate != requested
            attempt["checks"] += 1
            attempt["blocked_requests"] += blocked
            attempt["blocked_published"] += blocked and issued == requested
            shortened = issued is not None and issued != requested
            attempt["shortened"] += shortened
            attempt["withheld"] += issued is None
            if shortened:
                origin = event["origin_xy"]
                attempt["distances"].append(dict(
                    scene=attempt["scene"], seed=attempt["seed"], robot=attempt["robot"],
                    attempt_id=attempt["attempt_id"], elapsed_s=event["elapsed_sim_s"],
                    requested_m=math.dist(origin, requested), published_m=math.dist(origin, issued)))
        elif kind == "outcome":
            attempt = active.pop(event["robot"])
            if attempt["attempt_id"] != event["attempt_id"]:
                raise ValueError(f"Attempt closure mismatch in {name}")
            distances = attempt.pop("distances")
            yield dict(attempt, outcome=event["outcome"]), distances
    if active:
        raise ValueError(f"Unclosed attempt in {name}")


def groups(rows):
    for group in dict.fromkeys(["all", *(row["layout_group"] for row in rows)]):
        for mode in MODES:
            selected = [row for row in rows if row["waypoint_visibility_mode"] == mode
                        and (group == "all" or row["layout_group"] == group)]
            path = sum(float(row["path_m"]) for row in selected)
            total = {field: sum(int(row[field]) for row in selected) for field in (
                "waypoint_checks", "published_waypoints", "requested_blocked_waypoints",
                "published_blocked_waypoints", "shortened_waypoints", "withheld_waypoints",
                "stuck_events", "near_collision_events", "assignments", "reached",
                "attempts_with_waypoint_checks", "attempts_with_blocked_commands",
                "attempts_adjusted")}
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
    for axis, label in zip(axes, ("Final coverage (%)", "Blocked waypoint commands (%)",
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
    attempts, distances = [], []
    for row in rows:
        row["published_blocked_pct"] = (100.0 * int(row["published_blocked_waypoints"])
                                         / int(row["published_waypoints"]))
        current = []
        for attempt, changed in attempt_evidence(args.runs, row):
            current.append(attempt)
            distances.extend(changed)
        for attempt_field, summary_field in (
            ("checks", "waypoint_checks"), ("blocked_requests", "requested_blocked_waypoints"),
            ("blocked_published", "published_blocked_waypoints"), ("shortened", "shortened_waypoints"),
            ("withheld", "withheld_waypoints"),
        ):
            if sum(item[attempt_field] for item in current) != int(row[summary_field]):
                raise ValueError(f"Waypoint totals differ for seed {row['environment_seed']}")
        row["attempts_with_waypoint_checks"] = sum(item["checks"] > 0 for item in current)
        row["attempts_with_blocked_commands"] = sum(item["blocked_published"] > 0 for item in current)
        row["attempts_adjusted"] = sum(item["shortened"] > 0 or item["withheld"] > 0 for item in current)
        attempts.extend(current)
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "derived_metrics.csv", rows)
    write_csv(args.output / "group_summary.csv", list(groups(rows)))
    write_csv(args.output / "paired_deltas.csv", list(pairs(rows)))
    write_csv(args.output / "attempt_waypoints.csv", attempts)
    if distances:
        write_csv(args.output / "shortened_distances.csv", distances)
    plot(rows, args.output)


if __name__ == "__main__":
    main()
