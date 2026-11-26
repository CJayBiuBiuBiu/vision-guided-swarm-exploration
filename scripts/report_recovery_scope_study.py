"""Report matched frontier availability and recovery outcomes."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from audit_recovery_scope import clearance_events
from report_route_recovery_study import derived_rows
from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines
from run_recovery_scope_study import SCOPES, schedule
from run_wait_margin_study import ROBOTS, node_log


METRICS = (
    "coverage_pct", "path_m", "new_area_per_m", "stuck_events", "stuck_per_10m",
    "repeated_region_20s", "near_collision_events", "robots_crossing", "goal_free_robot_s",
    "blocked_frontier_point_seconds", "sparse_frontier_seconds", "route_peer_retirements",
)


def grouped(rows):
    for scope in SCOPES:
        runs = [row for row in rows if row["recovery_frontier_scope"] == scope]
        recoveries = sum(int(row["recovery_results"]) for row in runs)
        successes = sum(int(row["recovery_successes"]) for row in runs)
        yield dict(scope=scope, runs=len(runs), crossing_runs=sum(int(row["robots_crossing"]) > 0
                                                                for row in runs),
                   recovery_results=recoveries, recovery_successes=successes,
                   recovery_success_rate=successes / recoveries if recoveries else None,
                   **{f"{key}_mean": mean(float(row[key]) for row in runs) for key in METRICS},
                   **{f"{key}_total": sum(int(row[key]) for row in runs) for key in (
                       "stuck_events", "near_collision_events", "route_goals_retired",
                       "route_far_goals_retired", "route_peer_retirements", "repeated_region_20s",
                       "logged_near_events", "logged_near_navigation", "logged_near_inspection",
                       "near_within_5s_of_report")})


def matched(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[(row["scene"], row["environment_seed"])][row["recovery_frontier_scope"]] = row
    for (scene, seed), group in groups.items():
        if set(group) != set(SCOPES):
            raise ValueError(f"Incomplete matched pair {scene} {seed}")
        before, after = (group[scope] for scope in SCOPES)
        yield dict(scene=scene, seed=seed,
                   **{f"{key}_delta": round(float(after[key]) - float(before[key]), 6)
                      for key in METRICS})


def per_run(formal, original):
    # The earlier derived-metric reader uses the condition label to locate folders.
    labeled = [dict(row, route_recovery_mode=row["recovery_frontier_scope"]) for row in original]
    for row, stuck_events in derived_rows(formal, labeled):
        scope = row["recovery_frontier_scope"]
        name = f"{row['scene']}_{row['environment_seed']}_{scope}"
        marker = json.loads((formal / f"{name}.complete.json").read_text())
        directory = formal / marker["directory"]
        events = read_json_lines(directory / "decisions.jsonl")
        reports = defaultdict(list)
        for event in events:
            if event["event"] == "route_recovery":
                reports[event["robot"]].append(event["report_stamp"])
        logs = {robot: node_log(directory, robot + "_explorer") for robot in ROBOTS}
        near = sorted(clearance_events(logs), key=lambda event: event["stamp_s"])
        for event in near:
            ages = [event["stamp_s"] - stamp for stamp in reports[event["robot"]]
                    if stamp <= event["stamp_s"]]
            age = min(ages) if ages else None
            event.update(scene=row["scene"], seed=row["environment_seed"], scope=scope,
                         seconds_since_stuck=age, within_5s_of_report=age is not None and age <= 5.0)
        row["route_recovery_mode"] = "guarded"
        row["near_within_5s_of_report"] = sum(event["within_5s_of_report"] for event in near)
        yield row, stuck_events, near


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False, "axes.spines.right": False,
                        "pdf.fonttype": 42})
    fig, panels = plt.subplots(1, 3, figsize=(10.5, 3.8))
    fields = ("coverage_pct", "sparse_frontier_seconds", "stuck_events")
    for index, scope in enumerate(SCOPES):
        selected = [row for row in rows if row["recovery_frontier_scope"] == scope]
        x = [j + (index - 0.5) * 0.30 for j in range(len(selected))]
        for panel, key in zip(panels, fields):
            panel.bar(x, [float(row[key]) for row in selected], width=0.28,
                      color=("#345D85", "#50816C")[index],
                      label=("Shared exclusion", "Robot-scoped exclusion")[index])
    seeds = [row["environment_seed"] for row in rows if row["recovery_frontier_scope"] == SCOPES[0]]
    for panel, ylabel in zip(panels, ("Final coverage (%)", "Time with at most two frontiers (s)",
                                    "Stuck reports")):
        panel.set_xticks(range(len(seeds)))
        panel.set_xticklabels(seeds, rotation=35, ha="right")
        panel.set_xlabel("Matched seed")
        panel.set_ylabel(ylabel)
    panels[0].set_ylim(0, 100)
    handles, labels = panels[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    for ext in ("png", "pdf"):
        with (output / f"recovery_scope_comparison.{ext}").open("xb") as stream:
            fig.savefig(stream, format=ext, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", choices=("smoke", "formal"), default="formal")
    args = parser.parse_args()
    rows = csv_rows(args.runs / "summary.csv")
    manifest = json.loads((args.runs / "experiment_manifest.json").read_text())
    scheduled, _ = schedule(args.stage)
    if (manifest["runs"] != scheduled or len(rows) != len(scheduled)
            or any(any(row[key] != str(value) for key, value in run.items())
                   for row, run in zip(rows, scheduled))):
        raise ValueError("Summary does not match the locked schedule")
    args.output.mkdir(parents=True, exist_ok=False)
    derived, all_stuck, all_near = [], [], []
    for row, stuck, near in per_run(args.runs, rows):
        derived.append(row)
        all_stuck.extend(stuck)
        all_near.extend(near)
    write_csv(args.output / "derived_metrics.csv", derived)
    write_csv(args.output / "group_summary.csv", list(grouped(derived)))
    deltas = list(matched(derived))
    write_csv(args.output / "paired_deltas.csv", deltas)
    write_csv(args.output / "paired_summary.csv", [dict(
        pairs=len(deltas), coverage_improved=sum(row["coverage_pct_delta"] > 0 for row in deltas),
        stuck_reduced=sum(row["stuck_events_delta"] < 0 for row in deltas),
        **{f"{key}_mean_delta": mean(row[f"{key}_delta"] for row in deltas) for key in METRICS})])
    if all_stuck:
        write_csv(args.output / "stuck_events.csv", all_stuck)
    if all_near:
        write_csv(args.output / "clearance_events.csv", all_near)
    plot(derived, args.output)


if __name__ == "__main__":
    main()
