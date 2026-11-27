"""Summarize matched recovery runs and revisit the displacement at repeated reports."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from audit_recovery_scope import clearance_events
from audit_recovery_displacement import RECOVERY_TIMESTAMP_SKEW_S, recovery_outcomes
from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines
from run_recovery_displacement_study import MODES, schedule
from run_wait_margin_study import ROBOTS, node_log


REPEAT_WINDOW_S = 20.0
REPEAT_RADIUS_M = 0.90
FAILED_ROUTE_RADIUS_M = 0.65
PAIR_METRICS = (
    "coverage_pct", "path_m", "robots_crossing", "near_collision_events", "stuck_events",
    "repeat_reports", "repeats_inside_failed_region", "motion_cleared_region",
)


def repeats(events, metrics):
    history = defaultdict(list)
    for event in events:
        if event["event"] != "route_recovery":
            continue
        robot, elapsed, xy = (event[key] for key in ("robot", "elapsed_sim_s", "stuck_xy"))
        previous = next((item for item in reversed(history[robot])
                         if 0 < elapsed - item["elapsed_sim_s"] <= REPEAT_WINDOW_S
                         and math.dist(xy, item["stuck_xy"]) <= REPEAT_RADIUS_M), None)
        history[robot].append(event)
        if previous is None:
            continue
        t0, origin = previous["elapsed_sim_s"], previous["stuck_xy"]
        samples = [math.dist((float(row[f"{robot}_x"]), float(row[f"{robot}_y"])), origin)
                   for row in metrics if t0 < float(row["elapsed_sim_s"]) < elapsed]
        distance = math.dist(xy, origin)
        farthest = max([distance, *samples])
        yield dict(robot=robot, previous_elapsed_s=t0, elapsed_s=elapsed,
                   repeat_distance_m=distance, sampled_max_distance_m=farthest,
                   position_samples=len(samples), sampled_inside_failed_region=(
                       bool(samples) and farthest < FAILED_ROUTE_RADIUS_M))


def episode_followup(outcomes, events, horizon_s):
    reports = defaultdict(list)
    for event in events:
        if event["event"] == "route_recovery":
            reports[event["robot"]].append(event)
    for event in outcomes:
        later = reports[event["robot"]]
        origin = next(report for report in later
                      if abs(report["report_stamp"] - event["start_s"]) <= RECOVERY_TIMESTAMP_SKEW_S)
        followup = min(REPEAT_WINDOW_S, max(0.0, horizon_s - event["end_s"]))
        recurrence = any(0 < report["report_stamp"] - event["end_s"] <= followup
                         and math.dist(report["stuck_xy"], origin["stuck_xy"]) <= REPEAT_RADIUS_M
                         for report in later)
        yield dict(event, duration_s=event["end_s"] - event["start_s"],
                   followup_s=followup, repeated_during_followup=recurrence)


def read_runs(directory, planned):
    manifest = json.loads((directory / "experiment_manifest.json").read_text())
    summaries = csv_rows(directory / "summary.csv")
    if (manifest["runs"] != planned or len(summaries) != len(planned)
            or any(any(row[key] != str(value) for key, value in condition.items())
                   for row, condition in zip(summaries, planned))):
        raise ValueError("Summary does not match the locked schedule")
    for row in summaries:
        key = {field: row[field] for field in (
            "scene", "environment_seed", "recovery_motion_mode", "layout_group")}
        name = f"{key['scene']}_{key['environment_seed']}_{key['recovery_motion_mode']}"
        record = json.loads((directory / f"{name}.complete.json").read_text())
        raw = directory / record["directory"]
        events = read_json_lines(raw / "decisions.jsonl")
        metrics = csv_rows(raw / "metrics.csv")
        repeated = [dict(**key, **event) for event in repeats(events, metrics)]
        logs = {robot: node_log(raw, robot + "_explorer") for robot in ROBOTS}
        outcomes = sorted(recovery_outcomes(logs), key=lambda item: item["start_s"])
        offset = next(event["control_stamp"] - event["elapsed_sim_s"] for event in events
                      if "control_stamp" in event and "elapsed_sim_s" in event)
        horizon_s = offset + float(metrics[-1]["elapsed_sim_s"])
        episodes = [dict(**key, **event) for event in episode_followup(outcomes, events, horizon_s)]
        full_followup = [event for event in episodes if event["followup_s"] == REPEAT_WINDOW_S]
        near = []
        for event in clearance_events(logs):
            ages = [event["stamp_s"] - finish["end_s"] for finish in outcomes
                    if finish["robot"] == event["robot"] and finish["end_s"] <= event["stamp_s"]]
            age = min(ages) if ages else None
            near.append(dict(**key, **event, seconds_since_recovery=age,
                             within_5s_of_recovery=age is not None and age <= 5.0))
        yield dict(**row, repeat_reports=len(repeated), repeats_inside_failed_region=sum(
            event["sampled_inside_failed_region"] for event in repeated),
            recovery_full_followups=len(full_followup), recovery_repeats_after_finish=sum(
                event["repeated_during_followup"] for event in full_followup),
            near_within_5s_of_recovery=sum(event["within_5s_of_recovery"] for event in near)
        ), repeated, episodes, near


def paired(rows):
    grouped = defaultdict(dict)
    for row in rows:
        grouped[(row["scene"], row["environment_seed"], row["layout_group"])][
            row["recovery_motion_mode"]] = row
    for (scene, seed, group), pair in grouped.items():
        fixed, adaptive = (pair[mode] for mode in MODES)
        yield dict(scene=scene, seed=seed, layout_group=group, **{
            f"{metric}_delta": round(float(adaptive[metric]) - float(fixed[metric]), 6)
            for metric in PAIR_METRICS})


def summarize(rows):
    for group in dict.fromkeys(["all", *(row["layout_group"] for row in rows)]):
        for mode in MODES:
            group_rows = [row for row in rows if row["recovery_motion_mode"] == mode
                          and (group == "all" or row["layout_group"] == group)]
            completed = sum(int(row["motion_completions"]) for row in group_rows)
            cleared = sum(int(row["motion_cleared_region"]) for row in group_rows)
            distance = sum(float(row["path_m"]) for row in group_rows)
            stuck = sum(int(row["stuck_events"]) for row in group_rows)
            near = sum(int(row["near_collision_events"]) for row in group_rows)
            yield dict(layout_group=group, mode=mode, runs=len(group_rows),
                       coverage_mean_pct=mean(float(row["coverage_pct"]) for row in group_rows),
                       crossed_runs=sum(int(row["robots_crossing"]) > 0 for row in group_rows),
                       runs_90=sum(bool(row["time_to_90_s"]) for row in group_rows),
                       runs_97=sum(bool(row["time_to_97_s"]) for row in group_rows),
                       stuck_total=stuck, stuck_per_10m=10.0 * stuck / distance if distance else None,
                       repeat_total=sum(row["repeat_reports"] for row in group_rows),
                       repeats_inside_failed_region=sum(row["repeats_inside_failed_region"]
                                                       for row in group_rows),
                       near_total=near, near_per_10m=10.0 * near / distance if distance else None,
                       path_mean_m=distance / len(group_rows),
                       near_within_5s_of_recovery=sum(row["near_within_5s_of_recovery"]
                                                       for row in group_rows),
                       recovery_outcomes=completed, cleared_region=cleared,
                       cleared_region_fraction=cleared / completed if completed else None,
                       recovery_full_followups=sum(row["recovery_full_followups"] for row in group_rows),
                       recovery_repeats_after_finish=sum(row["recovery_repeats_after_finish"]
                                                         for row in group_rows),
                       mean_recovery_displacement_m=(
                           sum(float(row["motion_logged_displacement_sum_m"]) for row in group_rows)
                           / completed if completed else None))


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                        "axes.spines.right": False, "pdf.fonttype": 42})
    figure, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    fields = ("coverage_pct", "repeat_reports", "motion_cleared_region")
    for index, mode in enumerate(MODES):
        subset = [row for row in rows if row["recovery_motion_mode"] == mode]
        positions = [j + (index - 0.5) * 0.30 for j in range(len(subset))]
        for axis, field in zip(axes, fields):
            axis.bar(positions, [float(row[field]) for row in subset], width=0.28,
                     color=("#345D85", "#50816C")[index],
                     label=("Fixed recovery", "Displacement-aware recovery")[index])
    baseline = [row for row in rows if row["recovery_motion_mode"] == MODES[0]]
    seeds = [row["environment_seed"] + ("*" if row["layout_group"] == "holdout" else "")
             for row in baseline]
    for axis, ylabel in zip(axes, ("Final coverage (%)", "Same-robot repeat reports",
                                  "Recoveries that cleared 0.65 m")):
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds, rotation=45, ha="right")
        axis.set_xlabel("Matched seed" + (" (* holdout)" if any("*" in seed for seed in seeds) else ""))
        axis.set_ylabel(ylabel)
    axes[0].set_ylim(0, 100)
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    figure.tight_layout(rect=(0, 0, 1, 0.89))
    for suffix in ("png", "pdf"):
        with (output / f"recovery_displacement_comparison.{suffix}").open("xb") as stream:
            figure.savefig(stream, format=suffix, dpi=220)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", choices=("smoke", "formal"), default="formal")
    args = parser.parse_args()
    planned, _ = schedule(args.stage)
    rows, repeated, episodes, near = [], [], [], []
    for row, repeat_rows, outcome_rows, near_rows in read_runs(args.runs, planned):
        rows.append(row)
        repeated.extend(repeat_rows)
        episodes.extend(outcome_rows)
        near.extend(near_rows)
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "derived_metrics.csv", rows)
    write_csv(args.output / "group_summary.csv", list(summarize(rows)))
    deltas = list(paired(rows))
    write_csv(args.output / "paired_deltas.csv", deltas)
    if repeated:
        write_csv(args.output / "repeat_reports.csv", repeated)
    if episodes:
        write_csv(args.output / "recovery_episodes.csv", episodes)
    if near:
        write_csv(args.output / "clearance_events.csv", near)
    plot(rows, args.output)


if __name__ == "__main__":
    main()
