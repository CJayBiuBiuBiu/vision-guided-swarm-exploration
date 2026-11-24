"""Summarize goal retention, action preference and matched coverage effects."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
from statistics import mean, median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import MODES, OPTIONS, RULES


BASE_METRICS = ("coverage_pct", "reached", "assignments", "path_m",
                "near_collision_events", "stuck_events", "recovery_successes")
COUNT_METRICS = ("actor_decisions", "active_actor_decisions", "active_actor_wait",
                 "active_wait_preferred_over_continue", "active_wait_fallbacks",
                 "goals_abandoned", "traced_blocked", "traced_stuck", "waits_deferred",
                 "targets_retained", "retained_targets_reached", "waypoint_retires_sent",
                 "waypoint_retires_received", "clearance_or_route_preemptions",
                 "low_lidar_sectors", "low_sectors_increased")
LAYOUTS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}
CUE_NAMES = {"lidar": "LiDAR", "guarded_fusion": "Guarded RGB"}
COLORS = ("#305E84", "#8F6245", "#3C7761", "#805385")


def arrival_rate(row):
    return 100 * float(row["reached"]) / int(row["assignments"])


def aggregate(rows):
    groups = defaultdict(list)
    for row in rows:
        groups["all", row["rule_cue"], row["wait_mode"]].append(row)
        groups[row["layout"], row["rule_cue"], row["wait_mode"]].append(row)
    output = []
    for (layout, cue, mode), entries in sorted(groups.items()):
        means = {key + "_mean": mean(float(row[key]) for row in entries) for key in BASE_METRICS}
        totals = {key + "_total": sum(int(row[key]) for row in entries) for key in
                  (*COUNT_METRICS, "assignments", "reached")}
        output.append(dict(
            layout=layout, rule_cue=cue, wait_mode=mode, runs=len(entries),
            **means, **totals,
            arrival_pct=100 * totals["reached_total"] / totals["assignments_total"],
            wait_pct=100 * totals["goals_abandoned_total"] / totals["assignments_total"],
            active_actor_wait_pct=100 * totals["active_actor_wait_total"] /
            totals["active_actor_decisions_total"]))
    return output


def matched(blocks):
    records = []
    within_cues = [
        (name, (cue, alternative), (cue, baseline))
        for cue in RULES
        for name, alternative, baseline in (
            ("cancel_minus_legacy", "cancel", "legacy"),
            ("hold_minus_cancel", "hold", "cancel"),
            ("hold_minus_legacy", "hold", "legacy"))]
    across_cues = [("guarded_minus_lidar", ("guarded_fusion", mode), ("lidar", mode))
                   for mode in MODES]
    for (layout, seed), group in sorted(blocks.items()):
        for name, alternative, baseline in within_cues + across_cues:
            a, b = group[alternative], group[baseline]
            effects = {key + "_delta": round(float(a[key]) - float(b[key]), 6) for key in BASE_METRICS}
            records.append(dict(layout=layout, seed=seed, contrast=name,
                                cue=alternative[0], mode=alternative[1],
                                reference_cue=baseline[0], reference_mode=baseline[1],
                                arrival_pct_delta=round(arrival_rate(a) - arrival_rate(b), 6), **effects))
        interaction = {}
        for metric in ("coverage_pct", "reached", "near_collision_events"):
            def cue_gap(mode):
                return float(group["guarded_fusion", mode][metric]) - float(group["lidar", mode][metric])
            interaction[metric + "_delta"] = round(cue_gap("hold") - cue_gap("cancel"), 6)
        records.append(dict(layout=layout, seed=seed, contrast="hold_changes_rgb_gain",
                            cue="guarded_fusion", mode="hold", reference_cue="lidar",
                            reference_mode="cancel", arrival_pct_delta=None,
                            **{key + "_delta": interaction.get(key + "_delta") for key in BASE_METRICS}))
    return records


def actor_preferences(study, rows):
    margins = defaultdict(list)
    for row in rows:
        name = f"{row['layout']}_{row['environment_seed']}_{row['rule_cue']}_{row['wait_mode']}"
        marker = json.loads((study / f"{name}.complete.json").read_text())
        path = study / marker["directory"] / "actions.jsonl"
        for line in path.open():
            event = json.loads(line)
            margin = event["wait_minus_continue"]
            if event["active_goal"] and margin is not None and event["selectable_frontiers"] == 0:
                for cue, mode in (("all", "all"), (row["rule_cue"], row["wait_mode"])):
                    margins[cue, mode].append((float(margin), event["action_type"]))
    result = []
    for (cue, mode), entries in sorted(margins.items()):
        logits = [margin for margin, _ in entries]
        result.append(dict(
            rule_cue=cue, wait_mode=mode, wait_or_continue_only=len(entries),
            wait_selected=sum(action == 2 for _, action in entries),
            median_wait_minus_continue=median(logits),
            average_conditional_wait_probability=mean(1 / (1 + math.exp(-value)) for value in logits)))
    if not result:
        raise ValueError("Expected observations with just WAIT and CONTINUE enabled")
    return result


def idle_sampled_motion(study, rows):
    result = []
    for row in rows:
        key = f"{row['layout']}_{row['environment_seed']}_{row['rule_cue']}_{row['wait_mode']}"
        marker = json.loads((study / f"{key}.complete.json").read_text())
        directory = study / marker["directory"]
        snapshots = defaultdict(list)
        for tick in csv_rows(directory / "metrics.csv"):
            stamp = float(tick["elapsed_sim_s"])
            for robot, xy in json.loads(tick["robot_positions_json"]).items():
                snapshots[robot].append((stamp, xy))
        waiting, distances = {}, []
        intervals = 0
        for line in (directory / "decisions.jsonl").open():
            event = json.loads(line)
            if event["event"] == "outcome" and event["outcome"] == "wait":
                waiting[event["robot"]] = event["elapsed_sim_s"]
            elif event["event"] == "assigned" and event["robot"] in waiting:
                robot = event["robot"]
                start, end = waiting.pop(robot), event["elapsed_sim_s"]
                intervals += 1
                positions = [xy for time, xy in snapshots[robot] if start + 0.25 <= time < end]
                if len(positions) >= 2:
                    distances.append(sum(math.dist(a, b) for a, b in zip(positions, positions[1:])))
        result.append(dict(
            layout=row["layout"], seed=row["environment_seed"], rule_cue=row["rule_cue"],
            wait_mode=row["wait_mode"], closed_wait_intervals=intervals,
            sampled_wait_intervals=len(distances), sampled_wait_intervals_moving=sum(
                distance > 0.05 for distance in distances),
            average_sampled_idle_path_m=mean(distances) if distances else None))
    return result


def plot(blocks, destination):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "axes.grid": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 6.0), sharex="col", sharey="row")
    highest_arrival_pct = max(arrival_rate(row) for block in blocks.values() for row in block.values())
    arrival_axis_limit = min(102, 5 * math.ceil((highest_arrival_pct + 3) / 5))
    for index, ((layout, seed), block) in enumerate(sorted(blocks.items())):
        column = list(LAYOUTS).index(layout)
        for cue in RULES:
            records = [block[cue, mode] for mode in MODES]
            dash = "-" if cue == "lidar" else "--"
            marker = "o" if cue == "lidar" else "s"
            axes[0, column].plot(range(3), [float(row["coverage_pct"]) for row in records],
                                 dash, marker=marker, color=COLORS[index], alpha=0.85,
                                 label=f"{seed} {CUE_NAMES[cue]}")
            axes[1, column].plot(range(3), [arrival_rate(row) for row in records], dash,
                                 marker=marker, color=COLORS[index], alpha=0.85)
    for (top, bottom), title in zip(zip(*axes), LAYOUTS.values()):
        top.set_title(title, loc="left", fontweight="bold")
        top.set_ylim(0, 102)
        top.legend(frameon=False, fontsize=8)
        bottom.set_ylim(0, arrival_axis_limit)
        bottom.set_xticks(range(3))
        bottom.set_xticklabels(("Legacy", "Explicit cancel", "Short hold"))
    axes[0, 0].set_ylabel("Final coverage at 60 s (%)")
    axes[1, 0].set_ylabel("Goals reached / assigned (%)")
    fig.tight_layout()
    for kind in ("pdf", "png"):
        with (destination / f"goal_retention.{kind}").open("xb") as stream:
            fig.savefig(stream, format=kind, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    planned = json.loads((args.formal / "experiment_manifest.json").read_text())["runs"]
    keys = tuple(planned[0])
    if (len(rows) != 24 or [tuple(row[key] for key in keys) for row in rows] !=
            [tuple(str(row[key]) for key in keys) for row in planned]):
        raise ValueError("Run results must match all four predetermined blocks")
    blocks = defaultdict(dict)
    for row in rows:
        blocks[row["layout"], row["environment_seed"]][row["rule_cue"], row["wait_mode"]] = row
    if any(set(group) != set(OPTIONS) for group in blocks.values()):
        raise ValueError("A matched seed is missing an experiment condition")
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "group_metrics.csv", aggregate(rows))
    write_csv(args.output / "paired_contrasts.csv", matched(blocks))
    write_csv(args.output / "actor_preferences.csv", actor_preferences(args.formal, rows))
    write_csv(args.output / "sampled_motion_after_wait.csv", idle_sampled_motion(args.formal, rows))
    plot(blocks, args.output)


if __name__ == "__main__":
    main()
