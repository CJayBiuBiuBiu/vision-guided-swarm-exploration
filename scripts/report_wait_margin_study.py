"""Report paired navigation outcomes and the use of the MAPPO route recheck."""

import argparse
from collections import defaultdict
from itertools import combinations
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows, write_csv
from run_wait_margin_study import BLOCKS, OPTIONS


CUES = ("lidar", "guarded_fusion")
STRATEGIES = ("cancel", "hold", "margin")
METRICS = ("coverage_pct", "path_m", "reached", "assignments", "near_collision_events",
           "stuck_events", "recovery_successes")
COUNTS = ("active_decisions", "raw_active_waits", "published_active_waits", "goals_abandoned",
          "recheck_requests", "rechecks_consumed", "rechecks_accepted", "rechecked_targets",
          "rechecked_targets_reached", "recheck_age_expired", "recheck_clearance_rejected",
          "recheck_disconnected", "recheck_no_goal", "recheck_goal_changed",
          "waypoint_retires_sent", "waypoint_retires_received")


def arrival_rate(row):
    return 100 * int(row["reached"]) / int(row["assignments"])


def aggregate(rows):
    groups = defaultdict(list)
    for row in rows:
        groups["all", row["rule_cue"], row["strategy"]].append(row)
        groups[row["layout"], row["rule_cue"], row["strategy"]].append(row)
    for (layout, cue, strategy), entries in sorted(groups.items()):
        sums = {key + "_total": sum(int(row[key]) for row in entries)
                for key in ("reached", "assignments", *COUNTS)}
        means = {key + "_mean": mean(float(row[key]) for row in entries) for key in METRICS}
        yield dict(layout=layout, rule_cue=cue, strategy=strategy, runs=len(entries),
                   **means, **sums, arrival_pct=100 * sums["reached_total"] / sums["assignments_total"])


def matched(blocks):
    within = [(f"{new}_minus_{old}", (cue, new), (cue, old))
              for cue in CUES for old, new in combinations(STRATEGIES, 2)]
    across = [("guarded_minus_lidar", ("guarded_fusion", mode), ("lidar", mode))
              for mode in STRATEGIES]
    for (layout, seed), group in blocks.items():
        for contrast, alternative, baseline in (*within, *across):
            a, b = group[alternative], group[baseline]
            yield dict(layout=layout, seed=seed, contrast=contrast, rule_cue=alternative[0],
                       strategy=alternative[1], reference_cue=baseline[0], reference_strategy=baseline[1],
                       arrival_pct_delta=round(arrival_rate(a) - arrival_rate(b), 6),
                       **{key + "_delta": round(float(a[key]) - float(b[key]), 6) for key in METRICS})


def plot(contrasts, destination):
    selected = [r for r in contrasts if r["contrast"] == "margin_minus_hold"]
    seeds = [str(seed) for _, seed, _ in BLOCKS]
    colors = ("#305E84", "#A66B49")
    plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.7))
    for axis, metric, ylabel in zip(
            axes, ("coverage_pct_delta", "arrival_pct_delta"),
            ("Coverage change (pp)", "Goal arrival rate change (pp)")):
        axis.axhline(0, color="#707070", linewidth=0.8)
        for i, (cue, label) in enumerate(zip(CUES, ("LiDAR", "Guarded RGB"))):
            group = [r for r in selected if r["rule_cue"] == cue]
            axis.bar([seeds.index(r["seed"]) + 0.36 * (i - 0.5) for r in group],
                     [r[metric] for r in group], width=0.33, color=colors[i], label=label)
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds)
        axis.set_ylabel(ylabel)
        axis.set_xlabel("Matched environment seed")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    for extension in ("png", "pdf"):
        with (destination / f"margin_vs_short_hold.{extension}").open("xb") as stream:
            fig.savefig(stream, format=extension, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    manifest = json.loads((args.formal / "experiment_manifest.json").read_text())
    schedule = manifest["runs"]
    if (len(rows) != 24 or len(schedule) != 24 or
            any(any(row[key] != str(value) for key, value in run.items())
                for row, run in zip(rows, schedule))):
        raise ValueError("The summary must contain the complete planned comparison")
    blocks = defaultdict(dict)
    for row in rows:
        name = f"{row['layout']}_{row['environment_seed']}_{row['rule_cue']}_{row['strategy']}"
        sealed = json.loads((args.formal / f"{name}.complete.json").read_text())["result"]
        if {key: "" if value is None else str(value) for key, value in sealed.items()} != row:
            raise ValueError(f"Summary and sealed result differ for {name}")
        blocks[row["layout"], row["environment_seed"]][row["rule_cue"], row["strategy"]] = row
    if len(blocks) != len(BLOCKS) or any(set(group) != set(OPTIONS) for group in blocks.values()):
        raise ValueError("Each seed must have all six conditions")
    args.output.mkdir(parents=True, exist_ok=True)
    contrasts = list(matched(blocks))
    write_csv(args.output / "group_metrics.csv", list(aggregate(rows)))
    write_csv(args.output / "paired_contrasts.csv", contrasts)
    plot(contrasts, args.output)


if __name__ == "__main__":
    main()
