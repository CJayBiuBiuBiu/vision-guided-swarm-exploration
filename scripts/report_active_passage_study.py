"""Summarise matched active-passage rollouts and inspection outcomes."""

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_active_passage_study import STRATEGIES, schedule
from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines


METRICS = ("coverage_pct", "path_m", "robots_crossing", "goals_reached", "stuck_events",
           "near_collision_events", "applied_reranks", "inspection_redirects",
           "continuations_issued", "continuation_segments_arrived")


def summarise(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[row["scene"]][row["passage_strategy"]] = row
    for scene, group in groups.items():
        if set(group) != set(STRATEGIES):
            raise ValueError(f"Incomplete paired layout {scene}")
        passive, active = (group[strategy] for strategy in STRATEGIES)
        yield dict(scene=scene, seed=passive["environment_seed"],
                   passive_first_crossing_s=passive["first_crossing_s"] or None,
                   active_first_crossing_s=active["first_crossing_s"] or None,
                   **{field + "_delta": round(float(active[field]) - float(passive[field]), 6)
                      for field in METRICS})


def totals(rows):
    for strategy in STRATEGIES:
        subset = [row for row in rows if row["passage_strategy"] == strategy]
        yield dict(strategy=strategy, runs=len(subset),
                   runs_with_crossing=sum(int(row["robots_crossing"]) > 0 for row in subset),
                   **{field + "_mean": mean(float(row[field]) for row in subset) for field in METRICS},
                   **{field + "_total": sum(int(row[field]) for row in subset) for field in (
                       "inspections_started", "inspections_scored", "inspection_rebound",
                       "inspection_redirects_reached", "continuation_checks", "continuation_waits_kept")})


def inspection_results(formal, rows):
    for row in rows:
        name = f"{row['scene']}_{row['environment_seed']}_{row['passage_strategy']}"
        marker = json.loads((formal / f"{name}.complete.json").read_text())
        counts = Counter()
        scans = {}
        for event in read_json_lines(formal / marker["directory"] / "decisions.jsonl"):
            activity = event["event"]
            if activity == "passage_inspection_start":
                scans[event["robot"]] = event
            elif activity == "passage_inspection_result":
                start = scans.pop(event["robot"])
                counts[event["reason"]] += 1
                yield dict(scene=row["scene"], seed=row["environment_seed"], robot=event["robot"],
                           start_s=start["elapsed_sim_s"], duration_s=event["duration_s"],
                           reason=event["reason"], preferred=start["preferred"],
                           alternative=start["alternative"], selected=event["selected"],
                           resolved_preferred=event["resolved_preferred"],
                           resolved_alternative=event["resolved_alternative"],
                           original_clear_m=start["preferred_score"]["clear_to_m"],
                           alternative_clear_m=None if event["score"] is None else event["score"]["clear_to_m"])
            elif activity == "passage_inspection_cancel":
                scans.pop(event["robot"])
        if sum(counts.values()) != int(row["inspections_finished"]):
            raise ValueError(f"Unreconciled inspection trace {name}")


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False, "axes.spines.right": False,
                         "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.65))
    for i, strategy in enumerate(STRATEGIES):
        subset = [row for row in rows if row["passage_strategy"] == strategy]
        xs = [j + (i - 0.5) * 0.30 for j in range(len(subset))]
        color = ("#365F82", "#9B6646")[i]
        label = ("Passive reranking", "Active inspection + continuation")[i]
        axes[0].bar(xs, [float(r["coverage_pct"]) for r in subset], width=0.28, color=color, label=label)
        for x, row in zip(xs, subset):
            time = row["first_crossing_s"]
            axes[1].plot(x, float(time or row["horizon"]), marker="o" if time else "^",
                         color=color, markerfacecolor=color if time else "none", markersize=6)
        axes[2].bar(xs, [int(r["robots_crossing"]) for r in subset], width=0.28, color=color)
    seeds = [row["environment_seed"] for row in rows if row["passage_strategy"] == STRATEGIES[0]]
    for axis in axes:
        axis.set_xticks(range(len(seeds)), seeds, rotation=30, ha="right")
        axis.set_xlabel("Unseen layout seed")
    axes[0].set_ylabel("Final coverage (%)")
    axes[0].set_ylim(0, 100)
    axes[1].set_ylabel("First crossing (s)")
    axes[1].set_ylim(0, 127)
    axes[1].text(0.03, 0.04, "Open triangle: no crossing", fontsize=8, transform=axes[1].transAxes)
    axes[2].set_ylabel("Robots crossing")
    axes[2].set_yticks((0, 1, 2, 3))
    axes[2].set_ylim(0, 3.3)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    for ext in ("png", "pdf"):
        with (output / f"active_passage_pairs.{ext}").open("xb") as stream:
            fig.savefig(stream, format=ext, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    manifest = json.loads((args.formal / "experiment_manifest.json").read_text())
    runs, _ = schedule("formal")
    if (manifest["runs"] != runs or len(rows) != len(runs)
            or any(any(row[key] != str(value) for key, value in run.items()) for row, run in zip(rows, runs))):
        raise ValueError("Summary does not match the locked paired schedule")
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "paired_layouts.csv", list(summarise(rows)))
    write_csv(args.output / "group_summary.csv", list(totals(rows)))
    inspection_rows = list(inspection_results(args.formal, rows))
    if inspection_rows:
        write_csv(args.output / "inspection_results.csv", inspection_rows)
    plot(rows, args.output)


if __name__ == "__main__":
    main()
