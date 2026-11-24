"""Report paired coverage, low-score preservation and the rule fusion comparisons."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean, median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows, write_csv


LABELS = {"lidar": "LiDAR", "old_fixed": "Archived RGB",
          "new_fixed": "Calibrated RGB", "new_guarded": "Calibrated + guard"}
METRICS = ("coverage_pct", "path_m", "goals_reached", "near_collision_events", "stuck_events",
           "recovery_successes")
CONTRASTS = (("new_fixed", "old_fixed"), ("new_guarded", "new_fixed"), ("new_guarded", "lidar"))
LAYOUTS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}


def group_results(rows):
    groups = defaultdict(list)
    for row in rows:
        groups["all", row["method"]].append(row)
        groups[row["layout"], row["method"]].append(row)
    out = []
    for (layout, method), group in sorted(groups.items()):
        averages = {f"mean_{key}": mean(float(row[key]) for row in group) for key in METRICS}
        low = sum(int(row["low_lidar_sectors"]) for row in group)
        raised = sum(int(row["low_score_raised"]) for row in group)
        delta = (sum(int(row["low_lidar_sectors"]) * float(row["average_low_score_delta"])
                     for row in group if row["average_low_score_delta"]) / low) if low else None
        out.append(dict(layout=layout, method=method, runs=len(group), **averages,
                        low_lidar_sectors=low, low_score_raised=raised, mean_low_score_delta=delta,
                        reached_90=sum(bool(row["time_to_90_s"]) for row in group),
                        reached_97=sum(bool(row["time_to_97_s"]) for row in group)))
    return out


def paired(blocks):
    out = []
    for (layout, seed), group in sorted(blocks.items()):
        for preferred, baseline in CONTRASTS:
            a, b = group[preferred], group[baseline]
            result = dict(layout=layout, seed=seed, method=preferred, baseline=baseline,
                          method_coverage=float(a["coverage_pct"]), baseline_coverage=float(b["coverage_pct"]))
            result.update({f"{key}_delta": round(float(a[key]) - float(b[key]), 5) for key in METRICS})
            for key in ("time_to_90_s", "time_to_97_s"):
                result[f"{key}_method"] = float(a[key]) if a[key] else None
                result[f"{key}_baseline"] = float(b[key]) if b[key] else None
                result[f"{key}_delta"] = (round(float(a[key]) - float(b[key]), 5)
                                          if a[key] and b[key] else None)
            out.append(result)
    return out


def assignment_outcomes(study, rows):
    grouped = defaultdict(lambda: {"assignments": 0, "reached": 0, "wait_durations": []})
    for row in rows:
        key = f"{row['layout']}_{row['environment_seed']}_{row['method']}"
        seal = json.loads((study / f"{key}.complete.json").read_text())
        trace = study / seal["directory"] / "decisions.jsonl"
        assignments = {}
        summary = grouped[row["method"]]
        for line in trace.open():
            event = json.loads(line)
            if event["event"] == "assigned":
                assignments[event["attempt_id"]] = event["elapsed_sim_s"]
                summary["assignments"] += 1
            elif event["event"] == "outcome":
                if event["outcome"] == "reached":
                    summary["reached"] += 1
                if event["outcome"] == "wait":
                    summary["wait_durations"].append(
                        event["elapsed_sim_s"] - assignments[event["attempt_id"]])
    return [dict(method=method, assignments=stats["assignments"], reached=stats["reached"],
                 ended_by_wait=len(stats["wait_durations"]),
                 median_wait_delay_s=median(stats["wait_durations"]) if stats["wait_durations"] else None)
            for method, stats in sorted(grouped.items())]


def plot(groups, output):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "axes.grid": False, "pdf.fonttype": 42})
    figure, axes = plt.subplots(1, 2, figsize=(9.7, 3.8), sharey=True)
    colors = ("#316D95", "#A57143", "#42785B", "#795B86")
    for ax, label in zip(axes, LAYOUTS.values()):
        ax.set_title(label, loc="left", fontweight="bold")
        ax.set_ylim(0, 102)
        ax.set_xticks(range(len(LABELS)))
        ax.set_xticklabels(tuple(LABELS.values()), rotation=20, ha="right")
    for index, ((world, seed), group) in enumerate(sorted(groups.items())):
        axis = axes[list(LAYOUTS).index(world)]
        values = [float(group[name]["coverage_pct"]) for name in LABELS]
        axis.plot(range(len(LABELS)), values, marker="o", color=colors[index], label=f"Seed {seed}")
    for axis in axes:
        axis.legend(loc="lower right", frameon=False)
    axes[0].set_ylabel("Final coverage at 75 s (%)")
    figure.tight_layout()
    for extension in ("pdf", "png"):
        with (output / f"paired_coverage.{extension}").open("xb") as stream:
            figure.savefig(stream, format=extension, dpi=220)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.formal / "experiment_manifest.json").read_text())
    rows = csv_rows(args.formal / "summary.csv")
    keys = tuple(manifest["runs"][0])
    if (len(rows) != 16 or [tuple(row[key] for key in keys) for row in rows] !=
            [tuple(str(row[key]) for key in keys) for row in manifest["runs"]]):
        raise ValueError("The run results do not match the sealed schedule")
    blocks = defaultdict(dict)
    for row in rows:
        blocks[row["layout"], row["environment_seed"]][row["method"]] = row
    if any(set(group) != set(LABELS) for group in blocks.values()):
        raise ValueError("A pair block is incomplete")
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "paired_differences.csv", paired(blocks))
    write_csv(args.output / "group_metrics.csv", group_results(rows))
    write_csv(args.output / "assignment_outcomes.csv", assignment_outcomes(args.formal, rows))
    plot(blocks, args.output)


if __name__ == "__main__":
    main()
