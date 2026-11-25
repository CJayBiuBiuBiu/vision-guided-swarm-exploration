"""Report movement and frontier changes across held-out passage layouts."""

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_dynamic_passage_study import MODES, schedule
from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines


FIELDS = ("coverage_pct", "path_m", "robots_crossing", "reached", "assignments",
          "near_collision_events", "passage_decisions", "fresh_frame_pairs", "preferred_scored",
          "preferred_bottleneck", "would_rerank", "applied_reranks", "reranked_targets_reached")


def paired(rows):
    blocks = defaultdict(dict)
    for row in rows:
        blocks[row["scene"]][row["passage_mode"]] = row
    for scene, group in blocks.items():
        if set(group) != set(MODES):
            raise ValueError(f"Incomplete comparison for {scene}")
        a, b = (group[mode] for mode in MODES)
        numeric = {key + "_delta": round(float(b[key]) - float(a[key]), 6) for key in FIELDS}
        yield dict(scene=scene, seed=a["environment_seed"],
                   observe_first_crossing_s=a["first_crossing_s"] or None,
                   rerank_first_crossing_s=b["first_crossing_s"] or None,
                   observe_post_crossing_gain_pp=a["post_crossing_gain_pp"] or None,
                   rerank_post_crossing_gain_pp=b["post_crossing_gain_pp"] or None,
                   **numeric)


def totals(rows):
    for mode in MODES:
        group = [r for r in rows if r["passage_mode"] == mode]
        yield dict(mode=mode, runs=len(group), runs_with_crossing=sum(int(r["robots_crossing"]) > 0 for r in group),
                   coverage_mean=mean(float(r["coverage_pct"]) for r in group),
                   **{key + "_total": sum(int(r[key]) for r in group) for key in FIELDS[2:]})


def eligibility(formal, rows):
    reasons = ("no_preferred_path", "no_scored_alternative", "insufficient_clear_gain",
               "no_alternative_path", "extra_route_too_long", "eligible")
    for row in rows:
        mode = row["passage_mode"]
        trial = f"{row['scene']}_{row['environment_seed']}_{mode}"
        marker = json.loads((formal / f"{trial}.complete.json").read_text())
        events = read_json_lines(formal / marker["directory"] / "decisions.jsonl")
        counts = Counter()
        for event in events:
            if event["event"] != "passage_width_decision":
                continue
            options = {item["frontier_id"]: item for item in event["candidates"]}
            preferred = options.pop(event["preferred"])
            score = preferred["score"]
            if score is None or score["bottleneck_m"] is None:
                continue
            scored = [item for item in options.values() if item["score"] is not None]
            better = [item for item in scored if item["score"]["clear_to_m"] >= score["clear_to_m"] + 0.75]
            paths = [item["route_m"] for item in better if item["route_m"] is not None]
            if preferred["route_m"] is None:
                counts["no_preferred_path"] += 1
            elif not scored:
                counts["no_scored_alternative"] += 1
            elif not better:
                counts["insufficient_clear_gain"] += 1
            elif not paths:
                counts["no_alternative_path"] += 1
            elif min(paths) > preferred["route_m"] + 0.600000001:
                counts["extra_route_too_long"] += 1
            else:
                counts["eligible"] += 1
        if sum(counts.values()) != int(row["preferred_bottleneck"]) or counts["eligible"] != int(row["would_rerank"]):
            raise ValueError(f"Rerank eligibility differs from the verified result for {trial}")
        yield dict(scene=row["scene"], seed=row["environment_seed"], mode=mode,
                   **{reason: counts[reason] for reason in reasons})


def interventions(formal, rows, geometries):
    for row in rows:
        mode = row["passage_mode"]
        trial = f"{row['scene']}_{row['environment_seed']}_{mode}"
        entry = json.loads((formal / f"{trial}.complete.json").read_text())
        events = read_json_lines(formal / entry["directory"] / "decisions.jsonl")
        outcomes = {e["attempt_id"]: e["outcome"] for e in events if e["event"] == "outcome"}
        decisions = [e for e in events if e["event"] in ("passage_width_decision", "assigned")]
        wall_y = geometries[row["scene"]]["wall_y"]
        for choice, issued in zip(decisions[::2], decisions[1::2]):
            if choice["would_select"] == choice["preferred"]:
                continue
            candidates = {item["frontier_id"]: item for item in choice["candidates"]}
            a, b = (candidates[choice[key]] for key in ("preferred", "would_select"))
            xy = issued["selected_xy"]
            yield dict(scene=row["scene"], seed=row["environment_seed"], mode=mode,
                       robot=choice["robot"], elapsed_sim_s=choice["elapsed_sim_s"],
                       original_frontier=choice["preferred"], proposed_frontier=choice["would_select"],
                       applied=choice["selected"] == choice["would_select"],
                       original_clear_to_m=a["score"]["clear_to_m"], proposed_clear_to_m=b["score"]["clear_to_m"],
                       extra_route_m=round(b["route_m"] - a["route_m"], 6),
                       issued_x=xy[0], issued_y=xy[1], partition_y=wall_y,
                       issued_north_of_partition=xy[1] > wall_y,
                       outcome=outcomes[issued["attempt_id"]])


def plot(rows, destination):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.7))
    for index, mode in enumerate(MODES):
        subset = [row for row in rows if row["passage_mode"] == mode]
        positions = [i + (index - 0.5) * 0.28 for i in range(len(subset))]
        label = ("RGB shadow", "RGB reranking")[index]
        color = ("#365F82", "#9B6646")[index]
        axes[0].bar(positions, [float(r["coverage_pct"]) for r in subset],
                    width=0.26, label=label, color=color)
        for x, row in zip(positions, subset):
            event = row["first_crossing_s"]
            axes[1].plot(x, float(event or row["horizon"]), color=color, markersize=6,
                         marker="o" if event else "^", markerfacecolor=color if event else "none")
    seeds = [row["environment_seed"] for row in rows if row["passage_mode"] == MODES[0]]
    for axis in axes:
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds)
        axis.set_xlabel("Held-out layout seed")
    axes[0].set_ylim(0, 100)
    axes[0].set_ylabel("Final coverage (%)")
    axes[1].set_ylim(0, 96)
    axes[1].set_ylabel("Time to first crossing (s)")
    axes[1].text(0.03, 0.03, "Open triangle: no crossing within 90 s",
                 transform=axes[1].transAxes, fontsize=8)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    for extension in ("png", "pdf"):
        with (destination / f"unknown_passage_pairs.{extension}").open("xb") as stream:
            fig.savefig(stream, format=extension, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    runs = json.loads((args.formal / "experiment_manifest.json").read_text())["runs"]
    planned, geometries = schedule("formal")
    if (runs != planned or len(rows) != len(runs)
            or any(any(row[key] != str(value) for key, value in run.items())
                   for row, run in zip(rows, runs))):
        raise ValueError("Summary differs from the complete locked schedule")
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "paired_layouts.csv", list(paired(rows)))
    write_csv(args.output / "group_summary.csv", list(totals(rows)))
    write_csv(args.output / "rerank_eligibility.csv", list(eligibility(args.formal, rows)))
    intervention_rows = list(interventions(args.formal, rows, geometries))
    if intervention_rows:
        write_csv(args.output / "interventions.csv", intervention_rows)
    plot(rows, args.output)


if __name__ == "__main__":
    main()
