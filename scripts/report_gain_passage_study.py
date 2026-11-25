"""Report gain-region, active-width and passive-width exploration on matched layouts."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from statistics import mean, median, stdev

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_gain_passage_study import STRATEGIES, schedule
from run_dynamic_passage_study import crossing_audit
from run_matched_vision_training import csv_rows, write_csv
from run_policy_goal_retention import read_json_lines


METRICS = ("coverage_pct", "path_m", "robots_crossing", "goals_reached", "stuck_events",
           "near_collision_events", "applied_reranks", "inspection_redirects",
           "continuations_issued", "continuation_segments_arrived")


def comparisons(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[row["scene"]][row["passage_strategy"]] = row
    for scene, group in groups.items():
        if set(group) != set(STRATEGIES):
            raise ValueError(f"Incomplete matched layout {scene}")
        gain = group["gain_region"]
        for control in ("passive", "active"):
            base = group[control]
            yield dict(scene=scene, seed=base["environment_seed"], control=control,
                       control_first_crossing_s=base["first_crossing_s"] or None,
                       gain_first_crossing_s=gain["first_crossing_s"] or None,
                       **{field + "_delta": round(float(gain[field]) - float(base[field]), 6)
                          for field in METRICS})


def totals(rows):
    for strategy in STRATEGIES:
        subset = [row for row in rows if row["passage_strategy"] == strategy]
        yield dict(strategy=strategy, runs=len(subset),
                   runs_with_crossing=sum(int(row["robots_crossing"]) > 0 for row in subset),
                   runs_90=sum(bool(row["time_to_90_s"]) for row in subset),
                   runs_97=sum(bool(row["time_to_97_s"]) for row in subset),
                   coverage_sd=stdev(float(row["coverage_pct"]) for row in subset),
                   **{field + "_mean": mean(float(row[field]) for row in subset) for field in METRICS},
                   **{field + "_total": sum(int(row[field]) for row in subset) for field in (
                       "inspections_started", "inspections_scored", "inspections_finished",
                       "inspection_redirects_reached", "reranked_targets_reached", "inspection_rebound",
                       "continuation_checks", "continuation_waits_kept", "gain_advantage_cells",
                       "inspections_with_missing_ticks", "missing_region_ticks", "region_gain_depleted",
                       "region_unavailable", "redirects_after_missing_region")})


def matched_summary(pairs):
    for control in ("passive", "active"):
        subset = [row["coverage_pct_delta"] for row in pairs if row["control"] == control]
        yield dict(control=control, pairs=len(subset), gain_better=sum(value > 0 for value in subset),
                   coverage_delta_mean_pp=mean(subset), coverage_delta_median_pp=median(subset),
                   coverage_delta_min_pp=min(subset), coverage_delta_max_pp=max(subset))


def run_data(formal, row):
    name = f"{row['scene']}_{row['environment_seed']}_{row['passage_strategy']}"
    marker = json.loads((formal / f"{name}.complete.json").read_text())
    directory = formal / marker["directory"]
    return read_json_lines(directory / "decisions.jsonl"), csv_rows(directory / "metrics.csv")


def inspection_results(formal, rows):
    for row in rows:
        events, _ = run_data(formal, row)
        inspections = {}
        count = 0
        for event in events:
            activity, robot = event["event"], event.get("robot")
            if activity in ("gain_inspection_start", "passage_inspection_start"):
                inspections[robot] = event
            elif activity in ("gain_inspection_result", "passage_inspection_result"):
                started = inspections.pop(robot)
                count += 1
                gain_mode = activity == "gain_inspection_result"
                observed = (event["anchor_score"] is not None if gain_mode else event["score"] is not None)
                candidate_scored = (any(candidate["score"] is not None for candidate in event["candidates"])
                                    if gain_mode else observed)
                yield dict(scene=row["scene"], seed=row["environment_seed"],
                           strategy=row["passage_strategy"], robot=robot,
                           start_s=started["elapsed_sim_s"], duration_s=event["duration_s"],
                           reason=event["reason"], anchor_or_original_score=observed,
                           current_candidate_scored=candidate_scored, preferred=event["preferred"],
                           alternative=event["alternative"], selected=event["selected"],
                           base_gain_cells=event.get("base_gain_cells"), missing_ticks=event.get("missing_ticks"),
                           selected_gain_cells=next((candidate["gain_cells"] for candidate in event.get("candidates", [])
                                                     if candidate["frontier_id"] == event["selected"]), None))
            elif activity == "passage_inspection_cancel":
                inspections.pop(robot)
        if count != int(row["inspections_finished"]):
            raise ValueError(f"Inspection trace differs for {row['scene']}")


def redirects(formal, rows, geometries):
    for row in rows:
        events, metrics = run_data(formal, row)
        geometry = geometries[row["scene"]]
        times = {}
        for robot in ("robot1", "robot2", "robot3"):
            positions = [dict(item, robot_positions_json=json.dumps({
                robot: json.loads(item["robot_positions_json"])[robot]})) for item in metrics]
            times[robot] = crossing_audit(positions, geometry)["first_crossing_s"]
        outcomes = {e["attempt_id"]: e for e in events if e["event"] == "outcome"}
        pending = {}
        for event in events:
            kind, robot = event["event"], event.get("robot")
            if ((kind in ("gain_passage_decision", "passage_width_decision")
                 and event["selected"] != event["preferred"])
                    or (kind in ("gain_inspection_result", "passage_inspection_result")
                        and event["reason"] == "redirected")):
                pending[robot] = event
            elif kind == "assigned" and robot in pending:
                choice = pending.pop(robot)
                outcome = outcomes[event["attempt_id"]]
                crossing = times[robot]
                gain = next((entry.get("gain_cells") for entry in choice.get("candidates", [])
                             if entry["frontier_id"] == choice["selected"]), None)
                base = (choice["candidates"][0].get("gain_cells") if choice["event"] == "gain_passage_decision"
                        else choice.get("base_gain_cells"))
                yield dict(scene=row["scene"], seed=row["environment_seed"], strategy=row["passage_strategy"],
                           robot=robot, source=choice["event"], redirect_s=event["elapsed_sim_s"],
                           original_frontier=choice["preferred"], selected_frontier=event["selected_frontier_id"],
                           goal_x=event["selected_xy"][0], goal_y=event["selected_xy"][1],
                           partition_y=geometry["wall_y"], baseline_gain_cells=base, selected_gain_cells=gain,
                           outcome=outcome["outcome"], outcome_s=outcome["elapsed_sim_s"],
                           team_area_gain_m2=outcome["team_area_gain_m2"],
                           first_robot_crossing_s=crossing,
                           crossed_before=(crossing is not None and crossing < event["elapsed_sim_s"]),
                           crossed_after=(crossing is not None and crossing >= event["elapsed_sim_s"]))


def continuations(formal, rows):
    for row in rows:
        events, _ = run_data(formal, row)
        for event in events:
            if event["event"] != "passage_continuation_decision":
                continue
            target = event["target_xy"]
            yield dict(scene=row["scene"], seed=row["environment_seed"],
                       strategy=row["passage_strategy"], robot=event["robot"],
                       elapsed_s=event["elapsed_sim_s"], frontier_id=event["frontier_id"],
                       completed_segments=event["step"], gain_cells=event.get("gain_cells"),
                       clear_distance_m=None if event["score"] is None else event["score"]["clear_to_m"],
                       origin_xy=json.dumps(event["origin_xy"]),
                       target_xy=None if target is None else json.dumps(target),
                       extension_m=None if target is None else math.dist(event["origin_xy"], target))


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.65))
    palette = ("#365F82", "#9B6646", "#4B8269")
    labels = ("Passive width", "Active width", "Gain-region")
    for i, strategy in enumerate(STRATEGIES):
        subset = [row for row in rows if row["passage_strategy"] == strategy]
        xs = [j + (i - 1) * 0.23 for j in range(len(subset))]
        color, label = palette[i], labels[i]
        axes[0].bar(xs, [float(r["coverage_pct"]) for r in subset], width=0.21, color=color, label=label)
        for x, row in zip(xs, subset):
            time = row["first_crossing_s"]
            axes[1].plot(x, float(time or row["horizon"]), marker="o" if time else "^",
                         color=color, markerfacecolor=color if time else "none", markersize=6)
        axes[2].bar(xs, [int(r["robots_crossing"]) for r in subset], width=0.21, color=color)
    seeds = [row["environment_seed"] for row in rows if row["passage_strategy"] == STRATEGIES[0]]
    for axis in axes:
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds, rotation=30, ha="right")
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
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    for ext in ("png", "pdf"):
        with (output / f"gain_region_comparison.{ext}").open("xb") as stream:
            fig.savefig(stream, format=ext, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    manifest = json.loads((args.formal / "experiment_manifest.json").read_text())
    runs, geometries = schedule("formal")
    if (manifest["runs"] != runs or len(rows) != len(runs)
            or any(any(row[key] != str(value) for key, value in run.items()) for row, run in zip(rows, runs))):
        raise ValueError("Summary does not match the locked matched schedule")
    args.output.mkdir(parents=True, exist_ok=False)
    pairs = list(comparisons(rows))
    write_csv(args.output / "paired_layouts.csv", pairs)
    write_csv(args.output / "paired_summary.csv", list(matched_summary(pairs)))
    write_csv(args.output / "group_summary.csv", list(totals(rows)))
    inspections = list(inspection_results(args.formal, rows))
    if inspections:
        write_csv(args.output / "inspection_results.csv", inspections)
        diagnostics = []
        for strategy in STRATEGIES:
            subset = [item for item in inspections if item["strategy"] == strategy]
            runs = [row for row in rows if row["passage_strategy"] == strategy]
            diagnostics.append(dict(
                strategy=strategy, started=sum(int(row["inspections_started"]) for row in runs),
                finished=len(subset), anchor_or_original_scores=sum(item["anchor_or_original_score"] for item in subset),
                current_candidate_scores=sum(item["current_candidate_scored"] for item in subset),
                total_robot_seconds=sum(item["duration_s"] for item in subset),
                redirect_count=sum(item["reason"] == "redirected" for item in subset)))
        write_csv(args.output / "inspection_diagnostics.csv", diagnostics)
    redirect_rows = list(redirects(args.formal, rows, geometries))
    if redirect_rows:
        write_csv(args.output / "redirect_trajectories.csv", redirect_rows)
    segments = list(continuations(args.formal, rows))
    if segments:
        write_csv(args.output / "continuation_trace.csv", segments)
    plot(rows, args.output)


if __name__ == "__main__":
    main()
