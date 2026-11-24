"""Summarize paired navigation outcomes and logged clearance distortions."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean

import numpy as np
from scipy.stats import rankdata

from run_matched_vision_training import csv_rows, write_csv
from run_rgb_encoder_retest import METHODS


COVERAGE_BASELINES = ("lidar", "legacy", "robust_raw")
NUMERIC_METRICS = ("coverage_pct", "path_m", "goals_reached", "near_collision_events",
                   "stuck_events", "recovery_successes", "traced_reached", "traced_wait")


def summarize_pairs(runs):
    groups = defaultdict(dict)
    for run in runs:
        key = (run["layout"], run["environment_seed"])
        if run["method"] in groups[key]:
            raise ValueError(f"Duplicate method in {key}")
        groups[key][run["method"]] = run
    pairs = []
    for (layout, seed), group in sorted(groups.items()):
        if set(group) != set(METHODS):
            raise ValueError(f"Incomplete pair block {layout} {seed}")
        corrected = group["robust_aligned"]
        for method in COVERAGE_BASELINES:
            baseline = group[method]
            result = dict(layout=layout, seed=seed, baseline=method,
                          aligned_coverage_pct=float(corrected["coverage_pct"]),
                          baseline_coverage_pct=float(baseline["coverage_pct"]))
            result.update({name + "_delta": round(float(corrected[name]) - float(baseline[name]), 4)
                           for name in NUMERIC_METRICS})
            for name in ("time_to_90_s", "time_to_97_s"):
                a, b = corrected[name], baseline[name]
                result["aligned_" + name] = float(a) if a else None
                result["baseline_" + name] = float(b) if b else None
                result[name + "_delta"] = round(float(a) - float(b), 3) if a and b else None
            pairs.append(result)
    return pairs


def trace_measures(path):
    lidar_rows, rule_rows, rgb_rows, picked = [], [], [], []
    for line in path.open():
        event = json.loads(line)
        if event["event"] != "assigned":
            continue
        lidar = event["actor_directional_clearance"]
        rgb = event["fusion_pair"]["paired_rgb_scores"]
        rules = event["directional_clearance"]
        if lidar is None or rgb is None:
            continue
        lidar_rows.append(lidar)
        rule_rows.append(rules)
        rgb_rows.append(rgb)
        picked.append((event["selected_actor_clearance_bonus"],
                       event["selected_clearance_bonus"]))
    if not lidar_rows:
        raise ValueError(f"No synchronized cues in {path}")
    lidar, rules, rgb = map(np.asarray, (lidar_rows, rule_rows, rgb_rows))
    chosen_lidar, chosen_rule = np.asarray(picked).T
    low = lidar <= 0.25
    selected_low = (0 < chosen_lidar) & (chosen_lidar <= 0.25)
    top = np.sort(lidar, axis=1)
    informative = top[:, -1] - top[:, -2] >= 0.10
    ranks_differ = rules.argmax(axis=1) != lidar.argmax(axis=1)
    return dict(
        paired_assignments=len(lidar), low_scan_sectors=int(low.sum()),
        mean_low_sector_rgb_minus_scan=float((rgb - lidar)[low].mean()) if low.any() else None,
        mean_low_sector_rule_minus_scan=float((rules - lidar)[low].mean()) if low.any() else None,
        informative_assignments=int(informative.sum()),
        rule_best_sector_differs=int((ranks_differ & informative).sum()),
        selected_positive_low_scan_cases=int(selected_low.sum()),
        selected_low_cases_rule_increase=float((chosen_rule - chosen_lidar)[selected_low].mean())
        if selected_low.any() else None)


def group_metrics(runs):
    groups = defaultdict(list)
    for run in runs:
        groups["all", run["method"]].append(run)
        groups[run["layout"], run["method"]].append(run)
    records = []
    for (layout, method), group in sorted(groups.items()):
        means = {name + "_mean": mean(float(row[name]) for row in group) for name in NUMERIC_METRICS}
        counts = {f"reached_{threshold}_count": sum(bool(row[f"time_to_{threshold}_s"]) for row in group)
                  for threshold in (90, 97)}
        records.append(dict(layout=layout, method=method, runs=len(group), **means, **counts))
    return records


def calibration_by_direction(rows):
    records = []
    layouts = sorted({row["layout"] for row in rows})
    for layout in ["all", *layouts]:
        group = [row for row in rows if layout in ("all", row["layout"])]
        for model in ("legacy", "robust_aligned"):
            for sector in ("left", "centre", "right"):
                aligned_sector = {"left": "right", "centre": "centre", "right": "left"}[sector]
                pred_column = (f"legacy_clean_{sector}" if model == "legacy"
                               else f"robust_clean_{aligned_sector}")
                scores = np.asarray([float(row[pred_column]) for row in group])
                reference = np.asarray([float(row[f"lidar_{sector}"]) for row in group])
                low = reference <= 0.25
                positives, negatives = int(low.sum()), int((~low).sum())
                if not positives or not negatives:
                    raise ValueError(f"Both calibration classes are required in {layout} {sector}")
                ranks = rankdata(-scores)
                auroc = (float(ranks[low].sum()) - positives * (positives + 1) / 2) / (
                    positives * negatives)
                records.append(dict(
                    layout=layout, model=model, sector=sector, frames=len(group),
                    lidar_low_cases=positives, mean_low_lidar=float(reference[low].mean()),
                    mean_rgb_on_lidar_low=float(scores[low].mean()),
                    mean_rgb_on_other=float(scores[~low].mean()),
                    rgb_score_at_most_025_in_low_cases=int((scores[low] <= 0.25).sum()),
                    low_clearance_auroc=auroc))
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--offline-predictions", type=Path)
    args = parser.parse_args()
    runs = csv_rows(args.study / "summary.csv")
    expected = json.loads((args.study / "experiment_manifest.json").read_text())["runs"]
    identities = tuple(expected[0])
    if ([tuple(str(run[key]) for key in identities) for run in expected] !=
            [tuple(row[key] for key in identities) for row in runs]):
        raise ValueError("The experiment summary does not match the frozen schedule")
    trace = []
    for run in runs:
        name = f"{run['layout']}_{run['environment_seed']}_{run['method']}"
        marker = json.loads((args.study / f"{name}.complete.json").read_text())
        directory = args.study / marker["directory"]
        trace.append(dict(layout=run["layout"], seed=run["environment_seed"], method=run["method"],
                          **trace_measures(directory / "decisions.jsonl")))
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "paired_differences.csv", summarize_pairs(runs))
    write_csv(args.output / "group_metrics.csv", group_metrics(runs))
    write_csv(args.output / "clearance_by_assignment.csv", trace)
    if args.offline_predictions:
        rows = csv_rows(args.offline_predictions)
        write_csv(args.output / "calibration_by_direction.csv", calibration_by_direction(rows))


if __name__ == "__main__":
    main()
