"""Summarize complete seed pairs from the directional-clearance study."""

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path


METRICS = ("coverage_pct", "mean_coverage_pct", "path_m", "near_collision_events",
           "stuck_events", "recovery_results", "recovery_successes", "team_cue_ready_fraction",
           "trace_arrivals")
THRESHOLDS = (90, 97)


def save_csv(path, rows):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("summary_directory", type=Path)
    args = parser.parse_args()
    schedule = json.loads((args.study / "experiment_manifest.json").read_text())["schedule"]
    conditions = {condition for _, _, _, condition in schedule}
    reference = "lidar" if "lidar" in conditions else "latest_fusion"
    treatment = "buffered_fusion"
    if conditions != {reference, treatment}:
        raise ValueError("Expected one reference and one buffered fusion condition")

    by_block = defaultdict(dict)
    for layout, seed, _, condition in schedule:
        marker = args.study / f"{layout}_{seed}_{condition}.complete.json"
        if not marker.is_file():
            continue
        directory = args.study / json.loads(marker.read_text())["attempt"]
        row = json.loads((directory / "result.json").read_text())
        if (row["layout"], row["seed"], row["condition"]) != (layout, seed, condition):
            raise ValueError(f"Run marker does not match the schedule: {marker}")
        by_block[layout, seed][condition] = row

    output_runs = []
    paired = []
    threshold_times = defaultdict(list)
    for (layout, seed), results in sorted(by_block.items()):
        if set(results) != conditions:
            continue
        control, intervention = (results[condition] for condition in (reference, treatment))
        if control["coverage_mask_sha256"] != intervention["coverage_mask_sha256"]:
            raise ValueError(f"Coverage mask changed in {layout} {seed}")
        for result in (control, intervention):
            row = dict(result)
            row["robot_tick_pair_outcomes"] = json.dumps(row["robot_tick_pair_outcomes"], sort_keys=True)
            output_runs.append(row)
        paired.append({
            "layout": layout, "seed": seed, "reference": reference, "treatment": treatment,
            **{f"{metric}_difference": intervention[metric] - control[metric] for metric in METRICS},
        })
        for threshold in THRESHOLDS:
            key = f"time_to_{threshold}_s"
            threshold_times[layout, threshold].append((control[key], intervention[key]))
    if not paired:
        raise ValueError("The study contains no completed pairs")

    summaries = []
    for layout in sorted({row["layout"] for row in paired}):
        pairs = [row for row in paired if row["layout"] == layout]
        for metric in METRICS:
            differences = [row[f"{metric}_difference"] for row in pairs]
            summaries.append({
                "layout": layout, "metric": metric, "pairs": len(pairs),
                "buffered_minus_reference_mean": statistics.mean(differences),
                "sample_sd": statistics.stdev(differences) if len(pairs) > 1 else None,
                "minimum": min(differences), "maximum": max(differences),
            })
    threshold_summary = []
    for (layout, threshold), times in sorted(threshold_times.items()):
        matched_differences = [buffered - baseline for baseline, buffered in times
                               if baseline is not None and buffered is not None]
        threshold_summary.append({
            "layout": layout, "coverage_threshold_pct": threshold, "pairs": len(times),
            "reference": reference, "treatment": treatment,
            "reference_reached": sum(baseline is not None for baseline, _ in times),
            "buffered_reached": sum(buffered is not None for _, buffered in times),
            "both_reached": len(matched_differences),
            "buffered_minus_reference_mean_s_when_both_reached":
                statistics.mean(matched_differences) if matched_differences else None,
        })

    args.summary_directory.mkdir()
    save_csv(args.summary_directory / "paired_runs.csv", output_runs)
    save_csv(args.summary_directory / "paired_differences.csv", paired)
    save_csv(args.summary_directory / "layout_summary.csv", summaries)
    save_csv(args.summary_directory / "coverage_thresholds.csv", threshold_summary)
    print(f"{len(paired)} completed pairs summarized in {args.summary_directory}")


if __name__ == "__main__":
    main()
