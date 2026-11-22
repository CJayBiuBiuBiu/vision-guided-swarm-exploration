"""Audit and report all six conditions in each camera-quality block."""

import argparse
import json
import subprocess
from collections import defaultdict
from pathlib import Path

from run_clearance_study import QUALITY_CONDITIONS, digest, verify_run
from summarize_clearance_study import save_csv


METRICS = ("coverage_pct", "mean_coverage_pct", "trace_arrivals",
           "stuck_events", "near_collision_events", "path_m")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("study", type=Path)
    parser.add_argument("quality_profile", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.study / "experiment_manifest.json").read_text())
    source_tree = subprocess.check_output(
        ["git", "-C", str(args.ros_project), "rev-parse", "HEAD^{tree}"], text=True).strip()
    runner = Path(__file__).with_name("run_clearance_study.py")
    if (source_tree != manifest["source_tree"] or digest(runner) != manifest["input_sha256"]["runner"]
            or digest(args.quality_profile) != manifest["input_sha256"]["quality_profile"]):
        raise ValueError("The original source tree, runner and quality profile are required")
    subprocess.run(["git", "-C", str(args.ros_project), "diff", "--quiet", "HEAD", "--",
                    "turtlebot_ddpg", "laser_filters", "turtlebot3_description"], check=True)

    blocks = defaultdict(dict)
    results = []
    for layout, seed, order, condition in manifest["schedule"]:
        profile, cue = QUALITY_CONDITIONS[condition]
        name = f"{layout}_{seed}_{condition}"
        marker = json.loads((args.study / (name + ".complete.json")).read_text())
        attempt = marker["attempt"]
        if marker["order"] != order or Path(attempt).name != attempt or not attempt.startswith(name + "."):
            raise ValueError(f"Invalid completion marker for {name}")
        directory = args.study / attempt
        output = json.loads((directory / "result.json").read_text())
        audited = verify_run(directory, layout, seed, condition, manifest["horizon_s"],
                             args.ros_project / "turtlebot_ddpg", args.quality_profile)
        if output != audited or output["coverage_mask_sha256"] != marker["coverage_mask_sha256"]:
            raise ValueError(f"Raw logs differ from the recorded result for {name}")
        block = blocks[layout, seed]
        if (profile, cue) in block:
            raise ValueError(f"Duplicate condition in {layout}/{seed}")
        block[profile, cue] = output
        results.append({**output, "order": order, **{
            key: json.dumps(output[key], sort_keys=True) for key in
            ("trace_outcome_counts", "robot_tick_pair_outcomes", "camera_quality_outcomes")
        }})
    if not results:
        raise ValueError("The manifest contains no runs")

    differences = []
    for (layout, seed), block in sorted(blocks.items()):
        if set(block) != set(QUALITY_CONDITIONS.values()) or len({
                row["coverage_mask_sha256"] for row in block.values()}) != 1:
            raise ValueError(f"Incomplete or mismatched six-condition block for {layout}/{seed}")
        for profile in ("clean", "dark"):
            for metric in METRICS:
                lidar = block[profile, "lidar"][metric]
                fixed = block[profile, "fusion"][metric]
                adaptive = block[profile, "adaptive"][metric]
                differences.append({
                    "layout": layout, "seed": seed, "camera_profile": profile, "metric": metric,
                    "lidar": lidar, "fixed_fusion": fixed, "adaptive_fusion": adaptive,
                    "adaptive_minus_lidar": adaptive - lidar, "adaptive_minus_fixed": adaptive - fixed,
                })
    args.output.mkdir()
    save_csv(args.output / "quality_runs.csv", results)
    save_csv(args.output / "paired_camera_differences.csv", differences)
    print(f"Audited {len(results)} runs in {len(blocks)} seed blocks")


if __name__ == "__main__":
    main()
