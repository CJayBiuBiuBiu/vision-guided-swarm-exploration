"""Report layout-specific contrasts for complete four-condition seed blocks."""

import argparse
import json
import statistics
import subprocess
from collections import defaultdict
from pathlib import Path

from run_clearance_study import ROLE_CONDITIONS, digest, verify_run
from summarize_clearance_study import save_csv


METRICS = ("coverage_pct", "mean_coverage_pct", "trace_arrivals",
           "stuck_events", "near_collision_events", "path_m")
EFFECTS = ("actor_effect", "rule_effect", "interaction")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("summary_directory", type=Path)
    parser.add_argument("--verify-raw", type=Path, metavar="ROS_PROJECT")
    args = parser.parse_args()
    manifest = json.loads((args.study / "experiment_manifest.json").read_text())
    if args.verify_raw is not None:
        project = args.verify_raw.resolve()
        tree = subprocess.check_output(
            ["git", "-C", str(project), "rev-parse", "HEAD^{tree}"], text=True).strip()
        runner = Path(__file__).with_name("run_clearance_study.py")
        if tree != manifest["source_tree"] or digest(runner) != manifest["input_sha256"]["runner"]:
            raise ValueError("Raw verification requires the original source tree and runner")
        subprocess.run(["git", "-C", str(project), "diff", "--quiet", "HEAD", "--",
                        "turtlebot_ddpg", "laser_filters", "turtlebot3_description"], check=True)

    runs = []
    blocks = defaultdict(dict)
    for layout, seed, order, condition in manifest["schedule"]:
        actor, rules = ROLE_CONDITIONS[condition]
        name = f"{layout}_{seed}_{condition}"
        marker = json.loads((args.study / (name + ".complete.json")).read_text())
        attempt = marker["attempt"]
        if marker["order"] != order or Path(attempt).name != attempt or not attempt.startswith(name + "."):
            raise ValueError(f"Invalid completion marker for {name}")
        directory = args.study / attempt
        row = json.loads((directory / "result.json").read_text())
        if ((row["layout"], row["seed"], row["condition"], row["actor_cue"], row["rules_cue"])
                != (layout, seed, condition, actor, rules)
                or row["coverage_mask_sha256"] != marker["coverage_mask_sha256"]):
            raise ValueError(f"Run does not match its manifest in {directory}")
        if args.verify_raw is not None and row != verify_run(
                directory, layout, seed, condition, manifest["horizon_s"], project / "turtlebot_ddpg"):
            raise ValueError(f"Raw logs differ from the recorded result: {directory}")
        block = blocks[layout, seed]
        if (actor, rules) in block:
            raise ValueError(f"Duplicate condition in {layout}/{seed}")
        block[actor, rules] = row
        runs.append({**row, "order": order, **{
            key: json.dumps(row[key], sort_keys=True)
            for key in ("trace_outcome_counts", "robot_tick_pair_outcomes")
        }})
    if not blocks:
        raise ValueError("The manifest contains no runs")

    contrasts = []
    for (layout, seed), block in sorted(blocks.items()):
        if set(block) != set(ROLE_CONDITIONS.values()) or len({
                row["coverage_mask_sha256"] for row in block.values()}) != 1:
            raise ValueError(f"Incomplete or mismatched four-condition block for {layout}/{seed}")
        for metric in METRICS:
            ll = block["lidar", "lidar"][metric]
            fl = block["fusion", "lidar"][metric]
            lf = block["lidar", "fusion"][metric]
            ff = block["fusion", "fusion"][metric]
            contrasts.append({
                "layout": layout, "seed": seed, "metric": metric,
                "lidar_lidar": ll, "fusion_lidar": fl, "lidar_fusion": lf, "fusion_fusion": ff,
                "actor_effect": ((fl - ll) + (ff - lf)) / 2,
                "rule_effect": ((lf - ll) + (ff - fl)) / 2,
                "interaction": (ff - lf) - (fl - ll),
            })

    groups = defaultdict(list)
    for row in contrasts:
        for effect in EFFECTS:
            groups[row["layout"], row["metric"], effect].append(row[effect])
    layout_summary = [
        {"layout": layout, "metric": metric, "contrast": effect,
         "seed_blocks": len(values), "mean_difference": statistics.mean(values),
         "minimum": min(values), "maximum": max(values)}
        for (layout, metric, effect), values in sorted(groups.items())
    ]
    args.summary_directory.mkdir()
    save_csv(args.summary_directory / "role_runs.csv", runs)
    save_csv(args.summary_directory / "seed_contrasts.csv", contrasts)
    save_csv(args.summary_directory / "layout_role_summary.csv", layout_summary)
    print(f"Summarized {len(blocks)} four-condition blocks ({len(runs)} runs)")


if __name__ == "__main__":
    main()
