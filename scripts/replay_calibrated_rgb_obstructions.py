"""Replay the same controlled obstacle images with the calibrated RGB checkpoint."""

import argparse
import json
from pathlib import Path
import sys

import cv2
import torch

from run_clearance_study import digest
from run_matched_vision_training import csv_rows, write_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("obstruction_archive", type=Path)
    parser.add_argument("calibrated", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from learned_clearance import LearnedVisionScorer
    from directional_clearance import guarded_clearance

    manifest = json.loads((args.obstruction_archive / "experiment_manifest.json").read_text())
    gate = json.loads((args.calibrated / "test_gate.json").read_text())
    checkpoint = args.calibrated / "encoder.pt"
    if not gate["passed"] or digest(checkpoint) != gate["checkpoint_sha256"]:
        raise ValueError("Select the independent-test-verified calibrated checkpoint")
    torch.set_num_threads(1)
    scorer = LearnedVisionScorer(checkpoint)
    rows, groups = [], {}
    for row in csv_rows(args.obstruction_archive / "obstruction_scores.csv"):
        groups.setdefault(row["case"], []).append(row)
    for case, group in groups.items():
        if [row["runtime_sector"] for row in group] != ["left", "centre", "right"]:
            raise ValueError(f"Unexpected scan order in {case}")
        path = args.obstruction_archive / f"{case}.png"
        if digest(path) != manifest["cases"][case]["image_sha256"]:
            raise ValueError(f"Archived image checksum differs at {path}")
        rgb = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        new_scores = scorer.score(rgb)
        scan = [float(row["lidar"]) for row in group]
        guarded = guarded_clearance(scan, new_scores, 0.5)
        for record, first, second, third in zip(group, scan, new_scores, guarded):
            rows.append(dict(case=case, runtime_sector=record["runtime_sector"],
                             expected_blocked=record["expected_blocked"], lidar=first,
                             old_rgb=record["robust_aligned"], new_rgb=second,
                             new_fixed=(first + second) / 2, new_guarded=third))
    write_csv(args.output, rows)


if __name__ == "__main__":
    main()
