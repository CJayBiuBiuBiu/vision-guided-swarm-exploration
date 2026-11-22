"""Calibrate a camera-quality gate by run and evaluate it on held-out captures."""

import argparse
import csv
import json
import statistics
import subprocess
import sys
from collections import defaultdict
from itertools import product
from pathlib import Path

import cv2
import numpy as np


FIT_CLEAN_RETENTION = 0.95
FIT_PERCENTILES = range(1, 11)


def acceptance(features, luma, sharpness):
    return float(np.mean((features[:, 0] >= luma) & (features[:, 1] >= sharpness)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("dataset", type=Path, help="Directory containing run subdirectories")
    parser.add_argument("output", type=Path)
    parser.add_argument("--held-out-run", required=True, action="append")
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project.resolve() / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from camera_quality import CAMERA_PROFILES, camera_features, camera_input

    manifests = sorted(args.dataset.glob("*/manifest.csv"))
    if not manifests:
        raise ValueError("No capture manifests found")
    auditor = Path(__file__).with_name("audit_clearance_calibration.py")
    held_out_args = [item for name in args.held_out_run for item in ("--held-out-run", name)]
    audit = json.loads(subprocess.check_output(
        [sys.executable, str(auditor), *map(str, manifests), *held_out_args], text=True))
    features = defaultdict(list)
    held_out = set(args.held_out_run)
    for manifest in manifests:
        with manifest.open(newline="") as stream:
            for row in csv.DictReader(stream):
                bgr = cv2.imread(str(manifest.parent / row["image_path"]), cv2.IMREAD_COLOR)
                if bgr is None:
                    raise ValueError(f"Cannot decode {manifest.parent / row['image_path']}")
                image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                split = "held_out" if row["run_id"] in held_out else "fit"
                for profile in CAMERA_PROFILES:
                    features[split, row["layout"], row["run_id"], profile].append(
                        camera_features(camera_input(image, profile)))
    arrays = {key: np.asarray(values) for key, values in features.items()}
    clean_fit = [value for (split, _, _, profile), value in arrays.items()
                 if split == "fit" and profile == "clean"]
    stressed_fit = [value for (split, _, _, profile), value in arrays.items()
                    if split == "fit" and profile != "clean"]
    pooled = np.concatenate(clean_fit)
    candidates = [sorted({0.0, *map(float, np.percentile(pooled[:, column], list(FIT_PERCENTILES)))})
                  for column in (0, 1)]
    eligible = []
    for luma, sharpness in product(*candidates):
        retention = [acceptance(value, luma, sharpness) for value in clean_fit]
        if min(retention) < FIT_CLEAN_RETENTION:
            continue
        rejection = 1.0 - statistics.mean(acceptance(value, luma, sharpness) for value in stressed_fit)
        eligible.append((rejection, statistics.mean(retention), -luma, -sharpness))
    _, _, inverse_luma, inverse_sharpness = max(eligible)
    minimum_luma, minimum_sharpness = -inverse_luma, -inverse_sharpness

    performance = [
        {"split": split, "layout": layout, "run_id": run, "camera_profile": profile,
         "frames": len(values), "accepted_fraction": acceptance(values, minimum_luma, minimum_sharpness)}
        for (split, layout, run, profile), values in sorted(arrays.items())
    ]
    profile = {
        "format_version": 1,
        "thresholds": {"minimum_luma": minimum_luma, "minimum_sharpness": minimum_sharpness},
        "fit_clean_retention_floor": FIT_CLEAN_RETENTION,
        "fit_runs": sorted({run for split, _, run, _ in arrays if split == "fit"}),
        "held_out_runs": sorted(held_out),
        "manifest_sha256": {
            f"{Path(path).parent.name}/manifest.csv": digest
            for path, digest in audit["manifest_sha256"].items()},
    }
    args.output.mkdir()
    with (args.output / "camera_quality.json").open("x") as stream:
        json.dump(profile, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (args.output / "camera_gate_metrics.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(performance[0]))
        writer.writeheader()
        writer.writerows(performance)
    print(json.dumps(profile["thresholds"], sort_keys=True))
    for row in performance:
        if row["split"] == "held_out":
            print(row["layout"], row["run_id"], row["camera_profile"],
                  f"{row['accepted_fraction']:.1%} accepted")


if __name__ == "__main__":
    main()
