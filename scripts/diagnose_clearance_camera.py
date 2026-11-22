"""Measure frozen RGB clearance error on the held-out camera-quality captures."""

import argparse
import csv
import hashlib
import json
import statistics
import sys
from pathlib import Path

import cv2


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("quality_profile", type=Path)
    parser.add_argument("rgb_checkpoint", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project.resolve() / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from camera_quality import CAMERA_PROFILES, CameraQualityGate, camera_features, camera_input
    from learned_clearance import LearnedVisionScorer

    profile = json.loads(args.quality_profile.read_text())
    gate = CameraQualityGate.load(args.quality_profile)
    scorer = LearnedVisionScorer(args.rgb_checkpoint)
    results = []
    for run in profile["held_out_runs"]:
        relative = Path(run) / "manifest.csv"
        manifest = args.dataset / relative
        if sha256(manifest) != profile["manifest_sha256"][relative.as_posix()]:
            raise ValueError(f"Unexpected held-out manifest: {manifest}")
        with manifest.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        layouts = {row["layout"] for row in rows}
        if len(layouts) != 1 or {row["run_id"] for row in rows} != {run}:
            raise ValueError(f"Inconsistent hold-out run in {manifest}")
        frames = []
        for row in rows:
            path = manifest.parent / row["image_path"]
            if sha256(path) != row["image_sha256"]:
                raise ValueError(f"Image checksum differs from the capture: {path}")
            image = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
            target = tuple(float(row[name]) for name in ("left", "centre", "right"))
            frames.append((image, target))
        for camera_profile in CAMERA_PROFILES:
            errors = {True: [], False: []}
            for image, target in frames:
                stressed = camera_input(image, camera_profile)
                accepted = gate.reason(*camera_features(stressed)) == "accepted"
                prediction = scorer.score(stressed)
                errors[accepted].append(statistics.mean(abs(a - b) for a, b in zip(prediction, target)))
            results.append({
                "layout": next(iter(layouts)), "run_id": run, "camera_profile": camera_profile,
                "frames": len(frames), "accepted_frames": len(errors[True]),
                "rgb_mae": statistics.mean(errors[True] + errors[False]),
                "accepted_mae": statistics.mean(errors[True]) if errors[True] else None,
                "rejected_mae": statistics.mean(errors[False]) if errors[False] else None,
            })
    if not results:
        raise ValueError("No held-out runs in the profile")
    args.output.mkdir()
    with (args.output / "camera_prediction_error.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    with (args.output / "diagnostic_manifest.json").open("x") as stream:
        json.dump({
            "profile_sha256": sha256(args.quality_profile),
            "rgb_checkpoint_sha256": sha256(args.rgb_checkpoint),
            "diagnostic_script_sha256": sha256(Path(__file__)),
            "held_out_runs": profile["held_out_runs"],
        }, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"Measured RGB prediction errors for {len(results)} run and camera combinations")


if __name__ == "__main__":
    main()
