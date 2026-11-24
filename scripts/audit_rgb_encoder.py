"""Evaluate frozen RGB checkpoints and clearance ranking on independent captures."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import torch

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import csv_rows, write_csv


SECTORS = ("left", "centre", "right")
PROFILES = ("clean", "dark", "blur")
MIN_DIRECTION_GAP = 0.10
PERMUTATION_SEED = 61917
TRAINING_CHECKPOINT_HASHES = {
    "legacy": "3caa21c9b79254bda5cc957c90f99c23d6e6a50566a72ebb321fa1a8c6071fab",
    "robust": "dde9e882d94d842879e136715ddb6ae2572550217cf1c3d3c272e9725f44c186",
}


def load_training_checkpoint(path, name):
    if digest(path) != TRAINING_CHECKPOINT_HASHES[name]:
        raise ValueError(f"Unexpected training checkpoint at {path}")
    return torch.load(path, map_location="cpu", weights_only=False)


def verified_rows(dataset, profile):
    groups = {}
    for run in profile["fit_runs"] + profile["held_out_runs"]:
        relative = f"{run}/manifest.csv"
        path = dataset / relative
        if digest(path) != profile["manifest_sha256"][relative]:
            raise ValueError(f"Capture manifest differs at {path}")
        rows = csv_rows(path)
        if not rows or {row["run_id"] for row in rows} != {run}:
            raise ValueError(f"Missing or mismatched capture rows at {path}")
        groups[run] = rows
    hashes = {
        split: {row["image_sha256"] for run in profile[split] for row in groups[run]}
        for split in ("fit_runs", "held_out_runs")
    }
    if hashes["fit_runs"] & hashes["held_out_runs"]:
        raise ValueError("Capture splits share image hashes")
    return groups


def score_rows(dataset, groups, profile, scorers, camera_input):
    rows = []
    for run in profile["held_out_runs"]:
        group = groups[run]
        permutation = np.random.default_rng(PERMUTATION_SEED).permutation(len(group))
        for index, row in enumerate(group):
            path = dataset / run / row["image_path"]
            if digest(path) != row["image_sha256"]:
                raise ValueError(f"Capture image differs at {path}")
            bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            result = {key: row[key] for key in (
                "layout", "run_id", "robot", "image_path", "image_sha256",
                "camera_stamp_ns", "scan_stamp_ns")}
            result["permuted_row_0_based"] = int(permutation[index])
            result.update({f"lidar_{sector}": float(row[sector]) for sector in SECTORS})
            stressed = {mode: camera_input(rgb, mode) for mode in PROFILES}
            stressed["flip"] = np.ascontiguousarray(rgb[:, ::-1])
            for name, scorer in scorers.items():
                for mode, image in stressed.items():
                    vector = scorer.score(image)
                    if mode == "flip":
                        vector = vector[::-1]
                    result.update({f"{name}_{mode}_{sector}": score
                                   for sector, score in zip(SECTORS, vector)})
            rows.append(result)
    return rows


def vector_rows(rows, prefix):
    return np.asarray([[row[f"{prefix}_{sector}"] for sector in SECTORS] for row in rows])


def measure(predictions, lidar):
    ordered = np.sort(lidar, axis=1)
    informative = ordered[:, 2] - ordered[:, 1] >= MIN_DIRECTION_GAP
    expected = lidar.argmax(axis=1)
    ranked = predictions.argmax(axis=1)
    by_sector = [(ranked[informative & (expected == index)] == index)
                 for index in range(len(SECTORS))]
    disagreements = predictions - lidar
    low_clearance = lidar <= 0.25
    return dict(
        frames=len(lidar), informative_frames=int(informative.sum()),
        mae=float(np.abs(disagreements).mean()),
        mean_sector_bias=float(disagreements.mean()),
        low_clearance_count=int(low_clearance.sum()),
        low_clearance_bias=float(disagreements[low_clearance].mean()) if low_clearance.any() else None,
        predicted_direction_spread=float(np.ptp(predictions, axis=1).mean()),
        lidar_direction_spread=float(np.ptp(lidar, axis=1).mean()),
        top_direction_agreement=float((ranked[informative] == expected[informative]).mean())
        if informative.any() else None,
        balanced_direction_agreement=float(np.mean([values.mean() for values in by_sector if len(values)]))
        if informative.any() else None,
        **{f"reference_{sector}_count": len(values) for sector, values in zip(SECTORS, by_sector)},
        **{f"agreement_{sector}": float(values.mean()) if len(values) else None
           for sector, values in zip(SECTORS, by_sector)})


def summarize(rows, prior, model_names):
    scopes = defaultdict(list)
    for row in rows:
        scopes["all", "all"].append(row)
        scopes[row["layout"], "all"].append(row)
        scopes[row["layout"], row["run_id"]].append(row)
    summary, fusion, flip = [], [], []
    by_run = {run: entries for (_, run), entries in scopes.items() if run != "all"}
    for (layout, run), entries in sorted(scopes.items()):
        lidar = vector_rows(entries, "lidar")
        summary.append(dict(layout=layout, run_id=run, method="fit_median",
                            **measure(np.broadcast_to(prior, lidar.shape), lidar)))
        for name in model_names:
            clean = vector_rows(entries, f"{name}_clean")
            mirrored = vector_rows(entries, f"{name}_flip")
            shuffled = [by_run[row["run_id"]][row["permuted_row_0_based"]] for row in entries]
            sources = {mode: vector_rows(entries, f"{name}_{mode}") for mode in PROFILES}
            sources["shuffled"] = vector_rows(shuffled, f"{name}_clean")
            sources["swap_scores"] = clean[:, ::-1]
            for mode, predictions in sources.items():
                summary.append(dict(layout=layout, run_id=run, method=f"{name}_{mode}",
                                    **measure(predictions, lidar)))
            flip.append(dict(layout=layout, run_id=run, model=name, frames=len(lidar),
                             flip_reversal_mae=float(np.abs(clean - mirrored).mean())))
            settings = {name + "_clean": clean}
            if name == "robust":
                settings["robust_swap_scores"] = clean[:, ::-1]
            for cue, visual in settings.items():
                for weight in (0.25, 0.50):
                    mixed = (1 - weight) * lidar + weight * visual
                    fusion.append(dict(layout=layout, run_id=run, cue=cue, rgb_weight=weight,
                                       **measure(mixed, lidar)))
    return summary, fusion, flip


def historical_audit(original, aligned, training_checkpoints):
    first, second = csv_rows(original), csv_rows(aligned)
    key_columns = ("image_path", "run_id", "robot", "stamp")
    same_rows = len(first) == len(second) and all(
        all(a[key] == b[key] for key in key_columns) for a, b in zip(first, second))
    if not same_rows:
        raise ValueError("Historical row identities differ between manifests")
    records = defaultdict(list)
    for row in second:
        records[row["image_path"]].append(row)
    collisions = {key: val for key, val in records.items() if len({row["run_id"] for row in val}) > 1}
    conflicts = sum(len({tuple(row[key] for key in SECTORS) for row in value}) > 1
                    for value in collisions.values())
    audit_by_model = {}
    for name, checkpoint in training_checkpoints.items():
        metadata = checkpoint["training_metadata"]
        manifest_name = Path(metadata["manifest"]).name
        chosen = {original.name: first, aligned.name: second}[manifest_name]
        validation = set(metadata["validation_run_ids"])
        training_paths = {row["image_path"] for row in chosen if row["run_id"] not in validation}
        validation_rows = [row for row in chosen if row["run_id"] in validation]
        if len(validation_rows) != metadata["validation_samples"]:
            raise ValueError(f"Original validation size does not match the manifest for {name}")
        audit_by_model[name] = dict(
            manifest=manifest_name, validation_run_ids=sorted(validation),
            training_rows=len(chosen) - len(validation_rows), validation_rows=len(validation_rows),
            validation_rows_sharing_training_path=sum(
                row["image_path"] in training_paths for row in validation_rows))
    switched = sum(tuple(a[key] for key in SECTORS[::-1]) == tuple(b[key] for key in SECTORS)
                   for a, b in zip(first, second))
    changed = sum(a["left"] != b["left"] or a["right"] != b["right"] for a, b in zip(first, second))
    return dict(rows=len(second), cross_run_paths=len(collisions), conflicting_cross_run_paths=conflicts,
                splits=audit_by_model,
                reversed_label_rows=switched, numerically_changed_side_labels=changed,
                source_manifest_sha256=digest(original), aligned_manifest_sha256=digest(aligned))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("quality_profile", type=Path)
    parser.add_argument("original_dataset", type=Path)
    parser.add_argument("original_checkpoint", type=Path)
    parser.add_argument("robust_checkpoint", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from camera_quality import camera_input
    from learned_clearance import LearnedVisionScorer

    torch.set_num_threads(1)
    profile = json.loads(args.quality_profile.read_text())
    groups = verified_rows(args.dataset, profile)
    checkpoint = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    original_weights = load_training_checkpoint(args.original_checkpoint, "legacy")
    robust_weights = load_training_checkpoint(args.robust_checkpoint, "robust")
    scorers = {name: LearnedVisionScorer(checkpoint) for name in ("legacy", "robust")}
    if tuple(robust_weights["input_size"]) != tuple(original_weights["input_size"]):
        raise ValueError("A different RGB input size requires its own preprocessing settings")
    scorers["robust"].model.load_state_dict(robust_weights["model_state_dict"])
    if any(not torch.equal(value, original_weights["model_state_dict"][key])
           for key, value in scorers["legacy"].model.state_dict().items()):
        raise ValueError("Published inference weights differ from the training checkpoint")
    prior = np.median([[float(row[sector]) for sector in SECTORS]
                       for run in profile["fit_runs"] for row in groups[run]], axis=0)
    rows = score_rows(args.dataset, groups, profile, scorers, camera_input)
    summary, fusion, symmetry = summarize(rows, prior, scorers)
    historical = historical_audit(args.original_dataset / "manifest.csv",
                                  args.original_dataset / "manifest_camera_aligned.csv",
                                  {"legacy": original_weights, "robust": robust_weights})
    args.output.mkdir(parents=True, exist_ok=True)
    preserve_json(args.output / "manifest.json", dict(
        source_tree=subprocess.check_output(
            ["git", "-C", str(args.ros_project), "write-tree"], text=True).strip(),
        models={name: dict(sha256=TRAINING_CHECKPOINT_HASHES[name], input_size=list(weights["input_size"]))
                for name, weights in (("legacy", original_weights), ("robust", robust_weights))},
        inference_checkpoint_sha256=digest(checkpoint), historical=historical,
        quality_profile_sha256=digest(args.quality_profile), captures=profile["manifest_sha256"],
        diagnostic_script_sha256=digest(Path(__file__)), fit_median=dict(zip(SECTORS, map(float, prior))),
        minimum_best_direction_gap=MIN_DIRECTION_GAP, permutation_seed=PERMUTATION_SEED))
    write_csv(args.output / "predictions.csv", rows)
    write_csv(args.output / "encoder_metrics.csv", summary)
    write_csv(args.output / "fusion_metrics.csv", fusion)
    write_csv(args.output / "flip_consistency.csv", symmetry)
    print(f"Audited {len(rows)} held-out frames", flush=True)


if __name__ == "__main__":
    main()
