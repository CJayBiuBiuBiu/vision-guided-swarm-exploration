"""Export and validate locked-format raw and side-aligned robust RGB models."""

import argparse
from io import BytesIO
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

from audit_rgb_encoder import SECTORS, load_training_checkpoint
from run_clearance_study import digest, preserve_json
from run_matched_vision_training import csv_rows


def export(path, checkpoint):
    content = BytesIO()
    torch.save(checkpoint, content)
    if path.exists():
        previous = torch.load(path, map_location="cpu", weights_only=True)
        if (previous["output_names"] != checkpoint["output_names"]
                or previous["input_size"] != checkpoint["input_size"]
                or previous["format_version"] != checkpoint["format_version"]
                or previous["model_state_dict"].keys() != checkpoint["model_state_dict"].keys()
                or any(not torch.equal(previous["model_state_dict"][key], value)
                       for key, value in checkpoint["model_state_dict"].items())):
            raise ValueError(f"Existing export differs at {path}")
        return
    with path.open("xb") as stream:
        stream.write(content.getbuffer())


def verify_predictions(scorers, predictions, dataset):
    rows = csv_rows(predictions)
    if len(rows) != 715:
        raise ValueError("The archived replay must contain 715 frames")
    errors = {name: 0.0 for name in scorers}
    for row in rows:
        path = dataset / row["run_id"] / row["image_path"]
        if digest(path) != row["image_sha256"]:
            raise ValueError(f"Unexpected test frame at {path}")
        rgb = cv2.cvtColor(cv2.imread(str(path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        for name, scorer in scorers.items():
            scores = scorer.score(rgb)
            expected = [float(row[f"robust_clean_{sector}"]) for sector in SECTORS]
            if name == "robust_aligned":
                expected.reverse()
            errors[name] = max(errors[name], float(np.max(np.abs(np.asarray(scores) - expected))))
    if max(errors.values()) > 2e-6:
        raise ValueError(f"Exported values differ from the archived predictions: {errors}")
    return len(rows), errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("robust_checkpoint", type=Path)
    parser.add_argument("predictions", type=Path)
    parser.add_argument("held_out_dataset", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from learned_clearance import LearnedVisionScorer

    torch.set_num_threads(1)
    original = load_training_checkpoint(args.robust_checkpoint, "robust")
    if Path(original["training_metadata"]["manifest"]).name != "manifest.csv":
        raise ValueError("Checkpoint is expected to use the historical raw direction order")
    format_keys = ("format_version", "input_size", "output_names", "model_state_dict")
    raw = {key: original[key] for key in format_keys}
    aligned = dict(raw, model_state_dict=raw["model_state_dict"].copy())
    for key in ("head.weight", "head.bias"):
        aligned["model_state_dict"][key] = raw["model_state_dict"][key][[2, 1, 0]].clone()
    args.output.mkdir(parents=True, exist_ok=True)
    models = {"robust_raw": raw, "robust_aligned": aligned}
    paths = {name: args.output / f"{name}.pt" for name in models}
    for name, model in models.items():
        export(paths[name], model)
    scorers = {name: LearnedVisionScorer(path) for name, path in paths.items()}
    frames, errors = verify_predictions(scorers, args.predictions, args.held_out_dataset)
    preserve_json(args.output / "checkpoint_manifest.json", dict(
        original_checkpoint_sha256=digest(args.robust_checkpoint), output_reorder=[2, 1, 0],
        predictions_sha256=digest(args.predictions), script_sha256=digest(Path(__file__)),
        frames=frames, maximum_absolute_error=errors,
        checkpoints_sha256={name: digest(path) for name, path in paths.items()}))
    print(f"Validated exact side permutation on {frames} frames: {errors}", flush=True)


if __name__ == "__main__":
    main()
