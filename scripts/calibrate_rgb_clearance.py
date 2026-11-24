"""Calibrate the clean-data uniform model before a second independent test."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.special import expit, logit
import torch

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import csv_rows, write_csv
from train_rgb_clearance_factorial import (
    ARCHIVED_ALIGNED_SHA, SECTORS, load_frames, metrics, predict, read_fresh,
    save_checkpoint, sources)


SCALES = (0.45, 0.60, 0.75, 0.90, 1.0, 1.15, 1.30)
BIASES = (-1.6, -1.35, -1.10, -0.85, -0.60, -0.35, -0.15, 0.0, 0.15, 0.3)
SOURCE_TRIAL = "run_isolated_uniform_66702"


def strictly_improves(candidate, reference):
    return (candidate["mae"] < reference["mae"] and candidate["low_mae"] < reference["low_mae"]
            and candidate["low_detection_auroc"] > reference["low_detection_auroc"])


def candidate_metrics(base, old, x, y):
    scores = np.clip(predict(base, x), 1e-6, 1 - 1e-6)
    logits = logit(scores)
    reference = metrics(predict(old, x), y)
    candidates = []
    for scale in SCALES:
        for bias in BIASES:
            measured = metrics(expit(scale * logits + bias), y)
            safe_open = measured["other_mae"] <= reference["other_mae"] + 0.05
            eligible = strictly_improves(measured, reference) and safe_open
            score = 0.55 * measured["mae"] + 0.45 * measured["low_mae"]
            candidates.append(dict(scale=scale, bias=bias, eligible=eligible, score=score, **measured))
    options = [row for row in candidates if row["eligible"]]
    if not options:
        raise ValueError("No development calibration qualified against the frozen baseline")
    selected = min(options, key=lambda row: (row["score"], abs(row["scale"] - 1), abs(row["bias"])))
    return scores, reference, selected, candidates


def fit(args, scorer_type, x, y):
    source = json.loads((args.factorial / f"{SOURCE_TRIAL}.complete.json").read_text())
    checkpoint = args.factorial / source["directory"] / "encoder.pt"
    if digest(checkpoint) != source["checkpoint_sha256"] or digest(args.aligned) != ARCHIVED_ALIGNED_SHA:
        raise ValueError("Frozen calibration inputs have changed")
    model = scorer_type(checkpoint).model
    reference = scorer_type(args.aligned).model
    scores, old, selected, candidates = candidate_metrics(model, reference, x, y)
    scale, bias = selected["scale"], selected["bias"]
    export = args.output / "encoder.pt"
    if not export.exists():
        with torch.no_grad():
            model.head.weight.mul_(scale)
            model.head.bias.mul_(scale).add_(bias)
        save_checkpoint(export, model)
    export_scores = predict(scorer_type(export).model, x)
    if not np.allclose(export_scores, expit(scale * logit(scores) + bias), atol=2e-5, rtol=0):
        raise ValueError("The exported calibration differs from the fitted sigmoid transform")
    choice = dict(checkpoint_sha256=digest(export), checkpoint=export.name,
                  source_trial=SOURCE_TRIAL, source_sha256=digest(checkpoint),
                  scale=scale, bias=bias, raw_development_metrics=metrics(scores, y),
                  old_development_metrics=old, calibrated_development_metrics=metrics(export_scores, y),
                  script_sha256=digest(Path(__file__)))
    preserve_json(args.output / "selection.json", choice)
    write_csv(args.output / "calibration_grid.csv", candidates)
    print(f"FROZEN calibration scale={scale} bias={bias} checkpoint={choice['checkpoint_sha256']}", flush=True)
    return choice, checkpoint, export


def evaluate(args, scorer_type, selection, checkpoint, export):
    prior = {row["image_sha256"] for row in csv_rows(args.factorial / "input_frames.csv")}
    previous_test = args.factorial / "fresh_predictions.csv"
    prior.update(row["image_sha256"] for row in csv_rows(previous_test))
    fresh = read_fresh(args.challenge)
    if {row["experiment_seed"] for _, group in fresh for row in group} != {"66314", "66315", "66316"}:
        raise ValueError("The second independent test set is required")
    xtest, ytest, rows = load_frames(fresh, prior)
    labels = ytest.numpy()
    scopes = ["all", *sorted({row["layout"] for row in rows})]
    methods = {"archived_aligned": args.aligned, "clean_uniform": checkpoint, "calibrated": export}
    records = []
    raw = [dict(run_id=row["run_id"], layout=row["layout"], image_path=row["image_path"],
                image_sha256=row["image_sha256"],
                **{f"lidar_{sector}": float(row[sector]) for sector in SECTORS}) for row in rows]
    for name, model_path in methods.items():
        predictions = predict(scorer_type(model_path).model, xtest)
        for row, scores in zip(raw, predictions):
            row.update({f"{name}_{sector}": float(value) for sector, value in zip(SECTORS, scores)})
        for scope in scopes:
            mask = np.asarray([scope == "all" or row["layout"] == scope for row in rows])
            records.append(dict(method=name, layout=scope, **metrics(predictions[mask], labels[mask])))
    write_csv(args.output / "challenge_metrics.csv", records)
    write_csv(args.output / "challenge_predictions.csv", raw)
    indexed = {row["method"]: row for row in records if row["layout"] == "all"}
    old, new = indexed["archived_aligned"], indexed["calibrated"]
    available = sum(len(entries) for _, entries in fresh)
    gate = dict(passed=strictly_improves(new, old), checkpoint_sha256=selection["checkpoint_sha256"],
                selected="calibrated", test_rows_received=available,
                excluded_by_exact_hash=available - len(rows),
                test_manifest_sha256=digest(args.challenge / "experiment_manifest.json"),
                selected_metrics=new, baseline_metrics=old)
    preserve_json(args.output / "test_gate.json", gate)
    print(f"CHALLENGE pass={gate['passed']} total MAE={new['mae']:.4f} "
          f"low MAE={new['low_mae']:.4f} (old {old['mae']:.4f} and {old['low_mae']:.4f})", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("historical", type=Path)
    parser.add_argument("isolated", type=Path)
    parser.add_argument("profile", type=Path)
    parser.add_argument("factorial", type=Path)
    parser.add_argument("aligned", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--challenge", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from learned_clearance import LearnedVisionScorer

    torch.set_num_threads(2)
    args.output.mkdir(parents=True, exist_ok=True)
    profile = json.loads(args.profile.read_text())
    _, group = sources(args.historical, args.isolated, profile)
    x, y, _ = load_frames(group)
    choice, base, export = fit(args, LearnedVisionScorer, x, y.numpy())
    if args.challenge is not None:
        evaluate(args, LearnedVisionScorer, choice, base, export)


if __name__ == "__main__":
    main()
