"""Isolate data and low-clearance loss effects with a fixed RGB architecture."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
from scipy.stats import rankdata
import torch
from torch.nn import functional as F

from audit_rgb_encoder import SECTORS, verified_rows
from run_clearance_study import digest, preserve_json
from run_matched_vision_training import csv_rows, write_csv


SEEDS = (66701, 66702, 66703)
STEPS = 320
BATCH = 24
LOW = 0.25
DATASETS = ("historical", "run_isolated")
OBJECTIVES = ("uniform", "low_clearance")
HISTORICAL_MANIFEST_SHA = "a24e7f4adc2675293e48a620a552aa45c5c2001880a772883c4cc2991e9a7e68"
ARCHIVED_ALIGNED_SHA = "4be795fe75a3f91d040bc1a71595cdd9b4885821b8381d89a74e6f71f02715b3"


def load_frames(groups, exclude=None):
    images, labels, rows = [], [], []
    for directory, records in groups:
        for row in records:
            path = directory / row["image_path"]
            fingerprint = digest(path)
            if "image_sha256" in row and fingerprint != row["image_sha256"]:
                raise ValueError(f"Capture bytes were changed at {path}")
            if exclude is not None and fingerprint in exclude:
                continue
            bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(cv2.resize(bgr, (160, 96), interpolation=cv2.INTER_AREA),
                               cv2.COLOR_BGR2RGB)
            images.append(np.transpose(rgb, (2, 0, 1)))
            labels.append([float(row[name]) for name in SECTORS])
            rows.append(dict(row, image_sha256=fingerprint))
    return torch.from_numpy(np.asarray(images)), torch.tensor(labels, dtype=torch.float32), rows


def sources(historical, isolated, profile):
    path = historical / "manifest_camera_aligned.csv"
    if digest(path) != HISTORICAL_MANIFEST_SHA:
        raise ValueError("Historical aligned labels have changed")
    history = [row for row in csv_rows(path) if row["run_id"] != "seed_3304"]
    groups = verified_rows(isolated, profile)
    fit = [(isolated / run, groups[run]) for run in profile["fit_runs"]]
    dev = [(isolated / run, groups[run]) for run in profile["held_out_runs"]]
    return {"historical": [(historical, history)], "run_isolated": fit}, dev


def predict(model, images):
    model.eval()
    with torch.inference_mode():
        return torch.cat([torch.sigmoid(model(batch.float() / 255))
                          for batch in images.split(64)]).numpy()


def metrics(p, y):
    low = y <= LOW
    delta = p - y
    highest = np.sort(y, axis=1)
    clear_rank = highest[:, -1] - highest[:, -2] >= 0.10
    actual_best, predicted_best = y.argmax(axis=1), p.argmax(axis=1)
    recalls = [(actual_best[mask] == predicted_best[mask]).mean()
               for sector in range(3) if (mask := clear_rank & (actual_best == sector)).any()]
    aucs = []
    for values, positives in zip(p.T, low.T):
        n, m = int(positives.sum()), int((~positives).sum())
        if n and m:
            aucs.append((rankdata(-values)[positives].sum() - n * (n + 1) / 2) / (n * m))
    return dict(frames=len(y), low_sectors=int(low.sum()), mae=float(np.abs(delta).mean()),
                low_mae=float(np.abs(delta[low]).mean()) if low.any() else None,
                other_mae=float(np.abs(delta[~low]).mean()) if (~low).any() else None,
                low_mean_signed_error=float(delta[low].mean()) if low.any() else None,
                low_detection_auroc=float(np.mean(aucs)) if aucs else None,
                ranked_frames=int(clear_rank.sum()),
                balanced_top_agreement=float(np.mean(recalls)) if recalls else None)


def objective(prediction, label, variant):
    regression = F.smooth_l1_loss(prediction, label, beta=0.15, reduction="none")
    delta = label.unsqueeze(2) - label.unsqueeze(1)
    relative = prediction.unsqueeze(2) - prediction.unsqueeze(1)
    clearly_ordered = delta >= 0.10
    ranking = (F.relu(0.10 - relative) * clearly_ordered).sum() / clearly_ordered.sum().clamp_min(1)
    if variant == "uniform":
        return regression.mean() + 0.15 * ranking
    low = label <= LOW
    weight = 1 + 3 * low
    overestimate = ((prediction - label).relu().square() * low).sum() / low.sum().clamp_min(1)
    return (weight * regression).sum() / weight.sum() + 0.15 * ranking + 0.8 * overestimate


def train(model_type, x, y, seed, variant):
    torch.manual_seed(seed)
    random = torch.Generator().manual_seed(seed + 11_000)
    model = model_type()
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.001, weight_decay=0.0001)
    history = []
    for step in range(1, STEPS + 1):
        selection = torch.randint(len(x), (BATCH,), generator=random)
        image, label = x[selection].float() / 255, y[selection]
        flip = torch.rand(BATCH, generator=random) < 0.5
        image[flip] = image[flip].flip((-1,))
        label[flip] = label[flip][:, (2, 1, 0)]
        gain = 0.80 + 0.40 * torch.rand((BATCH, 1, 1, 1), generator=random)
        image = (image * gain).clamp(0, 1)
        prediction = torch.sigmoid(model(image))
        loss = objective(prediction, label, variant)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if step == 1 or step % 20 == 0:
            history.append(dict(step=step, training_loss=float(loss.detach())))
    return model, history


def score_for_selection(record):
    return (0.4 * record["mae"] + 0.4 * record["low_mae"]
            + 0.2 * (1 - record["balanced_top_agreement"]))


def read_fresh(root):
    manifest = json.loads((root / "experiment_manifest.json").read_text())
    groups = []
    for case in manifest["cases"]:
        seed = case["seed"]
        seal = json.loads((root / f"test_{seed}.complete.json").read_text())
        directory = root / seal["directory"] / "capture" / f"test_{seed}"
        path = directory / "manifest.csv"
        if digest(path) != seal["manifest_sha256"]:
            raise ValueError(f"Fresh test manifest changed at {directory}")
        groups.append((directory, csv_rows(path)))
    return groups


def save_checkpoint(path, model):
    with path.open("xb") as stream:
        torch.save(dict(format_version=2, input_size=(96, 160), output_names=SECTORS,
                        model_state_dict=model.state_dict()), stream)


def fit(args, model_type, scorer_type):
    grouped, dev_source = sources(args.historical, args.isolated, args.profile_data)
    development = load_frames(dev_source)
    dev_hashes = {row["image_sha256"] for row in development[2]}
    fitting = {name: load_frames(grouped[name], dev_hashes) for name in DATASETS}
    xdev, ydev, _ = development
    fingerprint_rows = [
        dict(split=split, run_id=row["run_id"], image_path=row["image_path"],
             image_sha256=row["image_sha256"])
        for split, (_, _, group) in {**fitting, "development": development}.items()
        for row in group]
    write_csv(args.output / "input_frames.csv", fingerprint_rows)
    records = []
    for data_kind, (x, y, _) in fitting.items():
        for variant in OBJECTIVES:
            for seed in SEEDS:
                name = f"{data_kind}_{variant}_{seed}"
                marker = args.output / f"{name}.complete.json"
                if marker.exists():
                    result = json.loads(marker.read_text())
                    path = args.output / result["directory"] / "encoder.pt"
                    if digest(path) != result["checkpoint_sha256"]:
                        raise ValueError(f"Training checkpoint changed at {path}")
                    scorer = scorer_type(path)
                    actual = metrics(predict(scorer.model, xdev), ydev.numpy())
                    if any(not np.isclose(actual[key], result["development"][key])
                           for key in actual if actual[key] is not None):
                        raise ValueError(f"Development audit differs at {path}")
                else:
                    print(f"TRAIN {name}", flush=True)
                    out = Path(tempfile.mkdtemp(prefix=name + ".", dir=args.output))
                    model, losses = train(model_type, x, y, seed, variant)
                    save_checkpoint(out / "encoder.pt", model)
                    write_csv(out / "losses.csv", losses)
                    result = dict(directory=out.name, checkpoint_sha256=digest(out / "encoder.pt"),
                                  development=metrics(predict(model, xdev), ydev.numpy()))
                    preserve_json(marker, result)
                row = dict(dataset=data_kind, objective=variant, seed=seed,
                           checkpoint_sha256=result["checkpoint_sha256"],
                           **result["development"])
                records.append(row)
                print(f"DONE {name} MAE={row['mae']:.4f} low={row['low_mae']:.4f}", flush=True)
    write_csv(args.output / "development_metrics.csv", records)
    groups = defaultdict(list)
    for row in records:
        groups[row["dataset"], row["objective"]].append(score_for_selection(row))
    selected = min(groups, key=lambda key: (np.mean(groups[key]), key))
    seed = SEEDS[1]
    marker = json.loads((args.output / f"{selected[0]}_{selected[1]}_{seed}.complete.json").read_text())
    preserve_json(args.output / "selection.json", dict(
        dataset=selected[0], objective=selected[1], seed=seed,
        checkpoint=f"{marker['directory']}/encoder.pt", checkpoint_sha256=marker["checkpoint_sha256"],
        development_group_scores={f"{a}_{b}": float(np.mean(values))
                                  for (a, b), values in sorted(groups.items())}))
    return fitting, development, records


def evaluate(args, scorer_type, fitting, development, records):
    selection = json.loads((args.output / "selection.json").read_text())
    used = {row["image_sha256"] for _, _, rows in [*fitting.values(), development] for row in rows}
    fresh = read_fresh(args.fresh)
    received = sum(len(rows) for _, rows in fresh)
    xtest, ytest, rows = load_frames(fresh, used)
    raw_rows = [dict(run_id=row["run_id"], layout=row["layout"], image_path=row["image_path"],
                     image_sha256=row["image_sha256"],
                     **{f"lidar_{s}": float(row[s]) for s in SECTORS}) for row in rows]
    paths = {}
    for row in records:
        name = f"{row['dataset']}_{row['objective']}_{row['seed']}"
        result = json.loads((args.output / f"{name}.complete.json").read_text())
        paths[name] = args.output / result["directory"] / "encoder.pt"
    paths["legacy"] = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    if digest(args.aligned) != ARCHIVED_ALIGNED_SHA:
        raise ValueError("The archived direction-corrected baseline differs")
    paths["robust_aligned"] = args.aligned
    selected = f"{selection['dataset']}_{selection['objective']}_{selection['seed']}"
    result = []
    for name, path in paths.items():
        predictions = predict(scorer_type(path).model, xtest)
        for row, prediction in zip(raw_rows, predictions):
            row.update({f"{name}_{sector}": float(score) for sector, score in zip(SECTORS, prediction)})
        for scope in ("all", *sorted({row["layout"] for row in rows})):
            mask = np.asarray([scope == "all" or row["layout"] == scope for row in rows])
            result.append(dict(model=name, layout=scope, **metrics(predictions[mask], ytest.numpy()[mask])))
    write_csv(args.output / "fresh_test_metrics.csv", result)
    write_csv(args.output / "fresh_predictions.csv", raw_rows)
    by_name = {row["model"]: row for row in result if row["layout"] == "all"}
    old, chosen = by_name["robust_aligned"], by_name[selected]
    passed = (chosen["mae"] < old["mae"] and chosen["low_mae"] < old["low_mae"]
              and chosen["low_detection_auroc"] > old["low_detection_auroc"])
    preserve_json(args.output / "test_gate.json", dict(
        selected=selected, checkpoint_sha256=selection["checkpoint_sha256"], passed=passed,
        test_rows_received=received, excluded_by_exact_hash=received - len(rows),
        test_manifest_sha256=digest(args.fresh / "experiment_manifest.json"),
        selected_metrics=chosen, baseline_metrics=old))
    print(f"FRESH TEST {selected} pass={passed} low MAE {chosen['low_mae']:.4f} "
          f"(old {old['low_mae']:.4f})", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("historical", type=Path)
    parser.add_argument("isolated", type=Path)
    parser.add_argument("quality_profile", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fresh", type=Path)
    parser.add_argument("--aligned", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from learned_clearance import LearnedVisionScorer, TinyOpennessCNN

    if args.fresh and args.aligned is None:
        parser.error("--fresh requires --aligned")
    torch.set_num_threads(2)
    args.output.mkdir(parents=True, exist_ok=True)
    args.profile_data = json.loads(args.quality_profile.read_text())
    preserve_json(args.output / "training_manifest.json", dict(
        seed_list=list(SEEDS), steps=STEPS, batch=BATCH,
        script_sha256=digest(Path(__file__)), quality_profile_sha256=digest(args.quality_profile),
        historical_manifest_sha256=HISTORICAL_MANIFEST_SHA,
        development_runs=args.profile_data["held_out_runs"],
        clean_fit_runs=args.profile_data["fit_runs"],
        selection_weights={"mae": 0.4, "low_mae": 0.4, "balanced_direction_error": 0.2}))
    fitting, development, records = fit(args, TinyOpennessCNN, LearnedVisionScorer)
    if args.fresh:
        evaluate(args, LearnedVisionScorer, fitting, development, records)


if __name__ == "__main__":
    main()
