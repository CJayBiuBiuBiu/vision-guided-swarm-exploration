"""Continue the paired MAPPO lineages and test the prespecified longer budget."""

import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import CUES, LAYOUTS, csv_rows, run_name, schedule, trial, write_csv


REPOSITORY = Path(__file__).resolve().parents[1]
PILOT = REPOSITORY / "ros_gazebo/experiments/2026-11-23-matched-vision"
STUDY = REPOSITORY / "ros_gazebo/experiments/2026-11-23-vision-training-extension"
TRAINING_KEYS = ("training_seeds", "training_seconds", "rollout_steps",
                 "updates_per_episode", "decision_interval_s")


def stage_io(plan, stage):
    common = {key: plan[key] for key in TRAINING_KEYS}
    if stage == "test_32":
        # Generate the frozen endpoint schedule through the same paired-order code.
        common.update(training_episodes=1, episode_seed_base=80000001,
                      sampling_seed_base=90000001, evaluation_seconds=plan["test_seconds"],
                      evaluation_cases=plan["test_cases"])
        _, evaluation = schedule(common)
        return 32, 32, common, [], evaluation
    stages = {f"update_{item['target_updates']}": item for item in plan["training_stages"]}
    settings = stages[stage]
    start = settings["parent_updates"]
    target = settings["target_updates"]
    step = common["updates_per_episode"]
    if not 0 < start < target or start % step or target % step:
        raise ValueError("Continuation stages require positive, whole-episode update bounds")
    common.update(training_episodes=(target-start)//step,
                  episode_seed_base=settings["episode_seed_base"],
                  sampling_seed_base=settings["sampling_seed_base"],
                  evaluation_seconds=plan["validation_seconds"],
                  evaluation_cases=plan["validation_cases"])
    training, evaluation = schedule(common)
    for run in training:
        run["episode"] += start // step
    return start, target, common, training, evaluation


def validate_plan(plan, pilot):
    stages = [(entry["parent_updates"], entry["target_updates"])
              for entry in plan["training_stages"]]
    if stages != [(8, 16), (16, 32)]:
        raise ValueError("Expected the published continuation structure")
    if any(plan[key] != pilot["plan"][key] for key in TRAINING_KEYS):
        raise ValueError("Training controls differ from the pilot")
    validation = {(row["layout"], row["seed"]) for row in plan["validation_cases"]}
    prior = {(row["layout"], row["seed"]) for row in pilot["plan"]["evaluation_cases"]}
    if validation != prior or plan["validation_seconds"] != pilot["plan"]["evaluation_seconds"]:
        raise ValueError("Validation must match the pilot evaluation conditions")
    train_env = {row["environment_seed"] for row in pilot["train_schedule"]}
    sample = {row["episode_seed"] for row in pilot["train_schedule"]}
    train_segments = [stage_io(plan, f"update_{target}")[3] for _, target in stages]
    namespaces = [set(plan["training_seeds"]), {row["seed"] for row in plan["validation_cases"]},
                  {row["seed"] for row in plan["test_cases"]}, train_env, sample]
    for rows in train_segments:
        namespaces.extend([{row["environment_seed"] for row in rows},
                           {row["episode_seed"] for row in rows}])
    if len(set.union(*namespaces)) != sum(map(len, namespaces)):
        raise ValueError("Training, validation and final test seed ranges overlap")
    if (plan["test_seconds"] != 120
            or {row["layout"] for row in plan["test_cases"]} != set(LAYOUTS)
            or any(sum(case["layout"] == world for case in plan["test_cases"]) != 3
                   for world in LAYOUTS)):
        raise ValueError("Expected three 120-second final test cases per layout")


def weight_digest(payload):
    h = hashlib.sha256()
    for part in ("actor", "critic"):
        for key, tensor in sorted(payload[part].items()):
            h.update(key.encode("utf-8") + b"\x00")
            h.update(tensor.detach().cpu().numpy().tobytes())
    return h.hexdigest()


def endpoint_marker(root, seed, cue, episode):
    marker = root / f"{run_name(dict(stage='train', seed=seed, cue=cue, episode=episode))}.complete.json"
    entry = json.loads(marker.read_text())
    directory = root / entry["directory"]
    result = entry["result"]
    if (directory.parent != root or result["seed"] != seed or result["cue"] != cue
            or result["episode"] != episode):
        raise ValueError(f"Endpoint marker has inconsistent lineage: {marker}")
    path = directory / "endpoint.pt"
    if digest(path) != result["endpoint_sha256"]:
        raise ValueError(f"Endpoint checksum mismatch: {path}")
    return path, result


def load_endpoint(path, seed, updates, rollout_steps):
    import torch

    payload = torch.load(path, map_location="cpu", weights_only=False)
    expected = dict(training_seed=seed, updates=updates, rollout_length=rollout_steps,
                    omit_reference_coverage=True, algorithm="mappo")
    if any(payload["metadata"][key] != value for key, value in expected.items()):
        raise ValueError(f"Checkpoint metadata differs: {path}")
    if not all(payload[key]["state"] for key in ("actor_optimizer", "critic_optimizer")):
        raise ValueError(f"Missing optimizer state: {path}")
    return weight_digest(payload)


def record_parent(parent, stage, parent_updates, plan, tree, catalog, seed_inputs):
    manifest_path = parent / "experiment_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["source_tree"] != tree:
        raise ValueError("Parent used a different prepared ROS tree")
    if parent_updates == 8:
        if digest(manifest_path) != plan["pilot_manifest_sha256"]:
            raise ValueError("Parent must be the verified eight-update pilot")
        rows = csv_rows(catalog)
    else:
        parent_name = f"update_{parent_updates}"
        if (manifest["stage"] != parent_name or manifest["target_updates"] != parent_updates
                or any(manifest["input_sha256"][key] != value
                       for key, value in seed_inputs.items())):
            raise ValueError(f"{stage} requires the matching {parent_name} results")
        rows = csv_rows(parent / "lineage_endpoints.csv")
        # Require the planned validation to have finished on the preceding stage.
        evaluations = csv_rows(parent / "evaluation_runs.csv")
        keys = {(int(row["seed"]), row["cue"], row["layout"], int(row["environment_seed"]))
                for row in evaluations}
        expected = {(seed, cue, case["layout"], case["seed"]) for seed in plan["training_seeds"]
                    for cue in CUES for case in plan["validation_cases"]}
        if keys != expected or len(evaluations) != len(expected):
            raise ValueError("Parent validation is incomplete")
    records = {(int(row["training_seed"]), row["cue"]): row for row in rows}
    valid = {(seed, cue) for seed in plan["training_seeds"] for cue in CUES}
    if set(records) != valid or len(records) != len(rows):
        raise ValueError("Parent has missing or repeated lineages")
    endpoints = {}
    hashes = {}
    prior_weights = {}
    for (seed, cue), row in records.items():
        episode = parent_updates // plan["updates_per_episode"]
        if (int(row["training_episodes"]) != episode or int(row["updates"]) != parent_updates
                or int(row["transitions"]) != parent_updates * plan["rollout_steps"]):
            raise ValueError("Parent checkpoint received the wrong budget")
        path, trial_result = endpoint_marker(parent, seed, cue, episode)
        if trial_result["endpoint_sha256"] != row["endpoint_sha256"]:
            raise ValueError("Parent checkpoint differs from its published endpoint")
        endpoints[seed, cue] = path
        hashes[f"{seed}_{cue}"] = row["endpoint_sha256"]
        prior_weights[seed, cue] = load_endpoint(path, seed, parent_updates, plan["rollout_steps"])
    return manifest_path, endpoints, hashes, prior_weights


def run_stage(stage, project, parent, output, audit_only):
    package = project / "turtlebot_ddpg"
    if Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()).resolve() != package:
        raise ValueError("Source the intended ROS workspace before running")
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "HEAD", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    tree = subprocess.check_output(["git", "-C", str(project), "rev-parse", "HEAD^{tree}"],
                                   text=True).strip()
    plan_path = STUDY / "plan.json"
    plan = json.loads(plan_path.read_text())
    pilot_path = PILOT / "paired_experiment_manifest.json"
    if digest(pilot_path) != plan["pilot_manifest_sha256"]:
        raise ValueError("Published pilot manifest reference differs")
    pilot = json.loads(pilot_path.read_text())
    validate_plan(plan, pilot)
    start, target, phase_plan, training, evaluation = stage_io(plan, stage)
    rgb = REPOSITORY / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    catalog = PILOT / "lineage_endpoints.csv"
    files = dict(runner=Path(__file__), paired_runner=Path(__file__).with_name("run_matched_vision_training.py"),
                 shared_runner=Path(__file__).with_name("run_clearance_study.py"),
                 plan=plan_path, pilot_manifest=pilot_path, pilot_catalog=catalog, rgb=rgb,
                 ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5",
                 observation_adapter=package / "scripts/marl/observation_adapter.py",
                 **{world: package / "worlds" / (world + "_arena.world") for world in LAYOUTS})
    input_hashes = {key: digest(value) for key, value in files.items()}
    shared = ("rgb", "ddpg", "observation_adapter", *LAYOUTS)
    if tree != pilot["source_tree"] or any(input_hashes[key] != pilot["input_sha256"][key]
                                           for key in shared):
        raise ValueError("Prepared sources, models or maps differ from the pilot")
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        manifest, endpoints, hashes, prior_weights = record_parent(
            parent, stage, start, plan, tree, catalog, input_hashes)
        preserve_json(output / "experiment_manifest.json",
                      dict(stage=stage, source_tree=tree, parent_updates=start, target_updates=target,
                           input_sha256=input_hashes, parent_manifest_sha256=digest(manifest),
                           parent_checkpoints_sha256=hashes, train_schedule=training,
                           eval_schedule=evaluation))
        train_results = []
        for run in training:
            lineage = (run["seed"], run["cue"])
            if audit_only and not (output / f"{run_name(run)}.complete.json").is_file():
                raise ValueError(f"Missing accepted trial {run_name(run)}")
            result, directory = trial(run, phase_plan, package, rgb, output, endpoints[lineage])
            if result["input_weights_sha256"] != prior_weights[lineage]:
                raise ValueError(f"Observed input weights do not match the preceding checkpoint: {directory}")
            endpoint = directory / "endpoint.pt"
            prior_weights[lineage] = load_endpoint(endpoint, run["seed"], result["updates"],
                                                    plan["rollout_steps"])
            endpoints[lineage] = endpoint
            train_results.append(dict(**result, output_weights_sha256=prior_weights[lineage]))
        if training:
            write_csv(output / "training_runs.csv", train_results)
            write_csv(output / "lineage_endpoints.csv", [
                dict(training_seed=seed, cue=cue, training_episodes=target // plan["updates_per_episode"],
                     updates=target, transitions=target * plan["rollout_steps"],
                     endpoint_sha256=digest(endpoint))
                for (seed, cue), endpoint in sorted(endpoints.items())])
        eval_results = []
        masks = {}
        for run in evaluation:
            if audit_only and not (output / f"{run_name(run)}.complete.json").is_file():
                raise ValueError(f"Missing accepted trial {run_name(run)}")
            result, _ = trial(run, phase_plan, package, rgb, output, endpoints[run["seed"], run["cue"]])
            if masks.setdefault(run["layout"], result["coverage_mask_sha256"]) != result["coverage_mask_sha256"]:
                raise ValueError("Coverage mask changed within layout")
            eval_results.append(result)
        write_csv(output / "evaluation_runs.csv", eval_results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("update_16", "update_32", "test_32"))
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("parent", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    run_stage(args.stage, args.ros_project.resolve(), args.parent.resolve(), args.output.resolve(),
              args.audit_only)
