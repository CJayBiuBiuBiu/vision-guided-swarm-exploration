"""Train two MAPPO visual-input variants from scratch with audited fixed budgets."""

import argparse
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault, threshold_time


CUES = ("lidar", "fusion")
TRAIN_LAYOUT = "hu_aligned"
LAYOUTS = (TRAIN_LAYOUT, "hu_layout_c")


def csv_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def schedule(plan):
    seeds = plan["training_seeds"]
    if not seeds or len(seeds) != len(set(seeds)):
        raise ValueError("Training seeds must be distinct")
    keys = ("training_episodes", "training_seconds", "episode_seed_base", "sampling_seed_base",
            "rollout_steps", "updates_per_episode", "evaluation_seconds")
    if any(type(plan[key]) is not int or plan[key] <= 0 for key in keys):
        raise ValueError("Budget fields and seed bases must be positive integers")
    interval = plan["decision_interval_s"]
    if not math.isfinite(interval) or interval <= 0:
        raise ValueError("Decision interval must be finite and positive")
    if interval * plan["rollout_steps"] * plan["updates_per_episode"] >= plan["training_seconds"]:
        raise ValueError("The requested rollouts do not fit the episode horizon")
    training = []
    evaluation = []
    training_environment_seeds = set()
    sampling_seeds = set()
    cases = plan["evaluation_cases"]
    cases_seeds = {case["seed"] for case in cases}
    if (not cases or len(cases) != len(cases_seeds) or
            any(case["layout"] not in LAYOUTS for case in cases)):
        raise ValueError("Use distinct evaluation seeds and supported layouts")
    for index, seed in enumerate(seeds):
        for episode in range(1, plan["training_episodes"] + 1):
            offset = index * plan["training_episodes"] + episode - 1
            environment_seed = plan["episode_seed_base"] + offset
            episode_seed = plan["sampling_seed_base"] + offset
            training_environment_seeds.add(environment_seed)
            sampling_seeds.add(episode_seed)
            cues = CUES if (index + episode) % 2 else CUES[::-1]
            training.extend(dict(stage="train", seed=seed, episode=episode, cue=cue,
                                 environment_seed=environment_seed, episode_seed=episode_seed,
                                 layout=TRAIN_LAYOUT, horizon=plan["training_seconds"]) for cue in cues)
        for position, case in enumerate(cases):
            cues = CUES if (index + position) % 2 == 0 else CUES[::-1]
            evaluation.extend(dict(stage="eval", seed=seed, cue=cue, environment_seed=case["seed"],
                                   layout=case["layout"], horizon=plan["evaluation_seconds"])
                              for cue in cues)
    namespaces = (set(seeds), cases_seeds, training_environment_seeds, sampling_seeds)
    if sum(map(len, namespaces)) != len(set.union(*namespaces)):
        raise ValueError("Environment, training, sampling and evaluation seeds must be disjoint")
    return training, evaluation


def run_name(run):
    label = f"episode_{run['episode']:03d}" if run["stage"] == "train" else str(run["environment_seed"])
    return f"{run['stage']}_{run['seed']}_{run['cue']}_{label}"


def verify_metrics(directory, run):
    if (directory / "exit_status.txt").read_text().strip() != "0":
        raise ValueError(f"ROS launch failed in {directory}")
    log = (directory / "launch.log").read_text(errors="replace")
    if re.search(r"Traceback|Segmentation fault|\[gazebo-[0-9]+\] process has died|"
                 r"REQUIRED process \[(robot[123]_explorer|mappo_train).*has died|"
                 r"observation failed", log):
        raise ValueError(f"Node failure in {directory}")
    rows = csv_rows(directory / "metrics.csv")
    expected = dict(selection_mode="mappo", mapping_mode="laser_scan", pose_source="gazebo_truth",
                    experiment_seed=str(run["environment_seed"]), world_profile=run["layout"],
                    vision_scoring_mode="matched_directional_clearance", clearance_actor_cue=run["cue"],
                    clearance_rules_cue="lidar", robot_count="3", fusion_pairing_mode="buffered")
    if not rows or any({row[key] for row in rows} != {value} for key, value in expected.items()):
        raise ValueError(f"Unexpected or missing runtime configuration in {directory}")
    last = rows[-1]
    times = [float(row["elapsed_sim_s"]) for row in rows]
    if (any(not math.isfinite(t) for t in times) or times != sorted(times) or
            not run["horizon"] <= times[-1] <= run["horizon"] + 1 or
            last["evaluation_horizon_reached"] != "1" or last["mission_done"] != "0" or
            last["scan_ready_robots"] != "3" or int(last["policy_actions_received"]) <= 0):
        raise ValueError(f"Episode horizon or control inputs are incomplete in {directory}")
    pairs = json.loads(last["fusion_pair_outcomes_json"])
    paired = pairs.get("latest", 0) + pairs.get("buffered", 0)
    if not 0 < paired <= sum(pairs.values()) == 3 * int(last["clearance_team_ticks"]):
        raise ValueError(f"Invalid sensor pairing counts in {directory}")
    mask = last["coverage_mask_sha256"]
    cells = int(last["coverage_explorable_cells"])
    if (not re.fullmatch(r"[0-9a-f]{64}", mask) or cells <= 0 or
            any(row["coverage_mask_sha256"] != mask or int(row["coverage_explorable_cells"]) != cells
                or not math.isclose(100 * int(row["coverage_explored_cells"]) / cells,
                                    float(row["coverage_pct"]), abs_tol=0.02) for row in rows)):
        raise ValueError(f"Coverage audit failed in {directory}")
    return dict(elapsed_s=times[-1], coverage_pct=float(last["coverage_pct"]),
                coverage_mask_sha256=mask, path_m=float(last["path_length_total_m"]),
                goals_reached=int(last["goals_reached"]), stuck_events=int(last["stuck_events"]),
                recovery_results=int(last["recovery_results"]),
                recovery_successes=int(last["recovery_successes"]),
                near_collision_events=int(last["near_collision_events"]),
                rgb_pair_availability=paired / sum(pairs.values()),
                time_to_90_s=threshold_time(rows, 90), time_to_97_s=threshold_time(rows, 97))


def verify_training(directory, run, plan):
    import torch

    count = plan["updates_per_episode"]
    before = (run["episode"] - 1) * count
    expected_updates = list(range(before + 1, before + count + 1))
    training = csv_rows(directory / "training.csv")
    diagnostics = csv_rows(directory / "diagnostics.csv")
    for rows in (training, diagnostics):
        if ([int(row["update"]) for row in rows] != expected_updates or
                any(not math.isfinite(float(value)) for row in rows for value in row.values())):
            raise ValueError(f"Update sequence or numerical check failed in {directory}")
    if any(int(row["transitions"]) != plan["rollout_steps"] for row in diagnostics):
        raise ValueError(f"Rollout length differs in {directory}")
    checkpoint = directory / "endpoint.pt"
    metadata = torch.load(checkpoint, map_location="cpu", weights_only=False)["metadata"]
    checks = dict(updates=expected_updates[-1], training_seed=run["seed"],
                  initial_updates=before, episode_seed=run["episode_seed"],
                  update_budget=count, rollout_length=plan["rollout_steps"],
                  omit_reference_coverage=True, algorithm="mappo")
    if any(metadata[key] != value for key, value in checks.items()):
        raise ValueError(f"Checkpoint metadata differs in {directory}")
    fingerprint = metadata["input_weights_sha256"]
    if not re.fullmatch(r"[0-9a-f]{64}", fingerprint):
        raise ValueError(f"Invalid initial weight fingerprint in {directory}")
    return dict(updates=expected_updates[-1], episode_updates=count,
                episode_transitions=count * plan["rollout_steps"],
                input_weights_sha256=fingerprint, endpoint_sha256=digest(checkpoint),
                mean_reward=sum(float(row["mean_reward"]) for row in training) / count)


def trial_configs(run, plan, package, rgb, directory, checkpoint):
    options = [f"world_profile:={run['layout']}",
               f"world_name:={package / 'worlds' / (run['layout'] + '_arena.world')}",
               "gui:=false", "robot1_x:=-0.8", "robot2_x:=0.0", "robot3_x:=0.8",
               "robot1_y:=4.0", "robot2_y:=4.0", "robot3_y:=4.0",
               "selection_mode:=mappo", "mapping_mode:=laser_scan", "pose_source:=gazebo_truth",
               "vision_enabled:=true", "vision_role:=both", "depth_corridor_mode:=off",
               "vision_scoring_mode:=matched_directional_clearance",
               f"clearance_actor_cue:={run['cue']}", "clearance_rules_cue:=lidar",
               "fusion_pairing_mode:=buffered", "clearance_rgb_weight:=0.5",
               f"learned_vision_checkpoint:={rgb}", "policy_omit_reference_coverage:=true",
               "fixed_horizon_only:=true", "complete_on_coverage:=false", "shutdown_on_done:=true",
               f"max_simulation_seconds:={run['horizon']}", "control_progress_mode:=reference_coverage",
               f"experiment_seed:={run['environment_seed']}", f"gazebo_seed:={run['environment_seed']}",
               f"metrics_path:={directory / 'metrics.csv'}"]
    if run["stage"] == "train":
        return options + ["start_mappo_training:=true", "start_network_policy:=false",
                          "training_algorithm:=mappo", "training_snapshot_interval:=0",
                          f"training_seed:={run['seed']}", f"training_episode_seed:={run['episode_seed']}",
                          f"training_rollout_length:={plan['rollout_steps']}",
                          f"training_decision_interval:={plan['decision_interval_s']}",
                          f"training_update_budget:={plan['updates_per_episode']}",
                          f"mappo_training_checkpoint:={directory / 'endpoint.pt'}",
                          f"mappo_training_log:={directory / 'training.csv'}",
                          f"mappo_diagnostics_log:={directory / 'diagnostics.csv'}"]
    return options + ["start_mappo_training:=false", "start_network_policy:=true",
                      "network_deterministic:=true", f"mappo_checkpoint:={checkpoint}",
                      f"network_decision_interval:={plan['decision_interval_s']}"]


def start_attempt(run, plan, package, rgb, output, checkpoint, input_hash):
    for process in ("gzserver", "gzclient", "rosmaster", "roslaunch"):
        if subprocess.run(["pgrep", "-x", process], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f"Another ROS or Gazebo process is running: {process}")
    directory = Path(tempfile.mkdtemp(prefix=run_name(run) + ".", dir=output))
    options = trial_configs(run, plan, package, rgb, directory, checkpoint)
    if run["stage"] == "train" and checkpoint is not None:
        shutil.copyfile(checkpoint, directory / "endpoint.pt")
        if digest(directory / "endpoint.pt") != input_hash:
            raise ValueError("Input checkpoint copy diverged")
    preserve_json(directory / "inputs.json", dict(run=run, checkpoint_input_sha256=input_hash,
                                                 launch_arguments=options))
    logs = directory / "ros_logs"
    logs.mkdir()
    print(f"START {run_name(run)} {directory}", flush=True)
    command = ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
               "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *options]
    status = run_launch(command, directory / "launch.log", {**os.environ, "ROS_LOG_DIR": str(logs)})
    (directory / "exit_status.txt").write_text(f"{status}\n")
    return directory


def verify_trial(directory, run, plan, package, rgb, checkpoint, input_hash):
    inputs = json.loads((directory / "inputs.json").read_text())
    expected_launch = trial_configs(run, plan, package, rgb, directory, checkpoint)
    if inputs != dict(run=run, checkpoint_input_sha256=input_hash, launch_arguments=expected_launch):
        raise ValueError(f"Trial inputs changed in {directory}")
    metrics = verify_metrics(directory, run)
    if run["stage"] == "train":
        result = verify_training(directory, run, plan)
    else:
        log = (directory / "launch.log").read_text(errors="replace")
        if ("reference coverage omitted from actor input" not in log or
                f"loaded mappo actor checkpoint {checkpoint}" not in log):
            raise ValueError(f"Evaluation actor or feature profile was not confirmed in {directory}")
        result = dict(checkpoint_sha256=input_hash)
    return dict(**run, **metrics, **result)


def trial(run, plan, package, rgb, output, checkpoint):
    marker = output / f"{run_name(run)}.complete.json"
    input_hash = digest(checkpoint) if checkpoint is not None else None
    cached = marker.exists()
    if cached:
        record = json.loads(marker.read_text())
        directory = output / record["directory"]
    else:
        directory = start_attempt(run, plan, package, rgb, output, checkpoint, input_hash)
        if startup_segfault(directory):
            directory = start_attempt(run, plan, package, rgb, output, checkpoint, input_hash)
    result = verify_trial(directory, run, plan, package, rgb, checkpoint, input_hash)
    preserve_json(marker, dict(directory=directory.name, result=result))
    prefix = "VERIFIED" if cached else "COMPLETE"
    print(f"{prefix} {run_name(run)} coverage={result['coverage_pct']:.2f}% "
          f"updates={result.get('updates', '-')} pairs={result['rgb_pair_availability']:.1%}",
          flush=True)
    return result, directory


def write_csv(path, rows):
    if path.exists():
        if csv_rows(path) != [{key: "" if value is None else str(value) for key, value in row.items()}
                               for row in rows]:
            raise ValueError(f"Archived summary differs: {path}")
        return
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("plan", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    project, output = args.ros_project.resolve(), args.output.resolve()
    package = project / "turtlebot_ddpg"
    if Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()).resolve() != package:
        raise ValueError("Source the intended ROS workspace before running")
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "HEAD", "--", "turtlebot_ddpg",
                    "turtlebot3_description", "laser_filters"], check=True)
    tree = subprocess.check_output(["git", "-C", str(project), "rev-parse", "HEAD^{tree}"],
                                   text=True).strip()
    plan = json.loads(args.plan.read_text())
    training, evaluation = schedule(plan)
    rgb = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    files = dict(runner=Path(__file__), shared_runner=Path(__file__).with_name("run_clearance_study.py"),
                 plan=args.plan, rgb=rgb,
                 ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5",
                 observation_adapter=package / "scripts/marl/observation_adapter.py",
                 **{layout: package / "worlds" / (layout + "_arena.world") for layout in LAYOUTS})
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json",
                      dict(plan=plan, source_tree=tree, train_schedule=training, eval_schedule=evaluation,
                           input_sha256={name: digest(path) for name, path in files.items()}))
        endpoints = {}
        paired_initial = {}
        train_results = []
        for run in training:
            lineage = (run["seed"], run["cue"])
            result, directory = trial(run, plan, package, rgb, output, endpoints.get(lineage))
            if run["episode"] == 1:
                initial = result["input_weights_sha256"]
                if paired_initial.setdefault(run["seed"], initial) != initial:
                    raise ValueError(f"Paired arms started from different weights: seed {run['seed']}")
            endpoints[lineage] = directory / "endpoint.pt"
            train_results.append(result)
        write_csv(output / "training_runs.csv", train_results)
        eval_results = []
        masks = {}
        for run in evaluation:
            checkpoint = endpoints[run["seed"], run["cue"]]
            result, _ = trial(run, plan, package, rgb, output, checkpoint)
            mask = result["coverage_mask_sha256"]
            if masks.setdefault(run["layout"], mask) != mask:
                raise ValueError(f"Coverage mask changed within {run['layout']}")
            eval_results.append(result)
        write_csv(output / "evaluation_runs.csv", eval_results)


if __name__ == "__main__":
    main()
