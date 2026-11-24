"""Run matched Gazebo trials for the new encoder and low-scan score guard."""

import argparse
from collections import Counter
import fcntl
import json
from pathlib import Path
import subprocess
import tempfile

from run_clearance_study import (
    digest, preserve_json, run_launch, startup_segfault, verify_matched_cues)
from run_matched_vision_training import csv_rows, trial_configs, verify_metrics, write_csv
from run_rgb_passage_study import ACTOR_SHA256, check_idle, logs_environment
from train_rgb_clearance_factorial import ARCHIVED_ALIGNED_SHA


OPTIONS = {
    "lidar": "lidar",
    "old_fixed": "fusion",
    "new_fixed": "fusion",
    "new_guarded": "guarded_fusion",
}
BLOCKS = (
    ("hu_aligned", 66401, (0, 1, 3, 2)),
    ("hu_layout_c", 66402, (1, 2, 0, 3)),
    ("hu_aligned", 66403, (2, 3, 1, 0)),
    ("hu_layout_c", 66404, (3, 0, 2, 1)),
)
HORIZON = 75
EXPECTED_SOURCE_TREE = "9c5f0a04f9d865ebe3f369ecdafedb11aba00a62"


def options_for(run, package, actor, checkpoint, directory):
    options = trial_configs(run, {"decision_interval_s": 1.5}, package,
                            checkpoint, directory, actor)
    override = [f"clearance_rules_cue:={OPTIONS[run['method']]}",
                "control_progress_mode:=observed_free_area",
                f"frontier_trace_path:={directory / 'decisions.jsonl'}"]
    replaced = {option.split(":=", 1)[0] for option in override}
    return [option for option in options if option.split(":=", 1)[0] not in replaced] + override


def audit_assignment(assignment, rules, directory):
    scan = assignment["actor_directional_clearance"]
    applied = assignment["directional_clearance"]
    rgb = assignment["fusion_pair"]["paired_rgb_scores"]
    if rules == "guarded_fusion":
        check = dict(assignment, directional_clearance=scan)
        verify_matched_cues(check, "lidar", "lidar", 0.5, directory)
        expected = scan
        if scan is not None and rgb is not None:
            weights = [0.5 * min(max(4 * value - 1, 0), 1) for value in scan]
            expected = [(1 - w) * base + w * camera for base, camera, w in zip(scan, rgb, weights)]
        if applied != expected and (applied is None or expected is None or any(
                abs(a - b) > 1e-10 for a, b in zip(applied, expected))):
            raise ValueError(f"Recorded guard arithmetic differs in {directory}")
    else:
        verify_matched_cues(assignment, "lidar", rules, 0.5, directory)
    if scan is None:
        return []
    return [target - baseline for target, baseline in zip(applied, scan) if baseline <= 0.25]


def verify(run, directory, options, actor, checkpoint):
    rules = OPTIONS[run["method"]]
    expected = dict(run=run, launch=options, actor_sha256=ACTOR_SHA256,
                    rgb_sha256=digest(checkpoint))
    if json.loads((directory / "inputs.json").read_text()) != expected:
        raise ValueError(f"Unexpected run inputs in {directory}")
    result = verify_metrics(directory, run, rules=rules)
    if {row["control_progress_mode"] for row in csv_rows(directory / "metrics.csv")} != {"observed_free_area"}:
        raise ValueError(f"Unexpected run progress mode in {directory}")
    log = (directory / "launch.log").read_text(errors="replace")
    if f"loaded mappo actor checkpoint {actor}" not in log or "reference coverage omitted from actor input" not in log:
        raise ValueError(f"The actor or its feature schema differs in {directory}")
    events = [json.loads(line) for line in (directory / "decisions.jsonl").read_text().splitlines()]
    assignments = [event for event in events if event["event"] == "assigned"]
    outcomes = [event for event in events if event["event"] == "outcome"]
    if (not assignments or events[-1]["event"] != "run_end"
            or events[0]["clearance_actor_cue"] != "lidar" or events[0]["clearance_rules_cue"] != rules
            or Counter(item["attempt_id"] for item in assignments) != Counter(
                item["attempt_id"] for item in outcomes)):
        raise ValueError(f"Unexpected decision trace in {directory}")
    changed = [delta for item in assignments for delta in audit_assignment(item, rules, directory)]
    if rules == "guarded_fusion" and any(abs(value) > 1e-10 for value in changed):
        raise ValueError(f"The rule guard failed to preserve recorded low scores in {directory}")
    return dict(**run, **result, rgb_sha256=digest(checkpoint), assignment_count=len(assignments),
                low_lidar_sectors=len(changed), low_score_raised=sum(value > 1e-10 for value in changed),
                average_low_score_delta=sum(changed) / len(changed) if changed else None)


def trial(run, package, actor, checkpoint, output):
    name = f"{run['layout']}_{run['environment_seed']}_{run['method']}"
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
    else:
        for _ in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            options = options_for(run, package, actor, checkpoint, directory)
            preserve_json(directory / "inputs.json", dict(
                run=run, launch=options, actor_sha256=ACTOR_SHA256, rgb_sha256=digest(checkpoint)))
            print(f"START {name} {directory}", flush=True)
            status = run_launch(
                ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                 "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *options],
                directory / "launch.log", logs_environment(directory))
            (directory / "exit_status.txt").write_text(f"{status}\n")
            if not startup_segfault(directory):
                break
    options = options_for(run, package, actor, checkpoint, directory)
    result = verify(run, directory, options, actor, checkpoint)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"VERIFIED {name} coverage={result['coverage_pct']:.2f}% "
          f"low sectors raised={result['low_score_raised']}/{result['low_lidar_sectors']}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("old", type=Path)
    parser.add_argument("training", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    training, output, actor, old = (
        path.resolve() for path in (args.training, args.output, args.actor, args.old))
    package = args.ros_project.resolve() / "turtlebot_ddpg"
    found = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    if Path(found).resolve() != package or digest(actor) != ACTOR_SHA256 or digest(old) != ARCHIVED_ALIGNED_SHA:
        raise ValueError("The prepared package or the comparison weights differ")
    tree = subprocess.check_output(["git", "-C", str(package.parent), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(package.parent), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if tree != EXPECTED_SOURCE_TREE:
        raise ValueError("Select the verified guarded-fusion ROS source tree")
    gate = json.loads((training / "test_gate.json").read_text())
    selection = json.loads((training / "selection.json").read_text())
    chosen = training / selection["checkpoint"]
    if (not gate["passed"] or gate["checkpoint_sha256"] != selection["checkpoint_sha256"]
            or digest(chosen) != selection["checkpoint_sha256"]):
        raise ValueError("The candidate checkpoint must pass the independent offline gate")
    models = dict(lidar=chosen, old_fixed=old, new_fixed=chosen, new_guarded=chosen)
    methods = tuple(OPTIONS)
    runs = [dict(stage="eval", cue="lidar", layout=layout, environment_seed=seed,
                 method=methods[index], order=order, horizon=HORIZON)
            for layout, seed, indices in BLOCKS for order, index in enumerate(indices, 1)]
    files = {"runner": Path(__file__),
             "trial_configurations": Path(__file__).with_name("run_matched_vision_training.py"),
             "pair_verifier": Path(__file__).with_name("run_clearance_study.py"),
             "selection": training / "selection.json", "test_gate": training / "test_gate.json",
             "ddpg": package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5"}
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json", dict(
            source_tree=tree, runs=runs, actor_sha256=ACTOR_SHA256,
            input_sha256={label: digest(path) for label, path in files.items()},
            rgb_sha256={method: digest(path) for method, path in models.items()},
            world_sha256={layout: digest(package / "worlds" / (layout + "_arena.world"))
                          for layout, _, _ in BLOCKS}))
        if args.audit_only and len(list(output.glob("*.complete.json"))) != len(runs):
            raise ValueError("Missing completed trials in the formal schedule")
        results = [trial(run, package, actor, models[run["method"]], output) for run in runs]
        for layout, seed, _ in BLOCKS:
            if len({row["coverage_mask_sha256"] for row in results
                    if row["layout"] == layout and row["environment_seed"] == seed}) != 1:
                raise ValueError(f"The coverage grid differs across methods in {layout} seed {seed}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
