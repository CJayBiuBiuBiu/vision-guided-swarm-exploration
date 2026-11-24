"""Run matched closed-loop comparisons of frozen RGB direction order."""

import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import subprocess
import tempfile

from run_clearance_study import (
    digest, preserve_json, run_launch, startup_segfault, verify_matched_cues)
from run_matched_vision_training import csv_rows, trial_configs, verify_metrics, write_csv
from run_rgb_passage_study import ACTOR_SHA256, check_idle, logs_environment


METHODS = ("lidar", "legacy", "robust_raw", "robust_aligned")
BLOCKS = (
    ("hu_aligned", 66121, (0, 1, 3, 2)),
    ("hu_layout_c", 66122, (1, 2, 0, 3)),
    ("hu_aligned", 66123, (2, 3, 1, 0)),
    ("hu_layout_c", 66124, (3, 0, 2, 1)),
)
HORIZON = 90


def settings(run, package, actor, rgb, attempt):
    rules = "lidar" if run["method"] == "lidar" else "fusion"
    options = trial_configs(run, {"decision_interval_s": 1.5}, package, rgb, attempt, actor)
    override = [f"clearance_rules_cue:={rules}", "control_progress_mode:=observed_free_area",
                f"frontier_trace_path:={attempt / 'decisions.jsonl'}"]
    replaced = {option.split(":=", 1)[0] for option in override}
    return [option for option in options if option.split(":=", 1)[0] not in replaced] + override


def audit(run, directory, options, actor, rgb):
    rules = "lidar" if run["method"] == "lidar" else "fusion"
    expected = dict(run=run, options=options, actor_sha256=ACTOR_SHA256, rgb_sha256=digest(rgb))
    if json.loads((directory / "inputs.json").read_text()) != expected:
        raise ValueError(f"Trial provenance changed in {directory}")
    result = verify_metrics(directory, run, rules=rules)
    log = (directory / "launch.log").read_text(errors="replace")
    if (f"loaded mappo actor checkpoint {actor}" not in log
            or "reference coverage omitted from actor input" not in log):
        raise ValueError(f"Could not confirm the exact frozen actor in {directory}")
    metrics = csv_rows(directory / "metrics.csv")
    if {row["control_progress_mode"] for row in metrics} != {"observed_free_area"}:
        raise ValueError(f"Unexpected progress signal in {directory}")
    events = [json.loads(line) for line in (directory / "decisions.jsonl").read_text().splitlines()]
    header = events[0]
    assignments = [event for event in events if event["event"] == "assigned"]
    outcomes = [event for event in events if event["event"] == "outcome"]
    if ((header.get("clearance_actor_cue"), header.get("clearance_rules_cue")) != ("lidar", rules)
            or events[-1]["event"] != "run_end" or not assignments
            or Counter(event["attempt_id"] for event in assignments) != Counter(
                event["attempt_id"] for event in outcomes)):
        raise ValueError(f"Incomplete or mismatched decision trace in {directory}")
    for assignment in assignments:
        verify_matched_cues(assignment, "lidar", rules, 0.5, directory)
    return dict(**run, **result, rgb_sha256=expected["rgb_sha256"], assignments=len(assignments),
                traced_reached=sum(event["outcome"] == "reached" for event in outcomes),
                traced_wait=sum(event["outcome"] == "wait" for event in outcomes))


def trial(run, package, actor, rgb, output):
    name = f"{run['layout']}_{run['environment_seed']}_{run['method']}"
    marker = output / (name + ".complete.json")
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
    else:
        for _ in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            options = settings(run, package, actor, rgb, directory)
            preserve_json(directory / "inputs.json", dict(
                run=run, options=options, actor_sha256=ACTOR_SHA256, rgb_sha256=digest(rgb)))
            print(f"START {name} {directory}", flush=True)
            status = run_launch(
                ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                 "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *options],
                directory / "launch.log", logs_environment(directory))
            (directory / "exit_status.txt").write_text(f"{status}\n")
            if not startup_segfault(directory):
                break
    result = audit(run, directory, settings(run, package, actor, rgb, directory), actor, rgb)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"VERIFIED {name} coverage={result['coverage_pct']:.2f}% "
          f"assigned={result['assignments']} reached={result['traced_reached']}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("exports", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, actor, exports, output = (
        path.resolve() for path in (args.ros_project, args.actor, args.exports, args.output))
    package = project / "turtlebot_ddpg"
    found = Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip())
    if found.resolve() != package or digest(actor) != ACTOR_SHA256:
        raise ValueError("Supply the exact prepared environment and seed-62600 32-update LiDAR actor")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    repo = Path(__file__).resolve().parents[1]
    legacy = repo / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    models = dict(lidar=legacy, legacy=legacy,
                  robust_raw=exports / "robust_raw.pt", robust_aligned=exports / "robust_aligned.pt")
    export_manifest = exports / "checkpoint_manifest.json"
    expected = json.loads(export_manifest.read_text())
    checkpoints = {name: digest(path) for name, path in models.items()}
    if (expected["frames"] != 715 or max(expected["maximum_absolute_error"].values()) > 2e-6
            or expected["output_reorder"] != [2, 1, 0]
            or any(checkpoints[name] != sha for name, sha in expected["checkpoints_sha256"].items())
            or checkpoints["legacy"] != "3ddcf69b973bef4aebaaddddc15e537bc25cd82b8c024fe5a797f082bd6f8df5"):
        raise ValueError("The aligned RGB checkpoint must pass the 715-frame verification")
    runs = [dict(stage="eval", cue="lidar", method=METHODS[index], layout=layout,
                 environment_seed=seed, order=order, horizon=HORIZON)
            for layout, seed, conditions in BLOCKS
            for order, index in enumerate(conditions, 1)]
    paths = dict(actor=actor, ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5",
                 exports=export_manifest, runner=Path(__file__),
                 shared_runner=Path(__file__).with_name("run_matched_vision_training.py"),
                 cue_audit=Path(__file__).with_name("run_clearance_study.py"))
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json", dict(
            source_tree=tree, runs=runs, rgb_sha256=checkpoints,
            inputs_sha256={key: digest(path) for key, path in paths.items()},
            world_sha256={layout: digest(package / "worlds" / (layout + "_arena.world"))
                          for layout, _, _ in BLOCKS}))
        if args.audit_only and len(list(output.glob("*.complete.json"))) != len(runs):
            raise ValueError("The completed schedule is incomplete")
        results = [trial(run, package, actor, models[run["method"]], output) for run in runs]
        for layout, seed, _ in BLOCKS:
            if len({row["coverage_mask_sha256"] for row in results
                    if row["layout"] == layout and row["environment_seed"] == seed}) != 1:
                raise ValueError(f"Coverage mask differs within {layout} seed {seed}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
