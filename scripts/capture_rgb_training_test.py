"""Collect three fresh camera and scan runs for encoder testing."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from run_clearance_study import digest, preserve_json, stop_launch, startup_segfault
from run_matched_vision_training import csv_rows, trial_configs
from run_rgb_passage_study import ACTOR_SHA256, check_idle, logs_environment


CASES = (("hu_aligned", 66311), ("hu_layout_c", 66312), ("hu_layout_b", 66313))
FRAMES = 150
HORIZON = 75


def audit(directory, layout, seed):
    manifest = directory / "capture" / f"test_{seed}" / "manifest.csv"
    rows = csv_rows(manifest)
    if len(rows) != FRAMES or {(r["layout"], r["experiment_seed"]) for r in rows} != {(layout, str(seed))}:
        raise ValueError(f"Incomplete capture at {directory}")
    if any(digest(manifest.parent / row["image_path"]) != row["image_sha256"] for row in rows):
        raise ValueError(f"Image checksum mismatch at {directory}")
    metrics = csv_rows(directory / "metrics.csv")
    last = metrics[-1]
    if ((directory / "exit_status.txt").read_text().strip() != "0"
            or last["evaluation_horizon_reached"] != "1" or last["scan_ready_robots"] != "3"
            or not HORIZON <= float(last["elapsed_sim_s"]) <= HORIZON + 1):
        raise ValueError(f"Missing capture horizon at {directory}")
    if re.search(r"Traceback|Segmentation fault|observation failed",
                 (directory / "launch.log").read_text(errors="replace")):
        raise ValueError(f"Capture launch failed at {directory}")
    return dict(directory=directory.name, manifest_sha256=digest(manifest))


def collect(directory, project, actor, rgb, layout, seed):
    run = dict(stage="eval", cue="lidar", layout=layout, environment_seed=seed, horizon=HORIZON)
    options = trial_configs(run, {"decision_interval_s": 1.5},
                            project / "turtlebot_ddpg", rgb, directory, actor)
    options = ["vision_scoring_mode:=lidar_openness" if item.startswith("vision_scoring_mode:=")
               else item for item in options]
    preserve_json(directory / "inputs.json", dict(run=run, launch=options))
    command = ["timeout", "--signal=INT", "--kill-after=25s", "240s",
               "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *options]
    environment = logs_environment(directory)
    with (directory / "launch.log").open("xb") as log:
        launch = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                  env=environment, start_new_session=True)
        try:
            collector = Path(__file__).with_name("collect_clearance_calibration.py")
            with (directory / "collector.log").open("xb") as output:
                capture = subprocess.Popen([
                    sys.executable, str(collector), str(project), str(directory / "capture"),
                    "--run-id", f"test_{seed}", "--layout", layout, "--seed", str(seed),
                    "--samples", str(FRAMES), "--interval", "1.2"],
                    env=environment, stdout=output, stderr=subprocess.STDOUT)
                try:
                    while True:
                        try:
                            if capture.wait(timeout=2):
                                raise ValueError(f"Camera collector failed at {directory}")
                            break
                        except subprocess.TimeoutExpired:
                            if launch.poll() is not None:
                                raise ValueError(f"Gazebo exited before camera collection at {directory}")
                    status = launch.wait(timeout=150)
                finally:
                    if capture.poll() is None:
                        capture.terminate()
                        capture.wait(timeout=5)
        finally:
            if launch.poll() is None:
                stop_launch(launch)
    (directory / "exit_status.txt").write_text(f"{status}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    project, actor, output = (path.resolve() for path in (args.ros_project, args.actor, args.output))
    package = project / "turtlebot_ddpg"
    found = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    if Path(found).resolve() != package or digest(actor) != ACTOR_SHA256:
        raise ValueError("Select the prepared ROS workspace and frozen LiDAR actor")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    output.mkdir(parents=True, exist_ok=True)
    rgb = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json", dict(
            cases=[dict(layout=layout, seed=seed) for layout, seed in CASES],
            source_tree=tree, frames_per_run=FRAMES, horizon=HORIZON, checkpoint_sha256=ACTOR_SHA256,
            ddpg_sha256=digest(package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5"),
            world_sha256={layout: digest(package / "worlds" / (layout + "_arena.world"))
                          for layout, _ in CASES},
            collector_sha256=digest(Path(__file__).with_name("collect_clearance_calibration.py")),
            script_sha256=digest(Path(__file__))))
        for layout, seed in CASES:
            marker = output / f"test_{seed}.complete.json"
            if marker.exists():
                directory = output / json.loads(marker.read_text())["directory"]
            else:
                for attempt in range(2):
                    check_idle()
                    directory = Path(tempfile.mkdtemp(prefix=f"test_{seed}.", dir=output))
                    print(f"CAPTURING {layout} {seed} {directory}", flush=True)
                    try:
                        collect(directory, project, actor, rgb, layout, seed)
                        break
                    except ValueError:
                        if attempt or not startup_segfault(directory):
                            raise
            preserve_json(marker, audit(directory, layout, seed))
            print(f"VERIFIED {layout} {seed} ({FRAMES} captures)", flush=True)


if __name__ == "__main__":
    main()
