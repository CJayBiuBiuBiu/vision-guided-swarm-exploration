"""Run and audit a matched Gazebo pilot for low-obstacle depth guidance."""

import argparse
from collections import Counter
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from xml.etree import ElementTree as ET

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault, verify_run


BASELINE = {"lidar": "matched_lidar_lidar", "fusion": "matched_fusion_fusion",
            "depth": "matched_fusion_fusion"}
SCENES = ("low_blocks", "clear")
WORLD_PROFILE = "hu_aligned"
BLOCK_CENTRES = [(-0.8, 2.95), (0.0, 2.78), (0.8, 2.95)]
BLOCK_SIZE = (0.29, 0.38, 0.09)


def read_schedule(path):
    with path.open(newline="") as stream:
        rows = [(row["scene"], int(row["seed"]), int(row["order"]), row["condition"])
                for row in csv.DictReader(stream)]
    if not rows:
        raise ValueError("The schedule is empty")
    blocks = {}
    for scene, seed, order, condition in rows:
        if scene not in SCENES or seed < 0 or condition not in BASELINE:
            raise ValueError(f"Invalid schedule row {(scene, seed, order, condition)}")
        blocks.setdefault((scene, seed), []).append((order, condition))
    if any([order for order, _ in rows] != [1, 2, 3] or
           {condition for _, condition in rows} != set(BASELINE) for rows in blocks.values()):
        raise ValueError("Every scene and seed must run all three conditions in logged order")
    return rows


def trial_world(base, scene):
    if scene == "clear":
        return base.read_bytes()
    tree = ET.parse(base)
    world = tree.find("world")
    size = " ".join(str(value) for value in BLOCK_SIZE)
    for index, (x, y) in enumerate(BLOCK_CENTRES, 1):
        model = ET.fromstring(f"""
          <model name="low_block_{index}">
            <static>true</static>
            <pose>{x} {y} {BLOCK_SIZE[2] / 2} 0 0 0</pose>
            <link name="body">
              <collision name="block"><geometry><box><size>{size}</size></box></geometry></collision>
              <visual name="block">
                <geometry><box><size>{size}</size></box></geometry>
                <material><ambient>0.35 0.16 0.08 1</ambient><diffuse>0.62 0.38 0.22 1</diffuse></material>
              </visual>
              <sensor name="block_contacts" type="contact">
                <always_on>true</always_on>
                <update_rate>30</update_rate>
                <contact><collision>block</collision></contact>
                <plugin name="block_bumper" filename="libgazebo_ros_bumper.so">
                  <robotNamespace>/depth_corridor</robotNamespace>
                  <bumperTopicName>contacts</bumperTopicName>
                  <frameName>world</frameName>
                </plugin>
              </sensor>
            </link>
          </model>""")
        world.append(model)
    return ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True)


def materialize_worlds(base, output, scenes):
    worlds = {}
    for scene in sorted(scenes):
        path = output / f"{scene}.world"
        content = trial_world(base, scene)
        try:
            with path.open("xb") as stream:
                stream.write(content)
        except FileExistsError:
            if path.read_bytes() != content:
                raise ValueError(f"Scene changed in {path}")
        worlds[scene] = path
    return worlds


def near_test_block(hit):
    if hit is None:
        return False
    x, y = hit["xy"]
    # World coordinates enter only the offline audit. Allow 7 cm for timing and projection.
    return any(abs(x - bx) <= BLOCK_SIZE[0] / 2 + 0.07 and
               abs(y - by) <= BLOCK_SIZE[1] / 2 + 0.07 for bx, by in BLOCK_CENTRES)


def count_block_contacts(events):
    stamps = {}
    for event in events:
        if event["event"] != "depth_contact":
            continue
        for pair in event["pairs"]:
            block = next((item.split("::")[0] for item in pair if item.startswith("low_block_")), None)
            robot = next((item.split("::")[0] for item in pair
                          if item.startswith(("robot1::", "robot2::", "robot3::"))), None)
            if block is not None and robot is not None:
                stamps.setdefault((block, robot), set()).add(event["contact_stamp"])
    return sum(1 + sum(right - left > 0.3 for left, right in zip(times, times[1:]))
               for values in stamps.values() for times in [sorted(values)])


def verify_pilot(directory, run, args, package):
    scene, seed, order, condition = run
    result = verify_run(directory, WORLD_PROFILE, seed, BASELINE[condition], args.horizon, package)
    events = [json.loads(line) for line in (directory / "decisions.jsonl").read_text().splitlines()]
    mode = "avoid" if condition == "depth" else "observe"
    if [event["mode"] for event in events if event["event"] == "depth_corridor"] != [mode]:
        raise ValueError(f"Unexpected depth mode in {directory}")
    decisions = [event for event in events if event["event"] == "depth_decision"]
    counts = Counter()
    assignments = {}
    outcomes = {event["attempt_id"]: event["outcome"] for event in events
                if event["event"] == "outcome"}
    selection_events = [event for event in events if event["event"] in ("assigned", "depth_decision")]
    for index, event in enumerate(selection_events):
        if event["event"] != "depth_decision":
            continue
        options = event["candidates"]
        first = options[0]["frontier_id"]
        target = next((item["frontier_id"] for item in options if item["depth_hit"] is None), None)
        expected = target if mode == "avoid" else first
        if target != event["would_select"] or expected != event["selected"]:
            raise ValueError(f"Depth decisions fail reconstruction in {directory}")
        for stamp in (event["depth_stamp"], event["laser_stamp"]):
            if stamp is not None and not 0 <= event["control_stamp"] - stamp <= 0.4000001:
                raise ValueError(f"Stale sensor evidence in {directory}")
        expected_pair = (event["depth_stamp"] is not None and event["laser_stamp"] is not None
                         and abs(event["depth_stamp"] - event["laser_stamp"]) <= 0.22)
        if event["paired"] != expected_pair:
            raise ValueError(f"Sensor pairing differs from the logged timestamps in {directory}")
        counts["fresh_depth"] += event["depth_stamp"] is not None
        counts["paired"] += event["paired"]
        if event["paired"] and options[0]["laser_hit"] is None and options[0]["depth_hit"] is not None:
            counts["depth_only"] += 1
            counts["depth_only_at_blocks"] += scene == "low_blocks" and near_test_block(
                options[0]["depth_hit"])
        counts["would_change"] += target != first
        counts["would_retarget"] += target is not None and target != first
        counts["changed"] += expected != first
        counts["withheld"] += expected is None
        if expected is not None:
            assignment = selection_events[index + 1] if index + 1 < len(selection_events) else None
            if (assignment is None or assignment["event"] != "assigned" or
                    (assignment["selected_frontier_id"], assignment["robot"]) != (expected, event["robot"])):
                raise ValueError(f"Applied depth choice differs from assignment in {directory}")
            assignments[assignment["attempt_id"]] = expected != first
    if (not decisions or counts["paired"] <= 0 or len(assignments) != result["trace_assignments"]):
        raise ValueError(f"Missing depth decision or assignment evidence in {directory}")
    clearance = {key: result[key] for key in ("coverage_pct", "mean_coverage_pct", "path_m",
                                               "stuck_events", "trace_arrivals", "trace_assignments",
                                               "time_to_90_s", "time_to_97_s", "simulation_seconds",
                                               "near_collision_events", "coverage_mask_sha256")}
    return dict(scene=scene, seed=seed, order=order, condition=condition, **clearance,
                decisions=len(decisions), fresh_depth=counts["fresh_depth"], paired=counts["paired"],
                depth_only=counts["depth_only"], depth_only_at_blocks=counts["depth_only_at_blocks"],
                would_change=counts["would_change"], would_retarget=counts["would_retarget"],
                changed=counts["changed"], withheld=counts["withheld"],
                retargeted_arrivals=sum(changed and outcomes[attempt] == "reached"
                                        for attempt, changed in assignments.items()),
                block_contacts=count_block_contacts(events))


def launch(run, args, worlds, mappo, rgb, name):
    scene, seed, _, condition = run
    for process in ("gzserver", "gzclient", "rosmaster", "roslaunch"):
        if subprocess.run(["pgrep", "-x", process], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f"Another ROS or Gazebo process is running: {process}")
    directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=args.output))
    log_dir = directory / "ros_logs"
    log_dir.mkdir()
    cue = "lidar" if condition == "lidar" else "fusion"
    depth = "avoid" if condition == "depth" else "observe"
    settings = [
        "gui:=false", f"world_profile:={WORLD_PROFILE}", f"world_name:={worlds[scene]}",
        "robot1_x:=-0.8", "robot2_x:=0.0", "robot3_x:=0.8",
        "robot1_y:=4.0", "robot2_y:=4.0", "robot3_y:=4.0",
        "selection_mode:=mappo_hybrid", "mapping_mode:=laser_scan", "pose_source:=gazebo_truth",
        "control_progress_mode:=observed_free_area", "vision_enabled:=true", "vision_role:=both",
        "vision_scoring_mode:=matched_directional_clearance", "vision_projection_mode:=thirds",
        f"clearance_actor_cue:={cue}", f"clearance_rules_cue:={cue}", "clearance_rgb_weight:=0.5",
        "fusion_pairing_mode:=buffered", f"learned_vision_checkpoint:={rgb}",
        "start_network_policy:=true", f"mappo_checkpoint:={mappo}", "network_deterministic:=true",
        "start_mappo_training:=false", f"depth_corridor_mode:={depth}",
        f"experiment_seed:={seed}", f"gazebo_seed:={seed}",
        f"max_simulation_seconds:={args.horizon}", "fixed_horizon_only:=true",
        "shutdown_on_done:=true", "complete_on_coverage:=false", "tail_completion_enabled:=false",
        f"metrics_path:={directory / 'metrics.csv'}", f"frontier_trace_path:={directory / 'decisions.jsonl'}",
    ]
    (directory / "launch_arguments.txt").write_text("\n".join(settings) + "\n")
    print(f"START {name} {directory}", flush=True)
    command = ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * args.horizon}s",
               "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *settings]
    status = run_launch(command, directory / "launch.log", {**os.environ, "ROS_LOG_DIR": str(log_dir)})
    (directory / "exit_status.txt").write_text(f"{status}\n")
    return directory


def run_one(run, args, package, worlds, mappo, rgb):
    scene, seed, _, condition = run
    name = f"{scene}_{seed}_{condition}"
    marker = args.output / f"{name}.complete.json"
    if marker.exists():
        prior = json.loads(marker.read_text())
        directory = args.output / prior["directory"]
        expected = verify_pilot(directory, run, args, package)
        if prior != dict(directory=directory.name, result=expected):
            raise ValueError(f"Archived result diverged for {name}")
        print(f"VERIFIED {name}", flush=True)
        return expected
    directory = launch(run, args, worlds, mappo, rgb, name)
    if startup_segfault(directory):
        directory = launch(run, args, worlds, mappo, rgb, name)
    result = verify_pilot(directory, run, args, package)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"COMPLETE {name} coverage={result['coverage_pct']:.2f}% "
          f"paired={result['paired']} depth-only={result['depth_only']} "
          f"changed={result['changed']} contacts={result['block_contacts']}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("schedule", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--horizon", type=int, default=75)
    args = parser.parse_args()
    if args.horizon <= 0:
        parser.error("horizon must be positive")
    args.output = args.output.resolve()
    project = args.ros_project.resolve()
    package = project / "turtlebot_ddpg"
    found = Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip())
    if found.resolve() != package:
        raise ValueError(f"The active ROS package is {found}")
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "HEAD", "--", "turtlebot_ddpg",
                    "turtlebot3_description", "laser_filters"], check=True)
    tree = subprocess.check_output(["git", "-C", str(project), "rev-parse", "HEAD^{tree}"],
                                   text=True).strip()
    schedule = read_schedule(args.schedule)
    mappo = project / "research_artifacts/dissertation_v1/checkpoint/mappo_selected_validation_v1.pt"
    ddpg = package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5"
    rgb = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    args.output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = materialize_worlds(package / "worlds/hu_aligned_arena.world", args.output,
                                    {run[0] for run in schedule})
        files = dict(mappo=mappo, rgb=rgb, ddpg=ddpg, schedule=args.schedule, runner=Path(__file__),
                     shared_runner=Path(__file__).with_name("run_clearance_study.py"), **worlds)
        preserve_json(args.output / "experiment_manifest.json", {
            "schedule": [list(row) for row in schedule], "horizon_s": args.horizon,
            "source_tree": tree, "input_sha256": {key: digest(path) for key, path in files.items()},
        })
        results = [run_one(run, args, package, worlds, mappo, rgb) for run in schedule]
        if len({result["coverage_mask_sha256"] for result in results}) != 1:
            raise ValueError("The reference coverage mask differs across pilot runs")
        summary = args.output / "pilot_summary.csv"
        if not summary.exists():
            with summary.open("x", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(results[0]))
                writer.writeheader()
                writer.writerows(results)


if __name__ == "__main__":
    main()
