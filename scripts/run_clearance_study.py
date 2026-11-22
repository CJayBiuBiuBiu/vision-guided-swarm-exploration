"""Run a sequential, resumable directional-clearance study in a sourced ROS workspace."""

import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path


MATCHED_MODE = "matched_directional_clearance"
ROLE_CONDITIONS = {
    "matched_lidar_lidar": ("lidar", "lidar"),
    "matched_fusion_lidar": ("fusion", "lidar"),
    "matched_lidar_fusion": ("lidar", "fusion"),
    "matched_fusion_fusion": ("fusion", "fusion"),
}
QUALITY_CONDITIONS = {
    f"quality_{profile}_{cue}": (profile, cue)
    for profile in ("clean", "dark")
    for cue in ("lidar", "fusion", "adaptive")
}
MIXED_CONDITIONS = {
    f"mixed_{cue}": ("mixed", cue) for cue in ("lidar", "fusion", "adaptive")
}
CAMERA_CONDITIONS = {**QUALITY_CONDITIONS, **MIXED_CONDITIONS}
MATCHED_CONDITIONS = {
    **ROLE_CONDITIONS,
    **{name: (cue, cue) for name, (_, cue) in CAMERA_CONDITIONS.items()},
}
CONDITIONS = {
    "lidar": ("lidar_openness", "buffered"),
    "latest_fusion": ("rgb_lidar_openness", "latest"),
    "buffered_fusion": ("rgb_lidar_openness", "buffered"),
    **{condition: (MATCHED_MODE, "buffered") for condition in MATCHED_CONDITIONS},
}
WORLDS = ("hu_aligned", "hu_layout_c")


def digest(path):
    with path.open("rb") as stream:
        sha256 = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha256.update(block)
    return sha256.hexdigest()


def preserve_json(path, record):
    if path.exists():
        if json.loads(path.read_text()) != record:
            raise ValueError(f"Cannot resume with changed inputs or results at {path}")
    else:
        with path.open("x") as stream:
            json.dump(record, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")


def read_schedule(path):
    with path.open(newline="") as stream:
        raw = list(csv.DictReader(stream))
    runs = [(row["layout"], int(row["seed"]), int(row["order"]), row["condition"]) for row in raw]
    blocks = defaultdict(list)
    for world, seed, order, condition in runs:
        if world not in WORLDS or seed < 0 or condition not in CONDITIONS:
            raise ValueError(f"Invalid run {world} {seed} {order} {condition}")
        blocks[world, seed].append((order, condition))
    if not blocks:
        raise ValueError("The study schedule is empty")
    expected = {run[3] for run in runs}
    if expected.intersection(ROLE_CONDITIONS) and expected != ROLE_CONDITIONS.keys():
        raise ValueError("A matched role study requires all four conditions")
    if expected.intersection(QUALITY_CONDITIONS) and expected != QUALITY_CONDITIONS.keys():
        raise ValueError("A camera-quality study requires all six conditions")
    if expected.intersection(MIXED_CONDITIONS) and expected != MIXED_CONDITIONS.keys():
        raise ValueError("A mixed-exposure study requires all three conditions")
    for block in blocks.values():
        if ([order for order, _ in block] != list(range(1, len(expected) + 1))
                or {condition for _, condition in block} != expected):
            raise ValueError("Each seed and layout must run each condition in the study once")
    return runs


def verify_matched_cues(assignment, actor, rules, weight, directory, thresholds=None):
    pair = assignment["fusion_pair"]
    lidar = pair["cue_lidar_scores"]
    rgb = pair["paired_rgb_scores"]
    paired = pair["status"] in ("latest", "buffered")
    stamp = pair["cue_lidar_stamp_s"]
    if (paired != (rgb is not None) or (stamp is None) != (lidar is None)
            or pair["lidar_fallback"] != (lidar is not None and not paired)
            or (paired and not (lidar is not None and 0.0 <= pair["paired_skew_s"] <= 0.25))):
        raise ValueError(f"Inconsistent matched sensor diagnostics in {directory}")
    if (lidar is not None and len(lidar) != 3) or (rgb is not None and len(rgb) != 3):
        raise ValueError(f"Invalid directional scores in {directory}")
    fused = ([weight * image + (1.0 - weight) * scan for image, scan in zip(rgb, lidar)]
             if paired else lidar)
    vectors = {"lidar": lidar, "fusion": fused}
    if thresholds is not None:
        quality = pair["camera_quality"]
        if paired != (quality is not None):
            raise ValueError(f"Camera-quality evidence does not match paired input in {directory}")
        accepted = False
        if quality is not None:
            luma, sharpness, reason = quality
            expected = "accepted"
            if not (math.isfinite(luma) and math.isfinite(sharpness)):
                expected = "invalid"
            elif luma < thresholds["minimum_luma"]:
                expected = "underexposed"
            elif sharpness < thresholds["minimum_sharpness"]:
                expected = "low_detail"
            if reason != expected:
                raise ValueError(f"Recorded camera quality does not match the profile in {directory}")
            accepted = reason == "accepted"
        vectors["adaptive"] = fused if accepted else lidar
    if (assignment["actor_directional_clearance"] != vectors[actor]
            or assignment["directional_clearance"] != vectors[rules]):
        raise ValueError(f"Recorded cue does not match its pathway or common LiDAR sample in {directory}")


def verify_mixed_window_stats(assignments, last, horizon, ticks, paired_ticks, accepted, directory):
    windows = json.loads(last["mixed_camera_windows_json"])
    if not isinstance(windows, list) or len(windows) != 5:
        raise ValueError(f"Missing mixed camera windows in {directory}")
    totals = Counter()
    for index, window in enumerate(windows):
        total = window.get("robot_ticks", 0)
        paired = window.get("paired", 0)
        allowed = window.get("accepted", 0)
        cross = window.get("cross_window", 0)
        allowed_cross = window.get("accepted_cross_window", 0)
        if (not 0 <= allowed <= paired <= total or not 0 <= cross <= paired
                or not 0 <= allowed_cross <= min(cross, allowed)
                or allowed - allowed_cross > paired - cross
                or (horizon > 60 * index and total <= 0)):
            raise ValueError(f"Invalid exposure-window telemetry in {directory}")
        totals.update(window)
    if (totals["robot_ticks"], totals["paired"], totals["accepted"]) != (
            3 * ticks, paired_ticks, accepted):
        raise ValueError(f"Exposure windows do not reconcile with timing telemetry in {directory}")
    for assignment in assignments:
        pair = assignment["fusion_pair"]
        control_stamp = pair["mixed_control_stamp_s"]
        image_stamp = pair["mixed_rgb_stamp_s"]
        if (not math.isfinite(control_stamp) or control_stamp < 0
                or pair["mixed_control_window"] != min(4, int(control_stamp // 60))
                or (pair["camera_quality"] is None) != (image_stamp is None)):
            raise ValueError(f"Invalid exposure clock in {directory}")
        if image_stamp is None:
            if pair["mixed_rgb_window"] is not None:
                raise ValueError(f"Unpaired input has a camera window in {directory}")
        elif (not math.isfinite(image_stamp) or not 0 <= control_stamp - image_stamp <= 0.75
              or pair["mixed_rgb_window"] != min(4, int(image_stamp // 60))):
            raise ValueError(f"Image exposure does not match its timestamp in {directory}")
    return windows


def verify_run(directory, world, seed, condition, horizon, package, quality_profile=None):
    mode, pairing = CONDITIONS[condition]
    if (directory / "exit_status.txt").read_text().strip() != "0":
        raise ValueError(f"roslaunch did not exit cleanly in {directory}")
    failure = r"Traceback|Segmentation fault|REQUIRED process \[(robot[123]_explorer|mappo_policy)"
    if re.search(failure, (directory / "launch.log").read_text(errors="replace")):
        raise ValueError(f"Simulator or policy failure in {directory}")
    with (directory / "metrics.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Empty metrics in {directory}")
    last = rows[-1]
    expected = {
        "world_profile": world, "experiment_seed": str(seed),
        "vision_scoring_mode": mode, "selection_mode": "mappo_hybrid",
        "mapping_mode": "laser_scan", "pose_source": "gazebo_truth",
        "control_progress_mode": "observed_free_area", "vision_role": "both",
        "robot_count": "3",
    }
    if condition in MATCHED_CONDITIONS:
        actor, rules = MATCHED_CONDITIONS[condition]
        expected.update(clearance_actor_cue=actor, clearance_rules_cue=rules,
                        fusion_pairing_mode=pairing, clearance_rgb_weight="0.5")
    thresholds = None
    if condition in CAMERA_CONDITIONS:
        camera_profile, cue = CAMERA_CONDITIONS[condition]
        expected["clearance_camera_profile"] = camera_profile
        if quality_profile is None:
            raise ValueError("The original camera-quality file is required")
        thresholds = json.loads(quality_profile.read_text())["thresholds"]
    if any({row[key] for row in rows} != {value} for key, value in expected.items()):
        raise ValueError(f"Logged configuration does not match the schedule in {directory}")
    elapsed = float(last["elapsed_sim_s"])
    if (last["evaluation_horizon_reached"] != "1" or last["mission_done"] != "0"
            or not horizon <= elapsed <= horizon + 1 or last["scan_ready_robots"] != "3"
            or int(last["policy_actions_received"]) <= 0):
        raise ValueError(f"Incomplete simulation or missing control inputs in {directory}")
    ticks = int(last["clearance_team_ticks"])
    ready = int(last["clearance_ready_team_ticks"])
    if ticks <= 0 or not 0 <= ready <= ticks:
        raise ValueError(f"Invalid cue availability counts in {directory}")
    mask = last["coverage_mask_sha256"]
    if not re.fullmatch(r"[0-9a-f]{64}", mask) or {row["coverage_mask_sha256"] for row in rows} != {mask}:
        raise ValueError(f"Invalid coverage mask in {directory}")
    sizes = {int(row["coverage_explorable_cells"]) for row in rows}
    if len(sizes) != 1 or min(sizes) <= 0 or any(
            not math.isclose(100 * int(row["coverage_explored_cells"]) / min(sizes),
                             float(row["coverage_pct"]), abs_tol=0.02) for row in rows):
        raise ValueError(f"Coverage percentages do not reproduce the logged counts in {directory}")
    events = [json.loads(line) for line in (directory / "decisions.jsonl").read_text().splitlines()]
    assignments = [event for event in events if event["event"] == "assigned"]
    assigned = [event["attempt_id"] for event in assignments]
    outcomes = [event for event in events if event["event"] == "outcome"]
    if (not events or events[-1].get("reason") != "simulation_horizon" or
            not assigned or sorted(assigned) != sorted(event["attempt_id"] for event in outcomes)
            or len(assigned) != len(set(assigned))):
        raise ValueError(f"Incomplete frontier trace in {directory}")
    pair_counts = {}
    if mode in ("rgb_lidar_openness", MATCHED_MODE):
        pair_counts = json.loads(last["fusion_pair_outcomes_json"])
        if last["fusion_pairing_mode"] != pairing or sum(pair_counts.values()) != 3 * ticks:
            raise ValueError(f"Invalid fusion timing telemetry in {directory}")
    role_details = {}
    if condition in MATCHED_CONDITIONS:
        if events[0]["event"] != "run" or (events[0]["clearance_actor_cue"],
                                          events[0]["clearance_rules_cue"]) != (actor, rules):
            raise ValueError(f"Trace header does not match the experiment in {directory}")
        for assignment in assignments:
            verify_matched_cues(assignment, actor, rules, 0.5, directory, thresholds)
        paired_ticks = pair_counts.get("latest", 0) + pair_counts.get("buffered", 0)
        fallback = int(last["matched_lidar_fallback_ticks"])
        if paired_ticks <= 0 or not 0 <= fallback <= 3 * ticks - paired_ticks:
            raise ValueError(f"Missing matched samples or invalid fallback counts in {directory}")
        role_details = {
            "actor_cue": actor, "rules_cue": rules,
            "matched_lidar_fallback_ticks": fallback,
            "trace_outcome_counts": dict(Counter(event["outcome"] for event in outcomes)),
        }
    if thresholds is not None:
        if events[0]["clearance_camera_profile"] != camera_profile:
            raise ValueError(f"Trace camera profile differs from the schedule in {directory}")
        quality_counts = json.loads(last["camera_pair_quality_json"])
        if (sum(quality_counts.values()) != 3 * ticks
                or quality_counts.get("unpaired", 0) != 3 * ticks - paired_ticks):
            raise ValueError(f"Camera-quality counts do not reconcile in {directory}")
        gate_acceptance = quality_counts.get("accepted", 0)
        rgb_used_ticks = {"lidar": 0, "fusion": paired_ticks, "adaptive": gate_acceptance}[cue]
        role_details.update(camera_profile=camera_profile, cue=cue,
                            camera_quality_outcomes=quality_counts,
                            gate_acceptance_fraction=gate_acceptance / paired_ticks,
                            rgb_robot_tick_fraction=rgb_used_ticks / (3 * ticks))
        if condition in MIXED_CONDITIONS:
            role_details["mixed_camera_windows"] = verify_mixed_window_stats(
                assignments, last, horizon, ticks, paired_ticks, gate_acceptance, directory)
    sys.path.insert(0, str(package / "scripts"))
    from summarize_evaluation import time_weighted_coverage

    return {
        "layout": world, "seed": seed, "condition": condition, "simulation_seconds": elapsed,
        "coverage_mask_sha256": mask, "coverage_pct": float(last["coverage_pct"]),
        "mean_coverage_pct": time_weighted_coverage(rows, horizon),
        "path_m": float(last["path_length_total_m"]), "stuck_events": int(last["stuck_events"]),
        "near_collision_events": int(last["near_collision_events"]),
        "recovery_results": int(last["recovery_results"]),
        "recovery_successes": int(last["recovery_successes"]),
        "team_cue_ready_fraction": ready / ticks,
        "robot_tick_pair_outcomes": pair_counts,
        "invalid_clearance_scans": int(last["invalid_clearance_scans"]),
        "trace_assignments": len(assigned),
        "trace_arrivals": sum(event["outcome"] == "reached" for event in outcomes),
        "trace_horizon_censored": sum(event["outcome"] == "simulation_horizon" for event in outcomes),
        "time_to_90_s": threshold_time(rows, 90),
        "time_to_97_s": threshold_time(rows, 97),
        **role_details,
    }


def threshold_time(rows, threshold):
    return next((float(row["elapsed_sim_s"]) for row in rows
                 if float(row["coverage_pct"]) >= threshold), None)


def stop_launch(process):
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        return process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.wait()


def run_launch(command, path, environment):
    with path.open("xb") as output, path.open("rb") as monitor:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                   stderr=subprocess.STDOUT, env=environment, start_new_session=True)
        tail = b""
        try:
            while True:
                try:
                    return process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    report = tail + monitor.read()
                    if b"Segmentation fault" in report:
                        return stop_launch(process)
                    tail = report[-32:]
        finally:
            if process.poll() is None:
                stop_launch(process)


def startup_segfault(directory):
    if b"Segmentation fault" not in (directory / "launch.log").read_bytes():
        return False
    metrics = directory / "metrics.csv"
    if not metrics.exists():
        return True
    with metrics.open(newline="") as stream:
        return all(float(row["elapsed_sim_s"]) <= 5.0 for row in csv.DictReader(stream))


def launch_condition(run, args, package, rgb, mappo, name):
    world, seed, order, condition = run
    mode, pairing = CONDITIONS[condition]
    for process in ("gzserver", "gzclient", "rosmaster", "roslaunch"):
        if subprocess.run(["pgrep", "-x", process], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f"Another ROS or Gazebo process is running: {process}")
    directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=args.output))
    log_directory = directory / "ros_logs"
    log_directory.mkdir()
    launch_args = [
        "gui:=false", f"world_profile:={world}",
        f"world_name:={package / 'worlds' / (world + '_arena.world')}",
        "robot1_x:=-0.8", "robot1_y:=4.0", "robot1_yaw:=-1.5708",
        "robot2_x:=0.0", "robot2_y:=4.0", "robot2_yaw:=-1.5708",
        "robot3_x:=0.8", "robot3_y:=4.0", "robot3_yaw:=-1.5708",
        "selection_mode:=mappo_hybrid", "mapping_mode:=laser_scan",
        "pose_source:=gazebo_truth", "vision_enabled:=true", "vision_role:=both",
        "vision_projection_mode:=thirds", f"vision_scoring_mode:={mode}",
        f"fusion_pairing_mode:={pairing}", "clearance_rgb_weight:=0.5",
        f"learned_vision_checkpoint:={rgb}", "start_network_policy:=true",
        f"mappo_checkpoint:={mappo}", "network_deterministic:=true",
        "start_mappo_training:=false", f"experiment_seed:={seed}", f"gazebo_seed:={seed}",
        "control_progress_mode:=observed_free_area", f"max_simulation_seconds:={args.horizon}",
        "fixed_horizon_only:=true", "complete_on_coverage:=false",
        "shutdown_on_done:=true", "tail_completion_enabled:=false",
        f"metrics_path:={directory / 'metrics.csv'}",
        f"frontier_trace_path:={directory / 'decisions.jsonl'}",
    ]
    if condition in MATCHED_CONDITIONS:
        actor, rules = MATCHED_CONDITIONS[condition]
        launch_args += [f"clearance_actor_cue:={actor}", f"clearance_rules_cue:={rules}"]
    if condition in CAMERA_CONDITIONS:
        camera_profile, _ = CAMERA_CONDITIONS[condition]
        launch_args += [f"clearance_camera_profile:={camera_profile}",
                        f"clearance_quality_profile:={args.quality_profile}"]
    (directory / "launch_arguments.txt").write_text("\n".join(launch_args) + "\n")
    print(f"START {name} order={order} {directory}", flush=True)
    status = run_launch(
        ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * args.horizon}s",
         "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *launch_args],
        directory / "launch.log", {**os.environ, "ROS_LOG_DIR": str(log_directory)})
    (directory / "exit_status.txt").write_text(f"{status}\n")
    return directory


def run_condition(run, args, package, rgb, mappo):
    world, seed, order, condition = run
    name = f"{world}_{seed}_{condition}"
    marker = args.output / (name + ".complete.json")
    if marker.exists():
        previous = json.loads(marker.read_text())
        preserve_json(args.output / previous["attempt"] / "result.json",
                      verify_run(args.output / previous["attempt"], world, seed, condition,
                                 args.horizon, package, args.quality_profile))
        print(f"SKIP {name}", flush=True)
        return previous["coverage_mask_sha256"]

    directory = launch_condition(run, args, package, rgb, mappo, name)
    if startup_segfault(directory):
        print(f"RETRY {name} after early segmentation fault; retained {directory}", flush=True)
        directory = launch_condition(run, args, package, rgb, mappo, name)
    record = verify_run(directory, world, seed, condition, args.horizon, package, args.quality_profile)
    preserve_json(directory / "result.json", record)
    preserve_json(marker, {"attempt": directory.name, "order": order,
                           "coverage_mask_sha256": record["coverage_mask_sha256"]})
    print(f"COMPLETE {name} coverage={record['coverage_pct']:.2f}% "
          f"team cue ready={record['team_cue_ready_fraction']:.1%}", flush=True)
    return record["coverage_mask_sha256"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("schedule", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--horizon", type=int, default=300)
    parser.add_argument("--quality-profile", type=Path)
    args = parser.parse_args()
    if args.horizon <= 0:
        parser.error("horizon must be positive")
    args.ros_project = args.ros_project.resolve()
    args.output = args.output.resolve()
    package = args.ros_project / "turtlebot_ddpg"
    found = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    if Path(found).resolve() != package:
        raise ValueError(f"The sourced ROS workspace uses another package: {found}")
    subprocess.run(["git", "-C", str(args.ros_project), "diff", "--quiet", "HEAD", "--",
                    "turtlebot_ddpg", "laser_filters", "turtlebot3_description"], check=True)
    tree = subprocess.check_output(["git", "-C", str(args.ros_project),
                                    "rev-parse", "HEAD^{tree}"], text=True).strip()
    rgb = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    mappo = args.ros_project / "research_artifacts/dissertation_v1/checkpoint/mappo_selected_validation_v1.pt"
    ddpg = package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5"
    schedule = read_schedule(args.schedule)
    quality_study = any(condition in CAMERA_CONDITIONS for _, _, _, condition in schedule)
    if quality_study != (args.quality_profile is not None):
        parser.error("Supply --quality-profile for a camera-quality schedule")
    files = {"rgb": rgb, "mappo": mappo, "ddpg": ddpg, "runner": Path(__file__),
             "schedule": args.schedule, "launch": package / "launch/gazebo_research_experiment.launch",
             **{world: package / "worlds" / (world + "_arena.world")
                for world in sorted({run[0] for run in schedule})}}
    if quality_study:
        args.quality_profile = args.quality_profile.resolve()
        files["quality_profile"] = args.quality_profile

    args.output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(args.output / "experiment_manifest.json", {
            "horizon_s": args.horizon, "source_tree": tree,
            "input_sha256": {name: digest(path) for name, path in files.items()},
            "schedule": [list(run) for run in schedule],
        })
        masks = {}
        for run in schedule:
            mask = run_condition(run, args, package, rgb, mappo)
            if masks.setdefault(run[0], mask) != mask:
                raise ValueError(f"Coverage mask changed within layout {run[0]}")


if __name__ == "__main__":
    main()
