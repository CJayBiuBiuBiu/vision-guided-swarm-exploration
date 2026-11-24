"""Compare shadow depth monitoring and velocity limiting in matched swarm runs."""

import argparse
from collections import Counter
import fcntl
import json
import math
from pathlib import Path
import subprocess
import tempfile

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_depth_corridor_study import count_block_contacts, materialize_worlds
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_wait_margin_study import (
    ACTOR_SHA256, RGB_SHA, ROBOTS, launch_arguments, node_log, verify_actions, audit_rechecks,
)


SOURCE_TREE = "cca36097bcc4c2c1f1640813f1525eb70f3eddbe"
DEPTH_STOP_M = 0.42
DEPTH_SLOW_M = 0.72
STALE_S = 0.40
FORMAL = (("low_blocks", 67011), ("low_blocks", 67012),
          ("clear", 67013), ("low_blocks", 67014), ("low_blocks", 67015), ("clear", 67016))
MODES = ("observe", "limit")


def plan(stage):
    blocks = FORMAL if stage == "formal" else (("low_blocks", 67010),)
    return [dict(stage="eval", scene=scene, environment_seed=seed,
                 layout="hu_aligned", cue="lidar", rule_cue="lidar", strategy="margin",
                 wait_mode="cancel", horizon=60 if stage == "formal" else 28,
                 depth_velocity_mode=mode, order=order)
            for index, (scene, seed) in enumerate(blocks)
            for order, mode in enumerate(MODES if index % 2 == 0 else MODES[::-1], 1)]


def inputs(run, package, actor, rgb, world, directory):
    shared = [arg for arg in launch_arguments(run, package, actor, rgb, directory)
              if not arg.startswith("world_name:=")]
    return dict(run=run, launch=shared + [f"world_name:={world}", "gazebo_lockstep:=true",
                f"depth_velocity_mode:={run['depth_velocity_mode']}",
                f"depth_velocity_trace_dir:={directory}"],
                actor_sha256=ACTOR_SHA256, rgb_sha256=RGB_SHA)


def audit_depth_velocity(directory, mode):
    totals = Counter()
    for robot in ROBOTS:
        events = read_json_lines(directory / f"{robot}_depth_velocity.jsonl")
        stamps = [event["stamp_s"] for event in events]
        if not stamps or stamps != sorted(stamps):
            raise ValueError(f"Missing or unsorted DDPG trace for {robot} in {directory}")
        counts = Counter()
        for event in events:
            now, frame, hit = event["stamp_s"], event["frame"], event["confirmed"]
            if (frame is not None and not 0 <= now - frame[0] <= STALE_S + 1e-8
                    or hit is not None and (frame is None or hit[0] > frame[0]
                                           or not 0 <= now - hit[0] <= STALE_S + 1e-8)):
                raise ValueError(f"Invalid timestamp for a camera decision in {directory}")
            scale = (1.0 if hit is None else max(0.0, min(
                1.0, (hit[1] - DEPTH_STOP_M) / (DEPTH_SLOW_M - DEPTH_STOP_M))))
            before = event["before_mps"]
            expected = before * scale if mode == "limit" else before
            if (not math.isclose(scale, event["scale"], abs_tol=1e-8)
                    or not math.isclose(expected, event["applied_mps"], abs_tol=1e-8)):
                raise ValueError(f"Applied velocity differs from the depth decision in {directory}")
            eligible = before - before * scale > 0.01
            counts["controller_ticks"] += 1
            counts["fresh_ticks"] += frame is not None
            counts["would_reduce_ticks"] += eligible
            counts["reduced_ticks"] += eligible and mode == "limit"
            counts["stopped_ticks"] += eligible and scale == 0.0 and mode == "limit"
            counts["laser_full_speed_reduce_ticks"] += (
                eligible and mode == "limit" and event["front_laser_m"] is not None
                and event["front_laser_m"] >= 0.35)
        if counts["fresh_ticks"] < 0.75 * counts["controller_ticks"]:
            raise ValueError(f"Less than 75% fresh depth availability for {robot} in {directory}")
        totals.update(counts)
    return {key: totals[key] for key in (
        "controller_ticks", "fresh_ticks", "would_reduce_ticks", "reduced_ticks", "stopped_ticks",
        "laser_full_speed_reduce_ticks")}


def verify(run, package, actor, rgb, world, directory):
    if json.loads((directory / "inputs.json").read_text()) != inputs(
            run, package, actor, rgb, world, directory):
        raise ValueError(f"Launch arguments changed in {directory}")
    result = verify_metrics(directory, run, rules=run["rule_cue"])
    events = read_json_lines(directory / "decisions.jsonl")
    mode = run["depth_velocity_mode"]
    if (events[0]["depth_velocity_mode"] != mode
            or any(event["event"] in ("depth_corridor", "depth_decision", "depth_active_check")
                   for event in events)
            or {row["control_progress_mode"] for row in csv_rows(directory / "metrics.csv")} != {"observed_free_area"}):
        raise ValueError(f"Depth velocity mode or coordinator inputs differ in {directory}")
    policy = node_log(directory, "mappo_policy")
    robot_logs = {name: node_log(directory, name + "_explorer") for name in ROBOTS}
    if (f"loaded mappo actor checkpoint {actor}" not in policy
            or "reference coverage omitted from actor input" not in policy
            or any(f"[{robot}] safety shield: enabled=True" not in log
                   or f"[{robot}] depth velocity mode={mode}" not in log
                   for robot, log in robot_logs.items())):
        raise ValueError(f"Could not verify the shared actor and LiDAR shield in {directory}")
    actions = verify_actions(run, directory)
    return dict(**run, **result, **actions, **audit_decisions(run, directory, robot_logs),
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **audit_depth_velocity(directory, mode), block_contacts=count_block_contacts(events))


def trial(run, package, actor, rgb, world, output, audit_only):
    name = f"{run['scene']}_{run['environment_seed']}_{run['depth_velocity_mode']}"
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
        command = "VERIFIED"
    elif audit_only:
        raise ValueError(f"Cannot audit missing trial {name}")
    else:
        for directory in sorted(output.glob(name + ".*")):
            status = directory / "exit_status.txt"
            if status.is_file() and status.read_text().strip() == "0":
                result = verify(run, package, actor, rgb, world, directory)
                preserve_json(marker, dict(directory=directory.name, result=result))
                print(f"RECOVERED {name} {directory}", flush=True)
                return result
        for _ in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            record = inputs(run, package, actor, rgb, world, directory)
            preserve_json(directory / "inputs.json", record)
            print(f"START {name} {directory}", flush=True)
            status = run_launch(
                ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                 "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *record["launch"]],
                directory / "launch.log", logs_environment(directory))
            (directory / "exit_status.txt").write_text(f"{status}\n")
            if not startup_segfault(directory):
                break
        command = "COMPLETED"
    result = verify(run, package, actor, rgb, world, directory)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"{command} {name} coverage={result['coverage_pct']:.2f}% "
          f"contacts={result['block_contacts']} reduced={result['reduced_ticks']}/"
          f"{result['would_reduce_ticks']} ticks", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("smoke", "formal"))
    for argument in ("project", "actor", "rgb", "output"):
        parser.add_argument(argument, type=Path)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, actor, rgb, output = map(Path.resolve, (args.project, args.actor, args.rgb, args.output))
    package = project / "turtlebot_ddpg"
    found = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if (Path(found).resolve() != package or tree != SOURCE_TREE or digest(actor) != ACTOR_SHA256
            or digest(rgb) != RGB_SHA):
        raise ValueError("The prepared ROS tree or frozen model inputs differ")
    schedule = plan(args.stage)
    names = ("run_depth_velocity_study", "run_depth_corridor_study", "run_wait_margin_study",
             "run_policy_goal_retention", "run_matched_vision_training", "run_clearance_study",
             "run_rgb_clearance_guard_study", "run_rgb_passage_study")
    sources = {name: Path(__file__).with_name(name + ".py") for name in names}
    sources.update(actor=actor, rgb=rgb,
                   ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5")
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = materialize_worlds(package / "worlds/hu_aligned_arena.world", output,
                                    {run["scene"] for run in schedule})
        preserve_json(output / "experiment_manifest.json", dict(
            stage=args.stage, runs=schedule, source_tree=tree, stop_m=DEPTH_STOP_M,
            slow_m=DEPTH_SLOW_M, maximum_age_s=STALE_S,
            input_sha256={name: digest(path) for name, path in {**sources, **worlds}.items()}))
        if args.plan_only:
            return
        results = [trial(run, package, actor, rgb, worlds[run["scene"]], output, args.audit_only)
                   for run in schedule]
        for a, b in zip(results[::2], results[1::2]):
            if a["coverage_mask_sha256"] != b["coverage_mask_sha256"]:
                raise ValueError(f"Coverage masks differ for seed {a['environment_seed']}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
