"""Compare legacy and route-aware recovery in repeated unseen topology families."""

import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from audit_gain_passage import audit_gain
from audit_route_recovery import audit_route
from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_dynamic_passage_study import (
    ARENA, create_world, crossing_audit, inputs as passive_inputs, passage_audit,
)
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_wait_margin_study import ACTOR_SHA256, RGB_SHA, ROBOTS, audit_rechecks, node_log, verify_actions


SOURCE_TREE = "924c245c7958bad3d0d9ca9181b5213caad5ca73"
MODES = ("legacy", "guarded")
SMOKE = (("route_diagnostic", (71322,), -0.63, (-1.87, 1.21), (0.25, 1.32), None,
          4.32, -1.63, ((-3.2, -3.5, -2.7, -2.1),)),)
FORMAL = (
    ("staggered_partitions", (71421, 71422), 0.85, (-1.74, 1.33), (0.26, 1.26), None,
     4.23, -1.54, ((-4.8, -2.03, -1.90, -1.83), (-0.65, -2.03, 4.8, -1.83),
                   (2.75, -4.0, 3.48, -2.8))),
    ("divided_rooms", (71423, 71424), 0.31, (-1.89, 1.59), (1.20, 1.12), None,
     4.08, -1.62, ((-0.19, -5.0, 0.01, -3.04), (-0.19, -1.86, 0.01, 0.21),
                   (-3.42, -3.64, -2.81, -2.70), (2.76, -1.5, 3.30, -0.71))),
    ("offset_approaches", (71425, 71426), -0.17, (-1.31, 1.71), (1.03, 1.24), None,
     4.27, -1.59, ((-0.20, 1.0, 0.0, 3.15), (1.05, 1.65, 2.75, 1.85),
                   (-2.90, -2.58, -0.65, -2.38), (0.69, -2.58, 4.8, -2.38))),
)


def schedule(stage):
    scenes = SMOKE if stage == "smoke" else FORMAL
    runs, geometries = [], {}
    for index, (name, seeds, wall_y, centres, widths, blocked, spawn_y, yaw, extras) in enumerate(scenes):
        openings = [(center - width / 2, center + width / 2)
                    for center, width in zip(centres, widths)]
        ends = [ARENA[0], *[x for gap in openings for x in gap], ARENA[2]]
        walls = [(ends[i], wall_y - 0.1, ends[i + 1], wall_y + 0.1)
                 for i in range(0, len(ends), 2)]
        if blocked is not None:
            left, right = openings[blocked]
            walls.append((left - 0.2, wall_y - 0.49, right + 0.2, wall_y - 0.31))
        geometries[name] = dict(arena=ARENA, obstacles=walls + list(extras), openings=openings,
                                wall_y=wall_y, blocked_exit=blocked, spawn_y=spawn_y, spawn_yaw=yaw)
        for repeat, seed in enumerate(seeds):
            order_modes = MODES if (index + repeat) % 2 == 0 else MODES[::-1]
            for order, mode in enumerate(order_modes, 1):
                runs.append(dict(stage="eval", scene=name, environment_seed=seed,
                                 layout="passage_audit", order=order, cue="lidar", rule_cue="lidar",
                                 strategy="margin", wait_mode="cancel", passage_mode="rerank",
                                 passage_strategy="gain_region", route_recovery_mode=mode, horizon=120))
    return runs, geometries


def inputs(run, package, actor, rgb, files, geometry, directory):
    record = passive_inputs(run, package, actor, rgb, files, geometry, directory)
    record["launch"].extend((f"rgb_passage_strategy:={run['passage_strategy']}",
                             f"route_recovery_mode:={run['route_recovery_mode']}"))
    return record


def verify(run, package, actor, rgb, files, geometry, directory):
    if json.loads((directory / "inputs.json").read_text()) != inputs(
            run, package, actor, rgb, files, geometry, directory):
        raise ValueError(f"Launch parameters differ in {directory}")
    result = verify_metrics(directory, run, rules=run["rule_cue"])
    metrics = csv_rows(directory / "metrics.csv")
    events = read_json_lines(directory / "decisions.jsonl")
    logs = {robot: node_log(directory, robot + "_explorer") for robot in ROBOTS}
    actor_log = node_log(directory, "mappo_policy")
    if (f"loaded mappo actor checkpoint {actor}" not in actor_log
            or "reference coverage omitted from actor input" not in actor_log
            or any(f"[{name}] safety shield: enabled=True" not in log for name, log in logs.items())
            or {row["control_progress_mode"] for row in metrics} != {"observed_free_area"}
            or any(int(row["tail_completion_activations"]) for row in metrics)):
        raise ValueError(f"Shared actor, shield or map information differs in {directory}")
    actions = verify_actions(run, directory)
    detail = audit_gain(run, events, directory)
    recovery = audit_route(run, events, logs, result, directory)
    return dict(**run, **result, **audit_decisions(run, directory, logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **detail, **recovery, **crossing_audit(metrics, geometry))


def trial(run, package, actor, rgb, files, geometry, output, audit_only):
    name = f"{run['scene']}_{run['environment_seed']}_{run['route_recovery_mode']}"
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
    elif audit_only:
        raise ValueError(f"Missing completed trial {name}")
    else:
        for directory in sorted(output.glob(name + ".*")):
            status = directory / "exit_status.txt"
            if status.is_file() and status.read_text().strip() == "0":
                result = verify(run, package, actor, rgb, files, geometry, directory)
                preserve_json(marker, dict(directory=directory.name, result=result))
                return result
        for _ in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            record = inputs(run, package, actor, rgb, files, geometry, directory)
            preserve_json(directory / "inputs.json", record)
            print(f"START {name} {directory}", flush=True)
            status = run_launch(
                ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                 "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *record["launch"]],
                directory / "launch.log", logs_environment(directory))
            (directory / "exit_status.txt").write_text(f"{status}\n")
            if not startup_segfault(directory):
                break
    result = verify(run, package, actor, rgb, files, geometry, directory)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"VERIFIED {name} coverage={result['coverage_pct']:.2f}% "
          f"crossing={result['robots_crossing']} stuck={result['stuck_events']} "
          f"reported={result['route_events']} far_goals_retired={result['route_far_goals_retired']} "
          f"path={result['path_m']:.2f}m", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("smoke", "formal"))
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("rgb", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, actor, rgb, output = (path.resolve() for path in (args.ros_project, args.actor, args.rgb, args.output))
    package = project / "turtlebot_ddpg"
    if (Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()).resolve() != package
            or digest(actor) != ACTOR_SHA256 or digest(rgb) != RGB_SHA):
        raise ValueError("Workspace or checkpoint fingerprint differs")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if tree != SOURCE_TREE:
        raise ValueError("Use the audited route-recovery source tree")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = schedule(args.stage)
    dependencies = ("run_route_recovery_study", "audit_route_recovery", "audit_gain_passage", "run_dynamic_passage_study", "run_wait_margin_study",
                    "run_policy_goal_retention", "run_matched_vision_training", "run_clearance_study",
                    "run_rgb_clearance_guard_study", "run_rgb_passage_study")
    sources = {name: Path(__file__).with_name(name + ".py") for name in dependencies}
    sources.update(actor=actor, rgb=rgb, ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5")
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = {name: create_world(name, geometry, output) for name, geometry in geometries.items()}
        preserve_json(output / "experiment_manifest.json", dict(
            stage=args.stage, runs=runs, source_tree=tree,
            world_sha256={name: digest(files[0]) for name, files in worlds.items()},
            reference_mask_input_sha256={name: digest(files[1]) for name, files in worlds.items()},
            input_sha256={name: digest(path) for name, path in sources.items()}))
        if args.plan_only:
            return
        results = [trial(run, package, actor, rgb, worlds[run["scene"]], geometries[run["scene"]],
                         output, args.audit_only) for run in runs]
        for scene in geometries:
            group = [row for row in results if row["scene"] == scene]
            if len({row["coverage_mask_sha256"] for row in group}) != 1:
                raise ValueError(f"Reference masks differ for {scene}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
