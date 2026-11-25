"""Compare information-gain regions with passive and active RGB passage guidance."""

import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from audit_gain_passage import ADDITIONAL_FIELDS, audit_gain
from run_active_passage_study import audit_active
from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_dynamic_passage_study import (
    ARENA, create_world, crossing_audit, inputs as passive_inputs, passage_audit,
)
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_wait_margin_study import ACTOR_SHA256, RGB_SHA, ROBOTS, audit_rechecks, node_log, verify_actions


SOURCE_TREE = "f723253c239b63e706b32036eb06ace282b1f052"
STRATEGIES = ("passive", "active", "gain_region")
ORDERS = (STRATEGIES, STRATEGIES[1:] + STRATEGIES[:1],
          STRATEGIES[2:] + STRATEGIES[:2], STRATEGIES[::-1])
SMOKE = (("gain_smoke", 71310, 0.06, (-1.54, 1.22), (1.14, 0.24), None,
          4.20, -1.60, ((2.1, -3.2, 2.7, -2.4),)),)
FORMAL = (
    ("gain_narrow_east", 71321, 0.11, (-1.48, 1.84), (1.18, 0.23), None,
     4.17, -1.52, ((1.0, -3.8, 1.7, -2.6),)),
    ("gain_narrow_west", 71322, -0.63, (-1.87, 1.21), (0.25, 1.32), None,
     4.32, -1.63, ((-3.2, -3.5, -2.7, -2.1),)),
    ("gain_west_exit", 71323, 0.61, (-1.24, 1.68), (0.82, 1.20), 0,
     4.40, -1.50, ((-0.20, -4.2, 0.40, -2.8),)),
    ("gain_east_exit", 71324, -0.36, (-1.67, 1.40), (1.14, 0.97), 1,
     4.10, -1.66, ((-3.8, -3.2, -3.1, -1.9), (2.5, -3.9, 3.0, -3.0))),
)

def schedule(stage):
    scenes = SMOKE if stage == "smoke" else FORMAL
    runs, geometries = [], {}
    for index, (name, seed, wall_y, centres, widths, blocked, spawn_y, yaw, extras) in enumerate(scenes):
        openings = [(center - width / 2, center + width / 2)
                    for center, width in zip(centres, widths)]
        ends = [ARENA[0], *[x for gap in openings for x in gap], ARENA[2]]
        walls = [(ends[i], wall_y - 0.1, ends[i + 1], wall_y + 0.1) for i in range(0, len(ends), 2)]
        if blocked is not None:
            left, right = openings[blocked]
            walls.append((left - 0.2, wall_y - 0.49, right + 0.2, wall_y - 0.31))
        geometries[name] = dict(arena=ARENA, obstacles=walls + list(extras), openings=openings,
                                wall_y=wall_y, blocked_exit=blocked, spawn_y=spawn_y, spawn_yaw=yaw)
        for order, strategy in enumerate(ORDERS[index], 1):
            runs.append(dict(stage="eval", scene=name, environment_seed=seed, layout="passage_audit",
                             order=order, cue="lidar", rule_cue="lidar", strategy="margin",
                             wait_mode="cancel", passage_mode="rerank", passage_strategy=strategy,
                             horizon=45 if stage == "smoke" else 120))
    return runs, geometries


def inputs(run, package, actor, rgb, files, geometry, directory):
    record = passive_inputs(run, package, actor, rgb, files, geometry, directory)
    record["launch"].append(f"rgb_passage_strategy:={run['passage_strategy']}")
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
    detail = (audit_gain(run, events, directory) if run["passage_strategy"] == "gain_region"
              else dict(**audit_active(run, events, directory), **dict.fromkeys(ADDITIONAL_FIELDS, 0)))
    return dict(**run, **result, **audit_decisions(run, directory, logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **detail, **crossing_audit(metrics, geometry))


def trial(run, package, actor, rgb, files, geometry, output, audit_only):
    name = f"{run['scene']}_{run['environment_seed']}_{run['passage_strategy']}"
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
          f"crossing={result['robots_crossing']} inspections={result['inspections_scored']}/"
          f"{result['inspections_started']} redirects={result['inspection_redirects']} "
          f"continuations={result['continuations_issued']} gain={result['gain_advantage_cells']}", flush=True)
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
        raise ValueError("Use the audited gain-region source tree")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = schedule(args.stage)
    dependencies = ("run_gain_passage_study", "audit_gain_passage", "run_active_passage_study", "run_dynamic_passage_study", "run_wait_margin_study",
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
        for first in range(0, len(results), len(STRATEGIES)):
            group = results[first:first + len(STRATEGIES)]
            if len({row["coverage_mask_sha256"] for row in group}) != 1:
                raise ValueError(f"Reference masks differ for {group[0]['scene']}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
