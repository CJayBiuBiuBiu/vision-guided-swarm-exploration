"""Compare shared and robot-scoped frontier exclusions on matched routes."""

import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from audit_gain_passage import audit_gain
from audit_recovery_scope import audit_scope
from audit_route_recovery import audit_route
from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_dynamic_passage_study import create_world, crossing_audit
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_route_recovery_study import inputs as route_inputs, schedule as route_schedule
from run_wait_margin_study import ACTOR_SHA256, RGB_SHA, ROBOTS, audit_rechecks, node_log, verify_actions


SOURCE_TREE = "7648109cfc1235f3ee58ea1b6ffbfeedbf7736ee"
SCOPES = ("shared", "robot")


def schedule(stage):
    runs, geometries = route_schedule(stage)
    scopes = {"legacy": "shared", "guarded": "robot"}
    paired_runs = [dict(run, route_recovery_mode="guarded",
                        recovery_frontier_scope=scopes[run["route_recovery_mode"]])
                   for run in runs]
    return paired_runs, geometries


def inputs(run, package, actor, rgb, files, geometry, directory):
    record = route_inputs(run, package, actor, rgb, files, geometry, directory)
    record["launch"].append(f"recovery_frontier_scope:={run['recovery_frontier_scope']}")
    return record


def verify(run, package, actor, rgb, files, geometry, directory):
    expected = inputs(run, package, actor, rgb, files, geometry, directory)
    if json.loads((directory / "inputs.json").read_text()) != expected:
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
    # Scope replay checks target rejections against the stricter robot-local position rule.
    original_events = [event for event in events if event["event"] != "route_target_rejected"]
    recovery = audit_route(run, original_events, logs, result, directory)
    scoped = audit_scope(run, events, metrics, logs, directory)
    recovery["route_target_rejections"] = scoped["scope_target_rejections"]
    return dict(**run, **result, **audit_decisions(run, directory, logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **audit_gain(run, events, directory), **recovery, **scoped,
                **crossing_audit(metrics, geometry))


def trial(run, package, actor, rgb, files, geometry, output, audit_only):
    name = f"{run['scene']}_{run['environment_seed']}_{run['recovery_frontier_scope']}"
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
    elif audit_only:
        raise ValueError(f"Missing completed trial {name}")
    else:
        previous = [directory for directory in sorted(output.glob(name + ".*"))
                    if (directory / "exit_status.txt").is_file()
                    and (directory / "exit_status.txt").read_text().strip() == "0"]
        if previous:
            directory = previous[0]
        else:
            for _ in range(2):
                check_idle()
                directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
                record = inputs(run, package, actor, rgb, files, geometry, directory)
                preserve_json(directory / "inputs.json", record)
                status = run_launch(
                    ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                     "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch",
                     *record["launch"]], directory / "launch.log", logs_environment(directory))
                (directory / "exit_status.txt").write_text(f"{status}\n")
                if not startup_segfault(directory):
                    break
    result = verify(run, package, actor, rgb, files, geometry, directory)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"VERIFIED {name} coverage={result['coverage_pct']:.2f}% "
          f"stuck={result['stuck_events']} near={result['near_collision_events']} "
          f"peer_retirements={result['route_peer_retirements']}", flush=True)
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
    workspace_package = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    if Path(workspace_package).resolve() != package or digest(actor) != ACTOR_SHA256 or digest(rgb) != RGB_SHA:
        raise ValueError("Workspace or checkpoint fingerprint differs")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if tree != SOURCE_TREE:
        raise ValueError("Use the audited robot-scoped recovery source tree")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = schedule(args.stage)
    dependencies = ("run_recovery_scope_study", "run_route_recovery_study", "audit_recovery_scope",
                    "audit_route_recovery", "audit_gain_passage", "run_dynamic_passage_study",
                    "run_wait_margin_study", "run_policy_goal_retention", "run_matched_vision_training",
                    "run_clearance_study", "run_rgb_clearance_guard_study", "run_rgb_passage_study")
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
