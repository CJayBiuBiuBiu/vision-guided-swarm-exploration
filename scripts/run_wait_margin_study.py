"""Compare confidence-gated MAPPO continuation with explicit cancellation and short hold."""

import argparse
from collections import Counter
import fcntl
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import (
    ACTOR_SHA256, RGB_SHA, SHIELD_CLEARANCE, audit_decisions, read_json_lines, settings,
)
from run_rgb_passage_study import check_idle, logs_environment


SOURCE_TREE = "2c38908fd69faa36e5101e57760b82fa959b0967"
MARGIN = 0.10
RECHECK_LIMIT_S = 12.0
OPTIONS = (("lidar", "cancel"), ("guarded_fusion", "cancel"),
           ("lidar", "hold"), ("guarded_fusion", "hold"),
           ("lidar", "margin"), ("guarded_fusion", "margin"))
HU_ORDER = (0, 5, 2, 1, 4, 3)
C_ORDER = (3, 0, 4, 1, 5, 2)
BLOCKS = (("hu_aligned", 66521, HU_ORDER), ("hu_layout_c", 66522, C_ORDER),
          ("hu_aligned", 66523, HU_ORDER[::-1]), ("hu_layout_c", 66524, C_ORDER[::-1]))
ROBOTS = ("robot1", "robot2", "robot3")


def plan(stage):
    blocks = BLOCKS if stage == "formal" else (("hu_aligned", 66525, (3, 5)),)
    return [
        dict(stage="eval", layout=layout, environment_seed=seed, order=order, cue="lidar",
             rule_cue=OPTIONS[index][0], strategy=OPTIONS[index][1],
             wait_mode="hold" if OPTIONS[index][1] == "hold" else "cancel",
             horizon=60 if stage == "formal" else 25)
        for layout, seed, indices in blocks for order, index in enumerate(indices, 1)]


def launch_arguments(run, package, actor, rgb, directory):
    margin = MARGIN if run["strategy"] == "margin" else 0.0
    return settings(run, package, actor, rgb, directory) + [f"policy_wait_margin:={margin}"]


def verify_actions(run, directory):
    events = read_json_lines(directory / "actions.jsonl")
    if {item["robot"] for item in events} != set(ROBOTS):
        raise ValueError(f"Missing actor telemetry in {directory}")
    count = Counter()
    for event in events:
        raw, delivered = event["action_index"], event["action_type"]
        gap = event["wait_minus_continue"]
        active = event["active_goal"] and event["can_continue"] and event["can_wait"]
        if not 0 <= raw <= 49 or (active and (gap is None or not math.isfinite(gap))):
            raise ValueError(f"Invalid actor score or action in {directory}")
        if raw == 48 and (not active or gap > 0):
            raise ValueError(f"CONTINUE did not match the active action scores in {directory}")
        if raw == 49 and active and gap < 0:
            raise ValueError(f"WAIT did not match the active action scores in {directory}")
        recheck = run["strategy"] == "margin" and active and raw == 49 and 0.0 <= gap <= MARGIN
        expected = 3 if recheck else 2 if raw == 49 else 1 if raw == 48 else 0
        if delivered != expected:
            raise ValueError(f"Published action does not match configured inference in {directory}")
        count["active_decisions"] += active
        count["raw_active_waits"] += active and raw == 49
        count["published_active_waits"] += active and delivered == 2
        count["recheck_requests"] += recheck
        if active and not event["selectable_frontiers"]:
            count["wait_continue_only"] += 1
            count["wait_continue_only_raw_wait"] += raw == 49
    return {key: count[key] for key in (
        "active_decisions", "raw_active_waits", "published_active_waits", "recheck_requests",
        "wait_continue_only", "wait_continue_only_raw_wait")}


def audit_rechecks(run, directory, requests):
    events = read_json_lines(directory / "decisions.jsonl")
    if events[0]["policy_wait_margin"] != (MARGIN if run["strategy"] == "margin" else 0.0):
        raise ValueError(f"Runtime wait margin differs in {directory}")
    active, retained = {}, set()
    count = Counter()
    for event in events:
        kind, robot = event["event"], event.get("robot")
        if kind == "assigned":
            active[robot] = (event["attempt_id"], event["selected_frontier_id"])
        elif kind == "outcome":
            attempt = active.pop(robot)
            count["rechecked_targets_reached"] += attempt[0] in retained and event["outcome"] == "reached"
        elif kind == "policy_recheck_continue":
            count["rechecks_consumed"] += 1
            if event["keep"]:
                attempt = active.get(robot)
                if (attempt is None or attempt[1] != event["active_frontier_id"]
                        or event["reason"] != "valid_route"
                        or not 0.0 <= event["goal_age_s"] < RECHECK_LIMIT_S
                        or event["clearance_m"] is None or event["clearance_m"] <= SHIELD_CLEARANCE):
                    raise ValueError(f"Recheck continued without valid route evidence in {directory}")
                retained.add(attempt[0])
                count["rechecks_accepted"] += 1
            else:
                count[f"recheck_{event['reason']}"] += 1
    if count["rechecks_consumed"] > requests:
        raise ValueError(f"Coordinator handled more requests than the actor published in {directory}")
    return dict(rechecks_consumed=count["rechecks_consumed"], rechecks_accepted=count["rechecks_accepted"],
                rechecked_targets=len(retained), rechecked_targets_reached=count["rechecked_targets_reached"],
                recheck_age_expired=count["recheck_hold_elapsed"],
                recheck_clearance_rejected=count["recheck_clearance_low"] +
                count["recheck_clearance_missing_or_stale"],
                recheck_disconnected=count["recheck_blocked_or_disconnected"],
                recheck_no_goal=count["recheck_no_active_goal"],
                recheck_goal_changed=count["recheck_valid_route"])


def node_log(directory, node):
    matches = [path for path in directory.glob(f"ros_logs/*/{node}-*.log")
               if not path.parent.is_symlink() and re.fullmatch(fr"{node}-\d+\.log", path.name)]
    if len(matches) != 1:
        raise ValueError(f"Missing or ambiguous ROS node log for {node} in {directory}")
    return matches[0].read_text(errors="replace")


def inputs(run, package, actor, rgb, directory):
    return dict(run=run, launch=launch_arguments(run, package, actor, rgb, directory),
                actor_sha256=ACTOR_SHA256, rgb_sha256=RGB_SHA)


def verify(run, package, actor, rgb, directory):
    if json.loads((directory / "inputs.json").read_text()) != inputs(run, package, actor, rgb, directory):
        raise ValueError(f"Trial inputs differ in {directory}")
    data = verify_metrics(directory, run, rules=run["rule_cue"])
    actor_log = node_log(directory, "mappo_policy")
    robot_logs = {robot: node_log(directory, f"{robot}_explorer") for robot in ROBOTS}
    if (f"loaded mappo actor checkpoint {actor}" not in actor_log
            or "reference coverage omitted from actor input" not in actor_log
            or any(f"[{robot}] safety shield: enabled=True" not in log for robot, log in robot_logs.items())
            or {row["control_progress_mode"] for row in csv_rows(directory / "metrics.csv")} != {"observed_free_area"}):
        raise ValueError(f"Actor, progress mode or speed shield differs in {directory}")
    actions = verify_actions(run, directory)
    return dict(**run, **data, **audit_decisions(run, directory, robot_logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]))


def run_name(run):
    return f"{run['layout']}_{run['environment_seed']}_{run['rule_cue']}_{run['strategy']}"


def trial(run, package, actor, rgb, output, audit_only):
    name = run_name(run)
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
        command = "VERIFIED"
    elif audit_only:
        raise ValueError(f"Cannot audit missing experiment {name}")
    else:
        for directory in sorted(output.glob(name + ".*")):
            status = directory / "exit_status.txt"
            if status.is_file() and status.read_text().strip() == "0":
                result = verify(run, package, actor, rgb, directory)
                preserve_json(marker, dict(directory=directory.name, result=result))
                print(f"RECOVERED {name} {directory}", flush=True)
                return result
        for attempt in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            record = inputs(run, package, actor, rgb, directory)
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
    result = verify(run, package, actor, rgb, directory)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"{command} {name} coverage={result['coverage_pct']:.2f}% "
          f"reached={result['reached']}/{result['assignments']} "
          f"rechecks={result['rechecks_accepted']}/{result['rechecks_consumed']}", flush=True)
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
        raise ValueError("The prepared ROS tree, actor or encoder differ from the comparison inputs")
    schedule = plan(args.stage)
    sources = {name: Path(__file__).with_name(name + ".py") for name in (
        "run_wait_margin_study", "run_policy_goal_retention", "run_matched_vision_training",
        "run_rgb_clearance_guard_study", "run_rgb_passage_study", "run_clearance_study")}
    sources.update(actor=actor, rgb=rgb,
                   ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5")
    experiment = dict(stage=args.stage, runs=schedule, source_tree=tree, logit_margin=MARGIN,
                      recheck_age_limit_s=RECHECK_LIMIT_S, minimum_clearance_m=SHIELD_CLEARANCE,
                      input_sha256={name: digest(path) for name, path in sources.items()},
                      world_sha256={layout: digest(package / "worlds" / f"{layout}_arena.world")
                                    for layout in sorted({run["layout"] for run in schedule})})
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json", experiment)
        if args.plan_only:
            return
        results = [trial(run, package, actor, rgb, output, args.audit_only) for run in schedule]
        masks = {}
        for result in results:
            block = result["layout"], result["environment_seed"]
            mask = result["coverage_mask_sha256"]
            if masks.setdefault(block, mask) != mask:
                raise ValueError(f"Coverage mask differs within {block}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
