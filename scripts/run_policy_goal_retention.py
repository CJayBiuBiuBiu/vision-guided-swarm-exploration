"""Audit a matched six-arm MAPPO goal-execution experiment in Gazebo."""

import argparse
from collections import Counter
import fcntl
import json
import math
from pathlib import Path
import re
from statistics import median
import subprocess
import tempfile

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_matched_vision_training import csv_rows, trial_configs, verify_metrics, write_csv
from run_rgb_clearance_guard_study import audit_assignment
from run_rgb_passage_study import ACTOR_SHA256, check_idle, logs_environment


MODES = ("legacy", "cancel", "hold")
RULES = ("lidar", "guarded_fusion")
OPTIONS = tuple((cue, mode) for mode in MODES for cue in RULES)
HOLD_SECONDS = 4.5
SHIELD_CLEARANCE = 0.25
SOURCE_TREE = "5f6db4736f0b1d220191298b07ee463e9ce9a6ff"
RGB_SHA = "b3115535d34539e380d69335be76f7822ab2eeb9afab5a10674c951030217d21"
HU_ORDER = (0, 5, 2, 1, 4, 3)
C_ORDER = (3, 0, 4, 1, 5, 2)
BLOCKS = (("hu_aligned", 66521, HU_ORDER), ("hu_layout_c", 66522, C_ORDER),
          ("hu_aligned", 66523, HU_ORDER[::-1]), ("hu_layout_c", 66524, C_ORDER[::-1]))


def settings(run, package, actor, rgb, directory):
    options = trial_configs(run, {"decision_interval_s": 1.5}, package, rgb, directory, actor)
    override = [f"clearance_rules_cue:={run['rule_cue']}", "control_progress_mode:=observed_free_area",
                f"policy_wait_mode:={run['wait_mode']}", f"policy_wait_hold_s:={HOLD_SECONDS}",
                "safety_shield_enabled:=true", f"near_collision_clear_range:={SHIELD_CLEARANCE}",
                f"frontier_trace_path:={directory / 'decisions.jsonl'}",
                f"network_action_trace_path:={directory / 'actions.jsonl'}"]
    replaced = {option.split(":=", 1)[0] for option in override}
    return [option for option in options if option.split(":=", 1)[0] not in replaced] + override


def read_json_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def audit_actions(directory):
    actions = read_json_lines(directory / "actions.jsonl")
    if not actions or {item["robot"] for item in actions} != {"robot1", "robot2", "robot3"}:
        raise ValueError(f"Incomplete actor telemetry in {directory}")
    active_choices = [row for row in actions if row["active_goal"]
                      and row["can_continue"] and row["can_wait"]]
    if not active_choices:
        raise ValueError(f"No active wait/continue comparison in {directory}")
    for choice in active_choices:
        margin = choice["wait_minus_continue"]
        index = choice["action_index"]
        action = choice["action_type"]
        if (not math.isfinite(margin) or action not in (0, 1, 2)
                or (index == 49 and (action != 2 or margin < 0))
                or (index == 48 and (action != 1 or margin > 0))):
            raise ValueError(f"Actor action does not match logged preference in {directory}")
    return dict(actor_decisions=len(actions), active_actor_decisions=len(active_choices),
                active_actor_wait=sum(row["action_index"] == 49 for row in active_choices),
                active_wait_preferred_over_continue=sum(
                    row["wait_minus_continue"] > 0 for row in active_choices),
                median_wait_minus_continue=median(row["wait_minus_continue"] for row in active_choices),
                active_wait_fallbacks=sum(row["action_type"] == 2 and row["action_index"] != 49
                                          for row in active_choices))


def audit_decisions(run, directory, log):
    events = read_json_lines(directory / "decisions.jsonl")
    header = events[0]
    rules, mode = run["rule_cue"], run["wait_mode"]
    if ((header["clearance_actor_cue"], header["clearance_rules_cue"],
         header["policy_wait_mode"], header["policy_wait_hold_s"]) != ("lidar", rules, mode, HOLD_SECONDS)
            or events[-1]["event"] != "run_end"):
        raise ValueError(f"Decision trace configuration or closure differs in {directory}")
    pending, retained, durations = {}, set(), []
    outcomes = Counter()
    held = Counter()
    blocked_waits = Counter()
    assignments = waited = kept = retired = 0
    low = []
    for event in events:
        activity = event["event"]
        if activity == "assigned":
            low.extend(audit_assignment(event, rules, directory))
            assignments += 1
            pending[event["robot"]] = (event["attempt_id"], event["selected_frontier_id"], event["elapsed_sim_s"])
        elif activity == "policy_wait":
            waited += int(event["active_frontier_id"] is not None)
            if event["keep"]:
                attempt = pending.get(event["robot"])
                if (mode != "hold" or event["reason"] != "valid_route" or attempt is None
                        or event["active_frontier_id"] != attempt[1]
                        or not 0.0 <= event["goal_age_s"] < HOLD_SECONDS
                        or event["clearance_m"] is None or event["clearance_m"] <= SHIELD_CLEARANCE):
                    raise ValueError(f"A retained target has no valid route and clearance evidence in {directory}")
                kept += 1
                retained.add(attempt[0])
            else:
                blocked_waits[event["reason"]] += 1
        elif activity == "outcome":
            attempt = pending.pop(event["robot"])
            if attempt[0] != event["attempt_id"]:
                raise ValueError(f"Outcome closes a mismatched target in {directory}")
            result = event["outcome"]
            outcomes[result] += 1
            if result == "wait":
                durations.append(event["elapsed_sim_s"] - attempt[2])
            if attempt[0] in retained:
                held[result] += 1
        elif activity == "policy_waypoint_retired":
            retired += 1
    received = len(re.findall(r"\[robot[123]\] retired policy waypoint", log))
    if (pending or not assignments or assignments != sum(outcomes.values())
            or (mode == "legacy" and (retired or received))
            or received > retired or (rules == "guarded_fusion" and any(abs(value) > 1e-10 for value in low))):
        raise ValueError(f"Goal, waypoint or guarded cue trace is inconsistent in {directory}")
    return dict(assignments=assignments, reached=outcomes["reached"], goals_abandoned=outcomes["wait"],
                traced_blocked=outcomes["blocked"], traced_stuck=outcomes["stuck"],
                median_abandonment_age_s=median(durations) if durations else None,
                waits_with_target=waited, waits_deferred=kept, targets_retained=len(retained),
                retained_targets_reached=held["reached"], waypoint_retires_sent=retired,
                waypoint_retires_received=received,
                clearance_or_route_preemptions=blocked_waits["clearance_low"]
                + blocked_waits["clearance_missing_or_stale"] + blocked_waits["blocked_or_disconnected"],
                low_lidar_sectors=len(low), low_sectors_increased=sum(value > 1e-10 for value in low))


def verify(run, package, actor, rgb, directory):
    expected = dict(run=run, launch=settings(run, package, actor, rgb, directory),
                    actor_sha256=ACTOR_SHA256, rgb_sha256=RGB_SHA)
    if json.loads((directory / "inputs.json").read_text()) != expected:
        raise ValueError(f"Trial inputs differ in {directory}")
    result = verify_metrics(directory, run, rules=run["rule_cue"])
    log = (directory / "launch.log").read_text(errors="replace")
    if (f"loaded mappo actor checkpoint {actor}" not in log
            or "reference coverage omitted from actor input" not in log
            or any(f"[robot{number}] safety shield: enabled=True" not in log for number in (1, 2, 3))):
        raise ValueError(f"Could not confirm the shared actor and speed shield in {directory}")
    if {row["control_progress_mode"] for row in csv_rows(directory / "metrics.csv")} != {"observed_free_area"}:
        raise ValueError(f"Unexpected progress mode in {directory}")
    return dict(**run, **result, **audit_actions(directory), **audit_decisions(run, directory, log))


def trial(run, package, actor, rgb, output):
    name = f"{run['layout']}_{run['environment_seed']}_{run['rule_cue']}_{run['wait_mode']}"
    marker = output / f"{name}.complete.json"
    if marker.exists():
        directory = output / json.loads(marker.read_text())["directory"]
    else:
        for _ in range(2):
            check_idle()
            directory = Path(tempfile.mkdtemp(prefix=name + ".", dir=output))
            options = settings(run, package, actor, rgb, directory)
            preserve_json(directory / "inputs.json", dict(
                run=run, launch=options, actor_sha256=ACTOR_SHA256, rgb_sha256=RGB_SHA))
            print(f"START {name} {directory}", flush=True)
            status = run_launch(
                ["timeout", "--signal=INT", "--kill-after=25s", f"{6 * run['horizon']}s",
                 "roslaunch", "turtlebot_ddpg", "gazebo_research_experiment.launch", *options],
                directory / "launch.log", logs_environment(directory))
            (directory / "exit_status.txt").write_text(f"{status}\n")
            if not startup_segfault(directory):
                break
    record = verify(run, package, actor, rgb, directory)
    preserve_json(marker, dict(directory=directory.name, result=record))
    print(f"VERIFIED {name} coverage={record['coverage_pct']:.2f}% "
          f"reached={record['reached']}/{record['assignments']} "
          f"deferred={record['waits_deferred']} retired={record['waypoint_retires_received']}",
          flush=True)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("smoke", "formal"))
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("rgb", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, actor, rgb, output = (
        path.resolve() for path in (args.ros_project, args.actor, args.rgb, args.output))
    package = project / "turtlebot_ddpg"
    found = Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip())
    if found.resolve() != package or digest(actor) != ACTOR_SHA256 or digest(rgb) != RGB_SHA:
        raise ValueError("The prepared workspace, actor or RGB encoder differ")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if tree != SOURCE_TREE:
        raise ValueError("Prepare the audited MAPPO goal-retention source tree")
    blocks = BLOCKS if args.stage == "formal" else (("hu_aligned", 66520, (3, 5)),)
    runs = [dict(stage="eval", cue="lidar", layout=layout, environment_seed=seed,
                 rule_cue=OPTIONS[index][0], wait_mode=OPTIONS[index][1],
                 order=order, horizon=60 if args.stage == "formal" else 25)
            for layout, seed, indices in blocks for order, index in enumerate(indices, 1)]
    inputs = dict(runner=Path(__file__), actor=actor, rgb=rgb,
                  ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5",
                  paired_setup=Path(__file__).with_name("run_matched_vision_training.py"),
                  cue_audit=Path(__file__).with_name("run_rgb_clearance_guard_study.py"),
                  common_runner=Path(__file__).with_name("run_clearance_study.py"))
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        preserve_json(output / "experiment_manifest.json", dict(
            stage=args.stage, runs=runs, source_tree=tree, hold_seconds=HOLD_SECONDS,
            input_sha256={key: digest(path) for key, path in inputs.items()},
            world_sha256={layout: digest(package / "worlds" / (layout + "_arena.world"))
                          for layout, _, _ in blocks}))
        if args.audit_only and len(list(output.glob("*.complete.json"))) != len(runs):
            raise ValueError("The verified run set is incomplete")
        results = [trial(run, package, actor, rgb, output) for run in runs]
        for layout, seed, _ in blocks:
            if len({row["coverage_mask_sha256"] for row in results
                    if row["layout"] == layout and row["environment_seed"] == seed}) != 1:
                raise ValueError(f"The reference coverage mask differs in {layout} seed {seed}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
