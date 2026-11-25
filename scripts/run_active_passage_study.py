"""Compare passive and active projected-width guidance on new Gazebo layouts."""

import argparse
from collections import Counter
import fcntl
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_dynamic_passage_study import (
    ARENA, create_world, crossing_audit, inputs as passive_inputs, passage_audit,
)
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_wait_margin_study import ACTOR_SHA256, RGB_SHA, ROBOTS, audit_rechecks, node_log, verify_actions


SOURCE_TREE = "13dacd0b3b00f0cf1ed6b67ce192350fb8a856e0"
STRATEGIES = ("passive", "active")
SMOKE = (("inspection_smoke", 71211, 0.35, (-1.42, 1.31), (0.24, 1.24), None,
          4.0, -1.58, ()),)
FORMAL = (
    ("near_wide_east", 71221, 0.29, (-1.82, 1.06), (0.23, 1.26), None,
     4.03, -1.49, ((-0.72, -3.8, -0.10, -2.55),)),
    ("far_wide_west", 71222, -0.52, (-1.15, 1.94), (1.15, 0.26), None,
     4.15, -1.64, ((0.72, -3.7, 1.24, -2.05),)),
    ("west_exit_obstructed", 71223, 0.48, (-1.50, 1.88), (0.91, 1.16), 0,
     4.35, -1.55, ((2.55, -3.4, 3.2, -2.1),)),
    ("east_exit_obstructed", 71224, -0.23, (-1.93, 1.23), (1.26, 0.89), 1,
     4.24, -1.62, ((-3.5, -2.5, -2.9, -1.4), (0.5, -4.1, 1.3, -3.5))),
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
        for order, strategy in enumerate(STRATEGIES if index % 2 == 0 else STRATEGIES[::-1], 1):
            runs.append(dict(stage="eval", scene=name, environment_seed=seed, layout="passage_audit",
                             order=order, cue="lidar", rule_cue="lidar", strategy="margin",
                             wait_mode="cancel", passage_mode="rerank", passage_strategy=strategy,
                             horizon=45 if stage == "smoke" else 120))
    return runs, geometries


def inputs(run, package, actor, rgb, files, geometry, directory):
    record = passive_inputs(run, package, actor, rgb, files, geometry, directory)
    record["launch"].append(f"rgb_passage_strategy:={run['passage_strategy']}")
    return record


def audit_active(run, events, directory):
    if events[0]["rgb_passage_strategy"] != run["passage_strategy"]:
        raise ValueError(f"Active strategy differs in {directory}")
    active = run["passage_strategy"] == "active"
    scans, choices, inflight = {}, {}, {}
    passive_events, counts = [events[0]], Counter()
    results = {event["attempt_id"]: event["outcome"] for event in events if event["event"] == "outcome"}
    for event in events[1:]:
        kind, robot = event["event"], event.get("robot")
        if kind == "passage_width_decision":
            if robot in choices:
                raise ValueError(f"Unassigned prior choice in {directory}")
            choices[robot] = event
        elif kind == "passage_inspection_start":
            if not active or robot in scans or robot in choices:
                raise ValueError(f"Unexpected concurrent inspection in {directory}")
            if event["preferred_score"]["bottleneck_m"] is None or len(event["image_stamps"]) != 2:
                raise ValueError(f"Unconfirmed bottleneck in {directory}")
            scans[robot] = event
            counts["inspections_started"] += 1
        elif kind == "passage_inspection_result":
            start = scans.pop(robot)
            duration = event["control_stamp"] - start["control_stamp"]
            stamps, score = event["image_stamps"], event["score"]
            if (not 0 <= duration <= 5.0 or abs(event["duration_s"] - duration) > 1e-6
                    or (event["preferred"], event["alternative"]) != (start["preferred"], start["alternative"])
                    or stamps != sorted(set(stamps))
                    or any(stamp < start["control_stamp"] or not 0 <= event["control_stamp"] - stamp <= .400001
                           for stamp in stamps)):
                raise ValueError(f"Inspection timing or source differs in {directory}")
            for coordinate, anchor in (("preferred_xy", "preferred_xy"), ("alternative_xy", "target_xy")):
                if event[coordinate] is not None and math.dist(event[coordinate], start[anchor]) > 0.600001:
                    raise ValueError(f"Rebound frontier exceeds the spatial gate in {directory}")
            counts["inspections_finished"] += 1
            counts["inspections_scored"] += score is not None
            counts["inspection_rebound"] += (
                event["resolved_preferred"] not in (None, start["preferred"])
                or event["resolved_alternative"] not in (None, start["alternative"]))
            selected = event["selected"]
            if event["reason"] == "redirected":
                if (selected != event["resolved_alternative"] or len(stamps) != 2 or score is None
                        or score["clear_to_m"] < start["preferred_score"]["clear_to_m"] + 0.75
                        or event["alternative_route_m"] > event["preferred_route_m"] + 1.200001):
                    raise ValueError(f"Unjustified inspection redirect in {directory}")
                counts["inspection_redirects"] += 1
            elif selected is not None and selected != event["resolved_preferred"]:
                raise ValueError(f"Unexpected fallback goal in {directory}")
            if selected is not None:
                choices[robot] = event
        elif kind == "passage_inspection_cancel":
            scans.pop(robot)
            counts["inspections_cancelled"] += 1
        elif kind == "assigned":
            choice = choices.pop(robot)
            if choice["selected"] != event["selected_frontier_id"]:
                raise ValueError(f"Choice and issued goal differ in {directory}")
            inflight[robot] = event["attempt_id"]
            if choice["event"] == "passage_width_decision":
                passive_events.extend((choice, event))
            elif choice["reason"] == "redirected":
                counts["inspection_redirects_reached"] += results[event["attempt_id"]] == "reached"
        elif kind == "passage_continuation_decision":
            if not active or robot not in inflight or not 0 <= event["step"] <= 2:
                raise ValueError(f"Unexpected continuation in {directory}")
            counts["continuation_checks"] += 1
            if event["step"] > 0:
                counts["continuation_segments_arrived"] += 1
            if event["target_xy"] is not None:
                length = math.dist(event["origin_xy"], event["target_xy"])
                if (event["step"] >= 2 or len(event["image_stamps"]) != 2 or event["score"] is None
                        or length > min(1.2, event["score"]["clear_to_m"] - 0.25) + 1e-6
                        or length <= 0.65):
                    raise ValueError(f"Unbounded or unsupported continuation in {directory}")
                counts["continuations_issued"] += 1
        elif kind == "passage_continuation_wait":
            if not active or robot not in inflight:
                raise ValueError(f"Unexpected continuation hold in {directory}")
            if event["keep"] and (not 0 <= event["age_s"] < 4.5 or event["clearance_m"] <= 0.25):
                raise ValueError(f"Unsafe continuation hold in {directory}")
            counts["continuation_waits_kept"] += event["keep"]
        elif kind == "outcome":
            inflight.pop(robot)
            passive_events.append(event)
    if choices or inflight or (scans and not active):
        raise ValueError(f"Incomplete decision/assignment trace in {directory}")
    counts["inspections_open_at_horizon"] = len(scans)
    passive = passage_audit(run, passive_events, directory)
    if not active and counts["inspections_started"]:
        raise ValueError(f"Inspection ran in passive condition in {directory}")
    return dict(**passive, **{key: counts[key] for key in (
        "inspections_started", "inspections_finished", "inspections_scored", "inspection_rebound", "inspection_redirects",
        "inspection_redirects_reached", "inspections_cancelled", "inspections_open_at_horizon",
        "continuation_checks", "continuations_issued", "continuation_segments_arrived",
        "continuation_waits_kept")})


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
    return dict(**run, **result, **audit_decisions(run, directory, logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **audit_active(run, events, directory), **crossing_audit(metrics, geometry))


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
          f"continuations={result['continuations_issued']}", flush=True)
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
        raise ValueError("Use the audited active-passage source tree")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = schedule(args.stage)
    dependencies = ("run_active_passage_study", "run_dynamic_passage_study", "run_wait_margin_study",
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
        for a, b in zip(results[::2], results[1::2]):
            if a["coverage_mask_sha256"] != b["coverage_mask_sha256"]:
                raise ValueError(f"Reference masks differ for {a['scene']}")
        write_csv(output / "summary.csv", results)


if __name__ == "__main__":
    main()
