"""Evaluate live RGB corridor-width guidance on unseen Gazebo layouts."""

import argparse
from collections import Counter
import fcntl
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from xml.etree import ElementTree as ET

from run_clearance_study import digest, preserve_json, run_launch, startup_segfault
from run_matched_vision_training import csv_rows, verify_metrics, write_csv
from run_policy_goal_retention import audit_decisions, read_json_lines
from run_rgb_passage_study import check_idle, logs_environment
from run_wait_margin_study import (
    ACTOR_SHA256, RGB_SHA, ROBOTS, audit_rechecks, launch_arguments, node_log, verify_actions,
)


SOURCE_TREE = "74d3a374a5963516344088ce4bca6e9016519906"
ARENA = (-4.8, -5.0, 4.8, 5.0)
MODES = ("observe", "rerank")
SMOKE = (("smoke_offset", 71010, 0.50, (-1.50, 1.55), (1.45, 0.25), None,
          4.30, -1.5708, ()),)
FORMAL = (
    ("offset_narrow_east", 71111, 0.18, (-1.58, 1.27), (1.23, 0.24), None,
     4.18, -1.56, ((0.02, -3.5, 0.7, -2.4),)),
    ("south_narrow_west", 71112, -0.43, (-1.28, 1.68), (0.22, 0.94), None,
     4.05, -1.65, ((-0.2, -3.4, 0.3, -1.8), (-3.7, -3.8, -2.9, -3.1))),
    ("east_blocked_exit", 71113, 0.62, (-1.62, 1.30), (1.12, 0.88), 1,
     4.45, -1.56, ((-3.5, -3.5, -2.9, -2.2),)),
    ("west_blocked_exit", 71114, -0.14, (-1.40, 1.70), (0.82, 1.25), 0,
     4.22, -1.61, ((0.25, -3.5, 0.80, -2.6), (2.7, -1.9, 3.7, -1.4))),
)


def schedule(stage):
    scenes = SMOKE if stage == "smoke" else FORMAL
    runs = []
    geometries = {}
    for index, (name, seed, wall_y, centres, widths, blocked, spawn_y, yaw, extras) in enumerate(scenes):
        openings = [(center - width / 2, center + width / 2)
                    for center, width in zip(centres, widths)]
        ends = [ARENA[0], *[x for gap in openings for x in gap], ARENA[2]]
        walls = [(ends[i], wall_y - 0.10, ends[i + 1], wall_y + 0.10)
                 for i in range(0, len(ends), 2)]
        if blocked is not None:
            left, right = openings[blocked]
            walls.append((left - 0.20, wall_y - 0.47, right + 0.20, wall_y - 0.29))
        geometries[name] = dict(arena=ARENA, obstacles=walls + list(extras),
                                openings=openings, wall_y=wall_y, blocked_exit=blocked,
                                spawn_y=spawn_y, spawn_yaw=yaw)
        modes = MODES if index % 2 == 0 else MODES[::-1]
        for order, mode in enumerate(modes, 1):
            runs.append(dict(stage="eval", scene=name, environment_seed=seed, layout="passage_audit",
                             order=order, cue="lidar", rule_cue="lidar", strategy="margin",
                             wait_mode="cancel", passage_mode=mode, horizon=40 if stage == "smoke" else 90))
    return runs, geometries


def create_world(name, geometry, directory):
    from passage_scene import box, world_xml

    root = ET.fromstring(world_xml("passage_left_open"))
    world = root.find("world")
    world.set("name", name)
    for wall in ("wall_0", "wall_1"):
        world.remove(world.find(f"./model[@name='{wall}']"))
    for i, bounds in enumerate(geometry["obstacles"]):
        world.append(ET.fromstring(box(f"interior_{i}", bounds, 0.8, 0.4, "0.26 0.29 0.34")))
    path = directory / f"{name}.world"
    content = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    if path.exists():
        if path.read_bytes() != content:
            raise ValueError(f"World changed in {path}")
    else:
        with path.open("xb") as stream:
            stream.write(content)
    reference = directory / f"{name}_reference.json"
    preserve_json(reference, dict(arena=list(ARENA),
                                  obstacles=[list(bounds) for bounds in geometry["obstacles"]]))
    return path, reference


def inputs(run, package, actor, rgb, files, geometry, directory):
    shared = launch_arguments(run, package, actor, rgb, directory)
    overrides = [
        f"world_name:={files[0]}", f"reference_obstacles_path:={files[1]}",
        f"rgb_passage_mode:={run['passage_mode']}", "rgb_passage_geometry:=projected_width",
        "tail_completion_enabled:=false",
        *[f"robot{i}_{axis}:={value}"
          for i, x in enumerate((-1.65, 0.0, 1.65), 1)
          for axis, value in (("x", x), ("y", geometry["spawn_y"]), ("yaw", geometry["spawn_yaw"]))]
    ]
    keys = {option.split(":=", 1)[0] for option in overrides}
    launch = [option for option in shared if option.split(":=", 1)[0] not in keys] + overrides
    return dict(run=run, launch=launch, actor_sha256=ACTOR_SHA256, rgb_sha256=RGB_SHA)


def passage_audit(run, events, directory):
    if ((events[0]["rgb_passage_geometry"], events[0]["rgb_passage_mode"],
         events[0]["reference_mask_override"]) != ("projected_width", run["passage_mode"], True)):
        raise ValueError(f"Runtime passage or reference mask configuration differs in {directory}")
    selections = [event for event in events if event["event"] in ("assigned", "passage_width_decision")]
    if not selections or len(selections) % 2:
        raise ValueError(f"Selection trace is incomplete in {directory}")
    reached = {item["attempt_id"] for item in events
               if item["event"] == "outcome" and item["outcome"] == "reached"}
    counts = Counter()
    for choice, assigned in zip(selections[::2], selections[1::2]):
        if ((choice["event"], assigned["event"]) != ("passage_width_decision", "assigned")
                or choice["robot"] != assigned["robot"]
                or choice["selected"] != assigned["selected_frontier_id"]):
            raise ValueError(f"Passage choice differs from issued target in {directory}")
        times = choice["image_stamps"]
        if (len(times) > 2 or times != sorted(set(times))
                or any(not 0 <= choice["control_stamp"] - stamp <= 0.400001 for stamp in times)):
            raise ValueError(f"Fresh distinct image timestamps required in {directory}")
        candidates = {item["frontier_id"]: item for item in choice["candidates"]}
        preferred = candidates[choice["preferred"]]
        base = preferred["score"]
        options = {}
        if len(times) == 2 and preferred["route_m"] is not None and base and base["bottleneck_m"] is not None:
            for fid, item in candidates.items():
                score = item["score"]
                if (score is not None and item["route_m"] is not None
                        and score["clear_to_m"] >= base["clear_to_m"] + 0.75
                        and item["route_m"] <= preferred["route_m"] + 0.600000001):
                    options[fid] = (-score["clear_to_m"], item["route_m"])
        proposed = choice["would_select"]
        expected = (proposed == choice["preferred"] if not options
                    else proposed in options and options[proposed] == min(options.values()))
        applied = proposed if run["passage_mode"] == "rerank" else choice["preferred"]
        if not expected or applied != choice["selected"]:
            raise ValueError(f"Failed to reconstruct passage decision in {directory}")
        counts["passage_decisions"] += 1
        counts["fresh_frame_pairs"] += len(times) == 2
        counts["preferred_scored"] += base is not None
        counts["any_direction_scored"] += any(item["score"] is not None for item in candidates.values())
        counts["preferred_bottleneck"] += base is not None and base["bottleneck_m"] is not None
        counts["would_rerank"] += proposed != choice["preferred"]
        counts["applied_reranks"] += applied != choice["preferred"]
        counts["reranked_targets_reached"] += (applied != choice["preferred"]
                                               and assigned["attempt_id"] in reached)
    if not counts["fresh_frame_pairs"]:
        raise ValueError(f"No fresh paired camera evidence in {directory}")
    return {key: counts[key] for key in (
        "passage_decisions", "fresh_frame_pairs", "preferred_scored", "any_direction_scored",
        "preferred_bottleneck",
        "would_rerank", "applied_reranks", "reranked_targets_reached")}


def crossing_audit(metrics, geometry):
    """Use held-out world geometry only after the rollout to score crossings."""
    wall = geometry["wall_y"]
    candidates, crossed, previous = {}, {}, {}
    for row in metrics:
        time_s = float(row["elapsed_sim_s"])
        coverage = float(row["coverage_pct"])
        for robot, (x, y) in json.loads(row["robot_positions_json"]).items():
            if robot in previous:
                px, py = previous[robot]
                if py >= wall > y:
                    wall_x = px + (x - px) * (py - wall) / (py - y)
                    for gate, (left, right) in enumerate(geometry["openings"]):
                        if left <= wall_x <= right:
                            candidates[robot] = (gate, time_s, coverage)
                elif y > wall + 0.10:
                    candidates.pop(robot, None)
            if y <= wall - 0.55 and robot in candidates and robot not in crossed:
                gate, _, _ = candidates[robot]
                crossed[robot] = (gate, time_s, coverage)
            previous[robot] = (x, y)
    first = min(crossed.values(), key=lambda item: item[1]) if crossed else None
    return dict(robots_crossing=len(crossed), first_crossing_s=first[1] if first else None,
                coverage_at_crossing_pct=first[2] if first else None,
                post_crossing_gain_pp=float(metrics[-1]["coverage_pct"]) - first[2] if first else None,
                crossing_gates_json=json.dumps({name: gate for name, (gate, _, _) in crossed.items()},
                                               sort_keys=True))


def verify(run, package, actor, rgb, files, geometry, directory):
    if json.loads((directory / "inputs.json").read_text()) != inputs(
            run, package, actor, rgb, files, geometry, directory):
        raise ValueError(f"Launch inputs differ in {directory}")
    result = verify_metrics(directory, run, rules=run["rule_cue"])
    metrics = csv_rows(directory / "metrics.csv")
    events = read_json_lines(directory / "decisions.jsonl")
    logs = {name: node_log(directory, name + "_explorer") for name in ROBOTS}
    actor_log = node_log(directory, "mappo_policy")
    if (f"loaded mappo actor checkpoint {actor}" not in actor_log
            or "reference coverage omitted from actor input" not in actor_log
            or any(f"[{name}] safety shield: enabled=True" not in log for name, log in logs.items())
            or {row["control_progress_mode"] for row in metrics} != {"observed_free_area"}
            or any(int(row["tail_completion_activations"]) for row in metrics)):
        raise ValueError(f"Shared actor, shield or control information differs in {directory}")
    actions = verify_actions(run, directory)
    return dict(**run, **result, **audit_decisions(run, directory, logs), **actions,
                **audit_rechecks(run, directory, actions["recheck_requests"]),
                **passage_audit(run, events, directory), **crossing_audit(metrics, geometry))


def trial(run, package, actor, rgb, files, geometry, output, audit_only):
    name = f"{run['scene']}_{run['environment_seed']}_{run['passage_mode']}"
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
                result = verify(run, package, actor, rgb, files, geometry, directory)
                preserve_json(marker, dict(directory=directory.name, result=result))
                print(f"RECOVERED {name} {directory}", flush=True)
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
        command = "COMPLETED"
    result = verify(run, package, actor, rgb, files, geometry, directory)
    preserve_json(marker, dict(directory=directory.name, result=result))
    print(f"{command} {name} coverage={result['coverage_pct']:.2f}% "
          f"crossings={result['robots_crossing']} reranked={result['applied_reranks']}/"
          f"{result['would_rerank']} fresh={result['fresh_frame_pairs']}/"
          f"{result['passage_decisions']}", flush=True)
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
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--", "turtlebot_ddpg",
                    "turtlebot3_description", "laser_filters"], check=True)
    if (Path(found).resolve() != package or tree != SOURCE_TREE or digest(actor) != ACTOR_SHA256
            or digest(rgb) != RGB_SHA):
        raise ValueError("Prepared workspace or shared actor/encoder differs from the study")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = schedule(args.stage)
    names = ("run_dynamic_passage_study", "run_wait_margin_study", "run_policy_goal_retention",
             "run_matched_vision_training", "run_clearance_study", "run_rgb_clearance_guard_study",
             "run_rgb_passage_study")
    sources = {name: Path(__file__).with_name(name + ".py") for name in names}
    sources.update(actor=actor, rgb=rgb,
                   ddpg=package / "scripts/fd_replay/play_human_data/actormodel-450-500.h5")
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
