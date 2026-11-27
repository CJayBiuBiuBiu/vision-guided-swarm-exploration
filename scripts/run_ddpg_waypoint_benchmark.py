"""Compare frozen DDPG and near-goal tracking on matched scripted targets."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from xml.etree import ElementTree

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import csv_rows, write_csv
from run_rgb_passage_study import check_idle, logs_environment, stop_launch


SOURCE_TREE = "ca3c862a1de8ae16bf6bcff95d9586cb68129c81"
CASES = (
    ("left_025", 0.25, 90), ("right_025", 0.25, -90),
    ("left_060", 0.60, 90), ("right_060", 0.60, -90),
    ("left_rear_060", 0.60, 135), ("right_rear_060", 0.60, -135),
    ("near_forward", 0.35, 0), ("long_forward", 2.0, 0),
)
SCENES = (
    ("open_floor", 71701, ()),
    ("offset_posts", 71702, ((0.52, -0.61, 0.72, -0.42), (0.75, 0.48, 0.95, 0.69),
                             (-0.67, 0.65, -0.47, 0.85))),
)


def plans():
    for scene_index, (name, seed, obstacles) in enumerate(SCENES):
        cases = []
        for case_index, (case, distance, bearing) in enumerate(CASES):
            modes = ("ddpg", "near_goal") if (scene_index + case_index) % 2 == 0 else ("near_goal", "ddpg")
            for mode in modes:
                cases.append(dict(name=case, mode=mode, distance_m=distance, bearing_deg=bearing))
        yield dict(layout=name, gazebo_seed=seed, obstacles=[list(box) for box in obstacles],
                   horizon_s=12.0, tolerance_m=0.12, dwell_s=0.25, runs=cases)


def world_for(plan, output):
    from passage_scene import box, world_xml

    xml = ElementTree.fromstring(world_xml("passage_left_open"))
    world = xml.find("world")
    world.set("name", plan["layout"])
    for name in ("wall_0", "wall_1"):
        world.remove(world.find(f"./model[@name='{name}']"))
    for index, bounds in enumerate(plan["obstacles"]):
        world.append(ElementTree.fromstring(box(f"post_{index}", bounds, 0.8, 0.4, "0.27 0.31 0.34")))
    path = output / f"{plan['layout']}.world"
    content = ElementTree.tostring(xml, encoding="utf-8", xml_declaration=True)
    if path.exists():
        if path.read_bytes() != content:
            raise ValueError(f"World differs in {path}")
    else:
        with path.open("xb") as stream:
            stream.write(content)
    return path


def audit_case(plan, case, path):
    rows = csv_rows(path)

    def require(ok, reason):
        if not ok:
            raise ValueError(f"{reason} in {path}")

    require(rows and rows[-1]["terminal"] == "1" and
            all(row["terminal"] == "0" for row in rows[:-1]), "Incomplete waypoint trace")
    times = [float(row["elapsed_s"]) for row in rows]
    positions = [(float(row["x_m"]), float(row["y_m"])) for row in rows]
    require(times == sorted(times) and abs(times[0]) <= 0.01 and
            math.hypot(*positions[0]) <= 0.04 and abs(float(rows[0]["yaw_rad"])) <= 0.04,
            "Invalid time or starting pose")
    target = tuple(case["distance_m"] * unit(math.radians(case["bearing_deg"]))
                   for unit in (math.cos, math.sin))
    inside_since = None
    near, active = 0, False
    for row, time_s, position in zip(rows, times, positions):
        distance = math.dist(target, position)
        require(abs(distance - float(row["target_distance_m"])) <= 1e-6, "Target distance mismatch")
        if distance <= plan["tolerance_m"]:
            if inside_since is None:
                inside_since = time_s
        else:
            inside_since = None
        new_active = row["threshold_active"] == "1"
        near += new_active and not active
        active = new_active
    reached = inside_since is not None and times[-1] - inside_since >= plan["dwell_s"]
    require((rows[-1]["reached"] == "1") == reached and
            (reached or plan["horizon_s"] <= times[-1] < plan["horizon_s"] + 0.5),
            "Terminal result does not match the position trace")
    for row in rows[:-1]:
        source = row["tracker"]
        require(source in ("ddpg", "near_goal") and (source == "ddpg" or case["mode"] == "near_goal"),
                "Unexpected active tracker")
        require(0 <= float(row["linear_mps"]) <= (0.20 if source == "near_goal" else 0.26) + 1e-6
                and abs(float(row["angular_rps"])) <= (0.80 if source == "near_goal" else 1.0) + 1e-6,
                "Local command exceeded the declared bounds")
    return dict(layout=plan["layout"], gazebo_seed=plan["gazebo_seed"], **case,
                reached=int(reached), elapsed_s=times[-1], final_distance_m=float(rows[-1]["target_distance_m"]),
                path_m=sum(math.dist(a, b) for a, b in zip(positions, positions[1:])),
                laser_threshold_events=near, min_laser_m=min(float(row["min_laser_m"]) for row in rows),
                near_goal_steps=sum(row["tracker"] == "near_goal" for row in rows))


def capture(plan, plan_path, package, world, output, audit_only):
    directory = output / plan["layout"]
    directory.mkdir(exist_ok=True)
    case_paths = [directory / f"{case['name']}_{case['mode']}.csv" for case in plan["runs"]]
    missing = [path for path in case_paths if not path.is_file()]
    if missing:
        if audit_only:
            raise ValueError(f"Missing waypoint capture {missing[0]}")
        check_idle()
        attempt = Path(tempfile.mkdtemp(prefix=plan["layout"] + ".", dir=output))
        launch = ["roslaunch", "turtlebot_ddpg", "voronoi_multi_robot.launch", "gui:=false",
                  f"world_name:={world}", f"gazebo_seed:={plan['gazebo_seed']}", "robot1_x:=0",
                  "robot1_y:=0", "robot1_yaw:=0", "robot2_x:=-4.1", "robot2_y:=-4.1",
                  "robot3_x:=4.1", "robot3_y:=-4.1"]
        preserve_json(attempt / "inputs.json", dict(launch=launch, plan=plan["layout"]))
        env = logs_environment(attempt)
        with (attempt / "launch.log").open("xb") as log:
            process = subprocess.Popen(launch, env=env, start_new_session=True, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                with (attempt / "collector.log").open("xb") as collector:
                    subprocess.run([sys.executable, str(Path(__file__).with_name("collect_ddpg_waypoints.py")),
                                    str(plan_path), str(directory), str(package)],
                                   env=env, stdout=collector, stderr=subprocess.STDOUT, timeout=360, check=True)
                if process.poll() is not None:
                    raise ValueError(f"Gazebo exited during {plan['layout']}")
            finally:
                stop_launch(process)
    return [audit_case(plan, case, path) for case, path in zip(plan["runs"], case_paths)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, output = args.ros_project.resolve(), args.output.resolve()
    package = project / "turtlebot_ddpg"
    if Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()).resolve() != package:
        raise ValueError("Source the intended ROS workspace")
    if subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip() != SOURCE_TREE:
        raise ValueError("Use the benchmarked near-goal tracking source tree")
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    output.mkdir(parents=True, exist_ok=True)
    schedule = list(plans())
    offline = package / "scripts/fd_replay/play_human_data"
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = {case["layout"]: world_for(case, output) for case in schedule}
        preserve_json(output / "experiment_manifest.json", dict(
            source_tree=SOURCE_TREE, ddpg_sha256=digest(offline / "actormodel-450-500.h5"),
            world_sha256={key: digest(path) for key, path in worlds.items()}, schedule=schedule,
            scripts={name: digest(Path(__file__).with_name(name)) for name in (
                "run_ddpg_waypoint_benchmark.py", "collect_ddpg_waypoints.py")}))
        rows = []
        for plan in schedule:
            scene = plan["layout"]
            path = output / f"{scene}_plan.json"
            preserve_json(path, plan)
            rows.extend(capture(plan, path, package, worlds[scene], output, args.audit_only))
        write_csv(output / "summary.csv", rows)
        print(f"Verified {len(rows)} fixed-waypoint trials.")


if __name__ == "__main__":
    main()
