"""Audit camera-visible doorway width and clearance immediately behind the opening."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from xml.etree import ElementTree as ET

import cv2
import numpy as np

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import write_csv
from run_rgb_passage_study import capture_probe


ROBOT_WIDTH_M = 0.266
SIDE_MARGIN_M = 0.05
WIDTH_LIMIT_M = ROBOT_WIDTH_M + 2 * SIDE_MARGIN_M
STEP_M = 0.025
MIN_FLOOR_SCORE = 0.80
GATE_NEAR_Y = 0.10
ENTRANCE_SAMPLE_Y = -0.14
EXIT_SAMPLE_Y = -0.78
CASE_TYPES = (("narrow", 0.22, False), ("tight", 0.46, False),
              ("wide", 0.90, False), ("reference", 1.50, False), ("blocked_exit", 0.90, True))


def cases():
    for kind, width, blocked in CASE_TYPES:
        for side, center in (("left", 1.50), ("right", -1.50)):
            yield dict(case=f"{side}_{kind}", side=side, center_x=center, kind=kind,
                       nominal_width_m=width, blocked_exit=blocked,
                       traversable=width >= WIDTH_LIMIT_M and not blocked)


def world_for(case):
    from passage_scene import ARENA, box, world_xml

    root = ET.fromstring(world_xml("passage_left_open"))
    world = root.find("world")
    world.set("name", case["case"])
    center, width = case["center_x"], case["nominal_width_m"]
    a, b = center - width / 2, center + width / 2
    for i, (x0, x1) in enumerate(((ARENA[0], a), (b, ARENA[2]))):
        wall = world.find(f"./model[@name='wall_{i}']")
        wall.find("pose").text = f"{(x0 + x1) / 2} 0 0.4 0 0 0"
        for size in wall.findall(".//size"):
            size.text = f"{x1 - x0} 0.2 0.8"
    if case["blocked_exit"]:
        world.append(ET.fromstring(box("exit_block", (a - 0.15, -0.53, b + 0.15, -0.31),
                                       0.8, 0.4, "0.26 0.29 0.34")))
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def projected_width(color, sensor, center):
    from depth_corridor import CAMERA_ORIGIN
    from rgb_passage import floor_samples

    intrinsics = sensor["intrinsics"]
    position, orientation = sensor["position"], sensor["quaternion"]
    pose = (position, orientation)
    reference, valid, yaw = floor_samples(
        color, intrinsics, pose, [0.0], [0.50, 0.60, 0.75], [-0.10, 0.0, 0.10])
    if not valid.all() or abs(yaw + math.pi / 2) > 0.01:
        raise ValueError("The aperture audit requires a surveyed, south-facing stationary camera")
    color_reference = np.median(reference, axis=(0, 1, 2))
    offsets = np.arange(-36, 37) * STEP_M
    camera_x = position[0] + CAMERA_ORIGIN[1]
    camera_y = position[1] - CAMERA_ORIGIN[0]
    widths = []
    for y in (ENTRANCE_SAMPLE_Y, EXIT_SAMPLE_Y):
        ratio = (camera_y - y) / (camera_y - GATE_NEAR_Y)
        sample_x = camera_x + (center + offsets - camera_x) * ratio
        colors, valid, _ = floor_samples(
            color, intrinsics, pose, [0.0], [position[1] - y], sample_x - position[0])
        likelihood = np.clip((48 - np.linalg.norm(colors[0, 0] - color_reference, axis=-1)) / 28,
                             0, 1)
        supported = valid[0, 0] & (likelihood >= MIN_FLOOR_SCORE)
        cursor = len(offsets) // 2
        if not supported[cursor]:
            widths.append(0.0)
            continue
        low = high = cursor
        while low > 0 and supported[low - 1]:
            low -= 1
        while high + 1 < len(offsets) and supported[high + 1]:
            high += 1
        widths.append(round((high - low + 1) * STEP_M, 3))
    return tuple(widths)


def measure(case, image, sensors):
    color_bgr = cv2.imread(str(image), cv2.IMREAD_COLOR)
    if color_bgr is None:
        raise ValueError(f"Cannot read {image}")
    sensor = json.loads(sensors.read_text())
    entrance, rear = projected_width(cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB),
                                     sensor, case["center_x"])
    old = sensor["scores"][case["side"]]
    old_open = old is not None and old["far"] >= 0.80
    bearing = math.atan2(case["center_x"] - sensor["position"][0], sensor["position"][1])
    beam = (bearing - sensor["scan_angle_min_rad"]) / sensor["scan_step_rad"]
    ranges = [sensor["scan_ranges_m"][i] for i in (math.floor(beam), math.ceil(beam))]
    no_return = all(value is None or value >= sensor["range_max_m"] - 0.015 for value in ranges)
    return dict(**case, old_far_score=None if old is None else old["far"], old_open=old_open,
                entry_width_m=entrance, rear_width_m=rear,
                width_rule_passable=min(entrance, rear) >= WIDTH_LIMIT_M,
                lidar_at_horizon=no_return, rgb_sha256=digest(image), sensors_sha256=digest(sensors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--archived-probes", type=Path,
                        help="Replay the two existing mirrored reference frames without Gazebo")
    args = parser.parse_args()
    package = args.ros_project.resolve() / "turtlebot_ddpg"
    modules = str(package / "scripts/voronoi_exploration")
    sys.path.insert(0, modules)
    os.environ["PYTHONPATH"] = os.pathsep.join(
        part for part in (modules, os.environ.get("PYTHONPATH")) if part)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    schedule = list(cases())
    if args.archived_probes:
        rows = []
        for case in schedule:
            if case["kind"] == "reference":
                base = args.archived_probes / f"passage_{case['side']}_open"
                rows.append(measure(case, base.with_suffix(".png"), base.with_suffix(".json")))
        write_csv(output / "summary.csv", rows)
        return
    found = Path(subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip())
    if found.resolve() != package:
        raise ValueError("Source the intended ROS workspace")
    tree = subprocess.check_output(["git", "-C", str(package), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(package), "diff", "--quiet", "--", "."], check=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = {}
        for case in schedule:
            path = output / (case["case"] + ".world")
            content = world_for(case)
            if path.exists():
                if path.read_bytes() != content:
                    raise ValueError(f"The test world changed in {path}")
            else:
                with path.open("xb") as stream:
                    stream.write(content)
            worlds[case["case"]] = path
        scripts = (Path(__file__), Path(__file__).with_name("capture_rgb_passage.py"),
                   Path(__file__).with_name("run_rgb_passage_study.py"))
        preserve_json(output / "experiment_manifest.json", dict(
            source_tree=tree, stationary_cases=schedule, measurement_plane_y=GATE_NEAR_Y,
            required_width_m=WIDTH_LIMIT_M, source_sha256={p.name: digest(p) for p in scripts},
            world_sha256={name: digest(path) for name, path in worlds.items()}))
        rows = []
        for case in schedule:
            name = case["case"]
            print(f"START {name}", flush=True)
            attempt = capture_probe(name, worlds[name], output)
            row = measure(case, attempt / "rgb.png", attempt / "sensors.json")
            rows.append(row)
            print(f"VERIFIED {name} width={row['entry_width_m']:.3f}/{row['rear_width_m']:.3f}m "
                  f"passable={row['width_rule_passable']} reference={case['traversable']}", flush=True)
        write_csv(output / "summary.csv", rows)


if __name__ == "__main__":
    main()
