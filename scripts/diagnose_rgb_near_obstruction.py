"""Test whether frozen RGB encoders react to close obstacles in known sectors."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
from xml.etree import ElementTree

import cv2
import torch

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import write_csv
from run_rgb_passage_study import capture_probe


CASES = {
    "clear": (),
    "physical_left_blocked": ("right",),
    "centre_blocked": ("centre",),
    "physical_right_blocked": ("left",),
    "centre_open": ("right", "left"),
}
RUNTIME_X = {"left": -0.42, "centre": 0.0, "right": 0.42}
SECTORS = ("left", "centre", "right")
MAX_LOW_CLEARANCE = 0.38
MIN_OPEN_CLEARANCE = 0.85


def make_world(case):
    from passage_scene import box, world_xml

    root = ElementTree.fromstring(world_xml("passage_left_open"))
    world = root.find("world")
    world.set("name", case)
    for index in (0, 1):
        world.remove(world.find(f'model[@name="wall_{index}"]'))
    for sector in CASES[case]:
        x = RUNTIME_X[sector]
        bounds = (x - 0.18, 3.03, x + 0.18, 3.27)
        world.append(ElementTree.fromstring(box(
            f"block_{sector}", bounds, 0.8, 0.4, "0.26 0.29 0.34")))
    return ElementTree.tostring(root, encoding="unicode") + "\n"


def capture_cases(output, models):
    from directional_clearance import lidar_openness

    rows, inputs = [], {}
    for case, blocked in CASES.items():
        sdf = output / f"{case}.world"
        source = make_world(case)
        if not sdf.exists():
            with sdf.open("x") as stream:
                stream.write(source)
        if sdf.read_text() != source:
            raise ValueError(f"Case geometry has changed: {sdf}")
        directory = capture_probe(case, sdf, output)
        frame = directory / "rgb.png"
        sensor = directory / "sensors.json"
        record = json.loads(sensor.read_text())
        ranges = [float("inf") if distance is None else distance for distance in record["scan_ranges_m"]]
        lidar = lidar_openness(ranges, record["scan_angle_min_rad"], record["scan_step_rad"],
                               record["range_min_m"], record["range_max_m"], 1.085595)
        reference = dict(zip(SECTORS, lidar))
        if (any(reference[sector] > MAX_LOW_CLEARANCE for sector in blocked)
                or any(reference[sector] < MIN_OPEN_CLEARANCE for sector in reference if sector not in blocked)):
            raise ValueError(f"The LiDAR check did not confirm the defined scene for {case}: {reference}")
        image = cv2.cvtColor(cv2.imread(str(frame)), cv2.COLOR_BGR2RGB)
        scores = {model: scorer.score(image) for model, scorer in models.items()}
        for index, sector in enumerate(SECTORS):
            rows.append(dict(case=case, runtime_sector=sector, expected_blocked=sector in blocked,
                             lidar=reference[sector],
                             **{model: values[index] for model, values in scores.items()}))
        inputs[case] = dict(capture=directory.name, world_sha256=digest(sdf),
                            image_sha256=digest(frame), sensors_sha256=digest(sensor))
    return rows, inputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("exports", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    from run_rgb_passage_study import PROBE_SEED
    modules = str((args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration").resolve())
    sys.path.insert(0, modules)
    os.environ["PYTHONPATH"] = os.pathsep.join(
        value for value in (modules, os.environ.get("PYTHONPATH")) if value)
    from learned_clearance import LearnedVisionScorer

    legacy = Path(__file__).resolve().parents[1] / "ros_gazebo/checkpoints/rgb_clearance_inference.pt"
    paths = dict(legacy=legacy, robust_aligned=args.exports / "robust_aligned.pt")
    torch.set_num_threads(1)
    models = {name: LearnedVisionScorer(path) for name, path in paths.items()}
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / ".vision_gazebo_experiment.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows, inputs = capture_cases(output, models)
        preserve_json(output / "experiment_manifest.json", dict(
            probe_seed=PROBE_SEED, cases=inputs,
            checkpoint_sha256={name: digest(path) for name, path in paths.items()},
            script_sha256=digest(Path(__file__))))
        write_csv(output / "obstruction_scores.csv", rows)
        print("Verified all physical obstruction cases and frozen RGB outputs", flush=True)


if __name__ == "__main__":
    main()
