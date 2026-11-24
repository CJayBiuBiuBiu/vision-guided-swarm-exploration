"""Check the RGB and LiDAR side convention in stationary mirrored scenes."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import sys
from xml.etree import ElementTree

import cv2
import torch

from audit_rgb_encoder import SECTORS, load_training_checkpoint
from run_clearance_study import digest, preserve_json
from run_matched_vision_training import write_csv
from run_rgb_passage_study import capture_probe


NEAR_PARTITION_Y = 1.35
CAMERA_FOV_RAD = 1.085595


def shifted_world(layout, xml):
    root = ElementTree.fromstring(xml)
    world = root.find('world')
    world.set('name', f'{layout}_near')
    for name in ('wall_0', 'wall_1'):
        pose = world.find(f'model[@name="{name}"]/pose')
        coordinates = pose.text.split()
        coordinates[1] = str(float(coordinates[1]) + NEAR_PARTITION_Y)
        pose.text = ' '.join(coordinates)
    return ElementTree.tostring(root, encoding='unicode') + '\n'


def measure(layout, record, image, scorers, distance_label, partition_y):
    from directional_clearance import lidar_openness
    from passage_scene import DOOR_X
    from vision_scoring import directional_openness

    ranges = [float('inf') if value is None else value for value in record['scan_ranges_m']]
    scores = lidar_openness(
        ranges, record['scan_angle_min_rad'], record['scan_step_rad'],
        record['range_min_m'], record['range_max_m'], CAMERA_FOV_RAD)
    references = {name: scorer.score(image) for name, scorer in scorers.items()}
    x, y, _ = record['position']
    qx, qy, qz, qw = record['quaternion']
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
    for physical_side, goal_x in (('left', DOOR_X), ('right', -DOOR_X)):
        bearing = math.atan2(partition_y-y, goal_x-x) - yaw
        sector = SECTORS[2 if bearing > CAMERA_FOV_RAD/6 else 0]
        yield dict(
            layout=layout, partition=distance_label, physical_side=physical_side,
            open_side=layout == f'passage_{physical_side}_open', yaw_deg=math.degrees(yaw),
            goal_bearing_deg=math.degrees(bearing), runtime_sector_name=sector,
            lidar=directional_openness(scores, bearing, CAMERA_FOV_RAD),
            **{name: directional_openness(values, bearing, CAMERA_FOV_RAD)
               for name, values in references.items()},
            **{f'{name}_scores_named': json.dumps(values)
               for name, values in {'lidar': scores, **references}.items()},
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ros_project', type=Path)
    parser.add_argument('range_probe', type=Path,
                        help='The archived far-passage stationary probe directory')
    parser.add_argument('robust_checkpoint', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    from run_rgb_passage_study import PROBE_SEED
    modules = str((args.ros_project / 'turtlebot_ddpg/scripts/voronoi_exploration').resolve())
    sys.path.insert(0, modules)
    os.environ['PYTHONPATH'] = os.pathsep.join(
        value for value in (modules, os.environ.get('PYTHONPATH')) if value)
    from passage_scene import PASSAGE_PROFILES, world_xml

    checkpoint = Path(__file__).resolve().parents[1] / 'ros_gazebo/checkpoints/rgb_clearance_inference.pt'
    from learned_clearance import LearnedVisionScorer

    torch.set_num_threads(1)
    scorers = {name: LearnedVisionScorer(checkpoint) for name in ('legacy', 'robust')}
    robust = load_training_checkpoint(args.robust_checkpoint, 'robust')
    if tuple(robust['input_size'][::-1]) != scorers['robust'].size:
        raise ValueError('A different RGB input size requires its own preprocessing settings')
    scorers['robust'].model.load_state_dict(robust['model_state_dict'])
    published = Path(args.range_probe).resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / '.vision_gazebo_experiment.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows = []
        inputs = {}
        for layout in PASSAGE_PROFILES:
            case = f'{layout}_near'
            sdf = output / f'{case}.world'
            xml = shifted_world(layout, world_xml(layout))
            if not sdf.exists():
                with sdf.open('x') as stream:
                    stream.write(xml)
            if sdf.read_text() != xml:
                raise ValueError(f'World was edited at {sdf}')
            near = capture_probe(case, sdf, output)
            data = {label: (path, y) for label, path, y in (
                ('near', near, NEAR_PARTITION_Y),
                ('far', published / json.loads(
                    (published / f'probe_{layout}_{PROBE_SEED}.complete.json').read_text())['directory'], 0.0))}
            for label, (path, y) in data.items():
                sensors, frame = path / 'sensors.json', path / 'rgb.png'
                image = cv2.cvtColor(cv2.imread(str(frame)), cv2.COLOR_BGR2RGB)
                rows.extend(measure(layout, json.loads(sensors.read_text()),
                                    image, scorers, label, y))
                inputs[f'{layout}_{label}'] = dict(rgb_sha256=digest(frame),
                                                  sensor_sha256=digest(sensors))
            inputs[case]['world_sha256'] = digest(sdf)
        preserve_json(output / 'manifest.json', dict(
            probe_seed=PROBE_SEED, near_partition_y_m=NEAR_PARTITION_Y,
            legacy_sha256=digest(checkpoint), robust_sha256=digest(args.robust_checkpoint),
            script_sha256=digest(Path(__file__)), inputs=inputs))
        write_csv(output / 'geometry.csv', rows)
        print('Verified two close and two far stationary scenes', flush=True)


if __name__ == '__main__':
    main()
