"""Run and audit stationary and sequential paired trials in a sourced ROS workspace."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from run_clearance_study import (
    digest, preserve_json, run_launch, startup_segfault, stop_launch, verify_matched_cues)
from run_matched_vision_training import csv_rows, trial_configs, verify_metrics, write_csv


PROBE_SEED = 65390
CONDITIONS = {
    'lidar': ('lidar', 'observe'),
    'fusion': ('fusion', 'observe'),
    'passage': ('fusion', 'rerank'),
}
BLOCKS = (
    ('passage_left_open', 65401, ('lidar', 'fusion', 'passage')),
    ('passage_right_open', 65402, ('fusion', 'passage', 'lidar')),
    ('passage_right_open', 65403, ('passage', 'lidar', 'fusion')),
    ('passage_left_open', 65404, ('passage', 'fusion', 'lidar')),
)
HORIZON_S = 90
ACTOR_SHA256 = '24c3c2edf94593484cf36a596054cc0d0273f922e3039e2fbe5fdd4d45cff212'


def check_idle():
    for name in ('gzserver', 'gzclient', 'rosmaster', 'roslaunch'):
        if subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f'Another ROS or Gazebo process is running: {name}')


def starts():
    from passage_scene import STAGING_Y

    return [f'robot{index}_{key}:={value}'
            for index, x in enumerate((-3.0, 0.0, 3.0), 1)
            for key, value in (('x', x), ('y', STAGING_Y), ('yaw', '-1.57079632679'))]


def logs_environment(directory):
    logs = directory / 'ros_logs'
    logs.mkdir()
    return {**os.environ, 'ROS_LOG_DIR': str(logs)}


def capture_probe(world, sdf, output):
    name = f'probe_{world}_{PROBE_SEED}'
    marker = output / f'{name}.complete.json'
    if marker.exists():
        entry = json.loads(marker.read_text())
        attempt = output / entry['directory']
        preserve_json(marker, probe_report(attempt, sdf))
        return attempt
    check_idle()
    attempt = Path(tempfile.mkdtemp(prefix=name + '.', dir=output))
    launch = ['roslaunch', 'turtlebot_ddpg', 'voronoi_multi_robot.launch', 'gui:=false',
              f'world_name:={sdf}', f'gazebo_seed:={PROBE_SEED}', *starts()]
    preserve_json(attempt / 'inputs.json', dict(launch=launch, world_sha256=digest(sdf)))
    environment = logs_environment(attempt)
    with (attempt / 'launch.log').open('xb') as log:
        process = subprocess.Popen(launch, env=environment, start_new_session=True,
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        try:
            with (attempt / 'capture.log').open('xb') as capture:
                subprocess.run([sys.executable, str(Path(__file__).with_name('capture_rgb_passage.py')),
                                str(attempt)], env=environment, stdout=capture,
                               stderr=subprocess.STDOUT, timeout=110, check=True)
            if process.poll() is not None:
                raise ValueError(f'Gazebo exited during the probe in {attempt}')
        finally:
            stop_launch(process)
    preserve_json(marker, probe_report(attempt, sdf))
    return attempt


def probe_report(attempt, sdf):
    log = (attempt / 'launch.log').read_text(errors='replace')
    if re.search(r'Traceback|Segmentation fault|\[gazebo-\d+\] process has died', log):
        raise ValueError(f'Gazebo failed during stationary capture in {attempt}')
    return dict(directory=attempt.name, world_sha256=digest(sdf),
                rgb_sha256=digest(attempt / 'rgb.png'),
                sensors_sha256=digest(attempt / 'sensors.json'))


def compare_probes(probes, output):
    from rgb_passage import FAR_DISTANCES, MAX_BLOCKED_SCORE, MIN_OPEN_SCORE

    comparison = []
    for world, attempt in probes.items():
        record = json.loads((attempt / 'sensors.json').read_text())
        # Use the two raw beams bracketing each direction; a max-range beam has
        # no useful obstacle return within the sensor's configured horizon.
        if record['range_max_m'] >= min(FAR_DISTANCES) - 0.1:
            raise ValueError('Camera bands must be beyond the LiDAR horizon')
        for direction, score in record['scores'].items():
            if score is None:
                raise ValueError(f'No usable passage cue for {world} {direction}')
            beam = ((math.radians(score['bearing_deg']) - record['scan_angle_min_rad'])
                    / record['scan_step_rad'])
            indices = (math.floor(beam), math.ceil(beam))
            ranges = [record['scan_ranges_m'][index] for index in indices]
            no_return = all(distance is None or distance >= record['range_max_m'] - 0.015
                            for distance in ranges)
            opened = world == f'passage_{direction}_open'
            expected = score['far'] >= MIN_OPEN_SCORE if opened else score['far'] <= MAX_BLOCKED_SCORE
            comparison.append(dict(layout=world, direction=direction, opening=opened,
                                   far_score=score['far'], near_score=score['near'],
                                   angle_deg=score['bearing_deg'],
                                   lidar_range_max_m=record['range_max_m'],
                                   lidar_bracket_m=json.dumps(ranges), lidar_at_horizon=no_return,
                                   expected_camera_class=expected))
    write_csv(output / 'probe_comparison.csv', comparison)
    if not all(row['lidar_at_horizon'] and row['expected_camera_class'] for row in comparison):
        raise ValueError('Static evidence does not meet the predefined probe criteria')
    print('PROBE VERIFIED: both mirrored openings distinguished at the raw LiDAR horizon', flush=True)


def options_for(run, args, sdf, attempt, rgb):
    rules, mode = CONDITIONS[run['condition']]
    options = trial_configs(run, {'decision_interval_s': 1.5},
                            args.ros_project / 'turtlebot_ddpg', rgb, attempt, args.actor)
    override = [f'world_name:={sdf}', *starts(), f'clearance_rules_cue:={rules}',
                'control_progress_mode:=observed_free_area', f'rgb_passage_mode:={mode}',
                f'frontier_trace_path:={attempt / "decisions.jsonl"}']
    replaced = {option.split(':=', 1)[0] for option in override}
    return [option for option in options if option.split(':=', 1)[0] not in replaced] + override


def audit_trial(attempt, run, arguments, actor):
    from rgb_passage import MAX_BLOCKED_SCORE, MAX_EXTRA_ROUTE_M, MAX_IMAGE_AGE_S, MIN_OPEN_SCORE

    rules, mode = CONDITIONS[run['condition']]
    inputs = json.loads((attempt / 'inputs.json').read_text())
    if inputs != dict(run=run, arguments=arguments, actor_sha256=ACTOR_SHA256):
        raise ValueError(f'Trial inputs changed in {attempt}')
    result = verify_metrics(attempt, run, rules=rules)
    log = (attempt / 'launch.log').read_text(errors='replace')
    if (f'loaded mappo actor checkpoint {actor}' not in log
            or 'reference coverage omitted from actor input' not in log):
        raise ValueError(f'Frozen MAPPO actor or feature schema was not confirmed in {attempt}')
    events = [json.loads(line) for line in (attempt / 'decisions.jsonl').read_text().splitlines()]
    meta = events[0]
    if (meta.get('rgb_passage_mode'), meta.get('clearance_actor_cue'),
            meta.get('clearance_rules_cue')) != (mode, 'lidar', rules):
        raise ValueError(f'Trace configuration changed in {attempt}')
    traced = [event for event in events if event['event'] in ('passage_decision', 'assigned')]
    decisions, assignments = traced[::2], traced[1::2]
    if not decisions or len(decisions) != len(assignments):
        raise ValueError(f'Incomplete passage decision trace in {attempt}')
    overrides = proposals = supported = 0
    for decision, assignment in zip(decisions, assignments):
        if (decision['event'] != 'passage_decision' or assignment['event'] != 'assigned'
                or decision['robot'] != assignment['robot']
                or decision['selected'] != assignment['selected_frontier_id']):
            raise ValueError(f'Passage decision differs from assigned frontier in {attempt}')
        verify_matched_cues(assignment, 'lidar', rules, 0.5, attempt)
        by_id = {option['frontier_id']: option for option in decision['candidates']}
        owners = dict(zip(assignment['candidate_frontier_ids'], assignment['candidate_owners']))
        if any(owners.get(index) != decision['robot'] for index in by_id):
            raise ValueError(f'Passage candidate has a different owner in {attempt}')
        preferred, proposed, selected = (decision[key] for key in ('preferred', 'would_select', 'selected'))
        if selected != (proposed if mode == 'rerank' else preferred):
            raise ValueError(f'Passage mode applied the wrong frontier in {attempt}')
        if by_id:
            if not 0 <= decision['control_stamp'] - decision['image_stamp'] <= MAX_IMAGE_AGE_S:
                raise ValueError(f'Stale image influenced a frontier in {attempt}')
            supported += by_id[preferred]['score'] is not None
        if proposed != preferred:
            old, new = by_id[preferred], by_id[proposed]
            if (old['score']['far'] > MAX_BLOCKED_SCORE or new['score']['far'] < MIN_OPEN_SCORE
                    or new['route_m'] - old['route_m'] > MAX_EXTRA_ROUTE_M + 1e-9):
                raise ValueError(f'Invalid RGB rerank in {attempt}')
            proposals += 1
        overrides += selected != preferred
    return dict(**run, **result, assignments=len(assignments), preferred_scored=supported,
                proposed_reranks=proposals, applied_reranks=overrides)


def navigate(run, args, sdf, rgb):
    name = f"{run['layout']}_{run['environment_seed']}_{run['condition']}"
    marker = args.output / f'{name}.complete.json'
    if marker.exists():
        attempt = args.output / json.loads(marker.read_text())['directory']
    else:
        for _ in range(2):
            check_idle()
            attempt = Path(tempfile.mkdtemp(prefix=name + '.', dir=args.output))
            options = options_for(run, args, sdf, attempt, rgb)
            preserve_json(attempt / 'inputs.json', dict(run=run, arguments=options,
                                                       actor_sha256=ACTOR_SHA256))
            command = ['timeout', '--signal=INT', '--kill-after=25s', f"{6 * run['horizon']}s",
                       'roslaunch', 'turtlebot_ddpg', 'gazebo_research_experiment.launch', *options]
            print(f'START {name} {attempt}', flush=True)
            status = run_launch(command, attempt / 'launch.log', logs_environment(attempt))
            (attempt / 'exit_status.txt').write_text(f'{status}\n')
            if not startup_segfault(attempt):
                break
    options = options_for(run, args, sdf, attempt, rgb)
    result = audit_trial(attempt, run, options, args.actor)
    preserve_json(marker, dict(directory=attempt.name, result=result))
    print(f"VERIFIED {name} coverage={result['coverage_pct']:.2f}% "
          f"applied={result['applied_reranks']} assigned={result['assignments']}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('probe', 'smoke', 'pilot'))
    parser.add_argument('ros_project', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--actor', type=Path)
    args = parser.parse_args()
    args.ros_project, args.output = args.ros_project.resolve(), args.output.resolve()
    package = args.ros_project / 'turtlebot_ddpg'
    found = subprocess.check_output(['rospack', 'find', 'turtlebot_ddpg'], text=True).strip()
    if Path(found).resolve() != package:
        raise ValueError('Source the intended ROS workspace')
    modules = str(package / 'scripts/voronoi_exploration')
    sys.path.insert(0, modules)
    os.environ['PYTHONPATH'] = os.pathsep.join(
        part for part in (modules, os.environ.get('PYTHONPATH')) if part)
    from passage_scene import PASSAGE_PROFILES, world_xml

    files = dict(runner=Path(__file__), clearance_runner=Path(__file__).with_name('run_clearance_study.py'),
                 training_runner=Path(__file__).with_name('run_matched_vision_training.py'),
                 probe=Path(__file__).with_name('capture_rgb_passage.py'))
    tree = subprocess.check_output(['git', '-C', str(args.ros_project), 'write-tree'], text=True).strip()
    subprocess.run(['git', '-C', str(args.ros_project), 'diff', '--quiet', '--',
                    'turtlebot_ddpg', 'turtlebot3_description', 'laser_filters'], check=True)
    rgb = Path(__file__).resolve().parents[1] / 'ros_gazebo/checkpoints/rgb_clearance_inference.pt'
    if args.stage != 'probe':
        if args.actor is None or digest(args.actor) != ACTOR_SHA256:
            raise ValueError('Supply the frozen 32-update LiDAR MAPPO checkpoint for training seed 62600')
        args.actor = args.actor.resolve()
        files.update(rgb=rgb, actor=args.actor,
                     ddpg=package / 'scripts/fd_replay/play_human_data/actormodel-450-500.h5')
    args.output.mkdir(parents=True, exist_ok=True)
    with (Path.home() / '.vision_gazebo_experiment.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        worlds = {}
        for world in PASSAGE_PROFILES:
            sdf = args.output / f'{world}.world'
            xml = world_xml(world)
            if not sdf.exists():
                with sdf.open('x') as stream:
                    stream.write(xml)
            if sdf.read_text() != xml:
                raise ValueError(f'Scene geometry differs at {sdf}')
            worlds[world] = sdf
        runs = []
        if args.stage != 'probe':
            smoke = args.stage == 'smoke'
            blocks = [('passage_left_open', 65391, tuple(CONDITIONS))] if smoke else BLOCKS
            runs = [dict(stage='eval', cue='lidar', layout=world, environment_seed=seed,
                         condition=condition, order=order, horizon=40 if smoke else HORIZON_S)
                    for world, seed, conditions in blocks
                    for order, condition in enumerate(conditions, 1)]
        preserve_json(args.output / 'experiment_manifest.json', dict(
            stage=args.stage, ros_tree=tree, runs=runs,
            probe_seed=PROBE_SEED if args.stage == 'probe' else None,
            spawn=starts(), world_sha256={name: digest(path) for name, path in worlds.items()},
            input_sha256={name: digest(path) for name, path in files.items()}))
        if args.stage == 'probe':
            probes = {world: capture_probe(world, sdf, args.output) for world, sdf in worlds.items()}
            compare_probes(probes, args.output)
        else:
            results = [navigate(run, args, worlds[run['layout']], rgb) for run in runs]
            for world, seed in {(run['layout'], run['environment_seed']) for run in runs}:
                if len({row['coverage_mask_sha256'] for row in results
                        if row['layout'] == world and row['environment_seed'] == seed}) != 1:
                    raise ValueError(f'Coverage masks differ within {world} seed {seed}')
            write_csv(args.output / 'summary.csv', results)


if __name__ == '__main__':
    main()
