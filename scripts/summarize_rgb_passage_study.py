"""Extract doorway crossings and paired navigation effects from sealed trials."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

from run_matched_vision_training import csv_rows, write_csv
from run_rgb_passage_study import CONDITIONS


def read_outcomes(directory, result):
    from rgb_passage import MAX_BLOCKED_SCORE, MIN_OPEN_SCORE

    rows = csv_rows(directory / 'metrics.csv')
    crossings = {}
    for row in rows:
        for robot, (_, y) in json.loads(row['robot_positions_json']).items():
            if y < -0.25:
                crossings.setdefault(robot, float(row['elapsed_sim_s']))
    events = [json.loads(line) for line in (directory / 'decisions.jsonl').read_text().splitlines()]
    decisions = [event for event in events if event['event'] == 'passage_decision']
    assignments = [event for event in events if event['event'] == 'assigned']
    outcomes = {event['attempt_id']: event for event in events if event['event'] == 'outcome'}
    if (len(decisions) != len(assignments) or {row['attempt_id'] for row in assignments} != outcomes.keys()
            or events[-1]['event'] != 'run_end'):
        raise ValueError(f'Incomplete assignment outcomes in {directory}')
    blocked = eligible = 0
    changes = []
    for decision, assignment in zip(decisions, assignments):
        candidates = {candidate['frontier_id']: candidate for candidate in decision['candidates']}
        preferred = candidates.get(decision['preferred'])
        if preferred and preferred['score'] is not None and preferred['score']['far'] <= MAX_BLOCKED_SCORE:
            blocked += 1
            eligible += any(
                index != decision['preferred'] and candidate['score'] is not None
                and candidate['score']['far'] >= MIN_OPEN_SCORE
                for index, candidate in candidates.items())
        if decision['selected'] != decision['preferred']:
            elapsed = decision['elapsed_sim_s']
            old, new = preferred, candidates[decision['selected']]
            outcome = outcomes[assignment['attempt_id']]
            changes.append(dict(
                layout=result['layout'], seed=result['environment_seed'], robot=decision['robot'],
                attempt_id=assignment['attempt_id'], elapsed_s=elapsed,
                image_age_s=round(decision['control_stamp'] - decision['image_stamp'], 3),
                original_frontier=decision['preferred'], selected_frontier=decision['selected'],
                original_far_score=old['score']['far'], selected_far_score=new['score']['far'],
                original_route_m=old['route_m'], selected_route_m=new['route_m'],
                outcome=outcome['outcome'], outcome_elapsed_s=outcome['elapsed_sim_s']))
    if len(changes) != result['applied_reranks']:
        raise ValueError(f'Applied decisions do not match the sealed count in {directory}')
    return dict(robots_crossed=len(crossings), first_crossing_s=min(crossings.values(), default=None),
                robot1_crossing_s=crossings.get('robot1'),
                robot2_crossing_s=crossings.get('robot2'),
                robot3_crossing_s=crossings.get('robot3'),
                blocked_preferred=blocked, blocked_with_open_candidate=eligible,
                ended_by_wait=sum(event['outcome'] == 'wait' for event in outcomes.values()),
                first_rerank_s=min((event['elapsed_s'] for event in changes), default=None),
                reranked_arrivals=sum(event['outcome'] == 'reached' for event in changes)), changes


def paired_results(runs):
    blocks = defaultdict(dict)
    for row in runs:
        by_condition = blocks[row['layout'], row['environment_seed']]
        if row['condition'] in by_condition:
            raise ValueError('Duplicate run in study')
        by_condition[row['condition']] = row
    paired = []
    for (layout, seed), conditions in sorted(blocks.items()):
        if conditions.keys() != CONDITIONS.keys():
            raise ValueError(f'Incomplete block in {layout} seed {seed}')
        passage = conditions['passage']
        for baseline_name in ('lidar', 'fusion'):
            baseline = conditions[baseline_name]
            difference = dict(
                layout=layout, seed=seed, baseline=baseline_name,
                applied_reranks=passage['applied_reranks'],
                coverage_delta_pp=round(passage['coverage_pct'] - baseline['coverage_pct'], 4),
                path_delta_m=round(passage['path_m'] - baseline['path_m'], 4),
                near_collision_delta=passage['near_collision_events'] - baseline['near_collision_events'],
                stuck_delta=passage['stuck_events'] - baseline['stuck_events'],
                recovery_success_delta=passage['recovery_successes'] - baseline['recovery_successes'],
                crossed_delta=passage['robots_crossed'] - baseline['robots_crossed'])
            for name in ('first_crossing_s', 'time_to_90_s', 'time_to_97_s'):
                a, b = passage[name], baseline[name]
                difference[f'passage_{name}'] = a
                difference[f'baseline_{name}'] = b
                difference[f'{name}_delta'] = round(a - b, 3) if a is not None and b is not None else None
            paired.append(difference)
    return paired


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ros_project', type=Path)
    parser.add_argument('study', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str((args.ros_project / 'turtlebot_ddpg/scripts/voronoi_exploration').resolve()))

    manifest = json.loads((args.study / 'experiment_manifest.json').read_text())
    if manifest['stage'] not in ('pilot', 'smoke'):
        raise ValueError('Supply a matched navigation experiment')
    runs, reranks = [], []
    for run in manifest['runs']:
        name = f"{run['layout']}_{run['environment_seed']}_{run['condition']}"
        sealed = json.loads((args.study / f'{name}.complete.json').read_text())
        result = sealed['result']
        if any(result[key] != value for key, value in run.items()):
            raise ValueError(f'Sealed result does not match the manifest for {name}')
        metrics, changes = read_outcomes(args.study / sealed['directory'], result)
        runs.append(dict(**result, **metrics))
        reranks.extend(changes)
    args.destination.mkdir(parents=True, exist_ok=True)
    write_csv(args.destination / 'navigation.csv', runs)
    write_csv(args.destination / 'paired_differences.csv', paired_results(runs))
    if reranks:
        write_csv(args.destination / 'applied_reranks.csv', reranks)


if __name__ == '__main__':
    main()
