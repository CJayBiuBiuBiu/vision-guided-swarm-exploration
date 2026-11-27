"""Probe the frozen DDPG actor over counterfactual goal distances and bearings."""

import argparse
from pathlib import Path
import sys

import numpy as np
from scipy.io import loadmat

from run_clearance_study import digest, preserve_json
from run_matched_vision_training import write_csv


DISTANCES = (0.20, 0.35, 0.60, 1.00, 2.50, 11.50)
BEARINGS = (0, 30, 60, 90, 120, 150)


def states_for(original, distance, bearing):
    states = np.vstack((original, original))
    states[:, -4] = distance
    states[:len(original), -3] = np.deg2rad(bearing)
    states[len(original):, -3] = -np.deg2rad(bearing)
    return states


def probe(model, cases):
    for name, base in cases:
        for distance in DISTANCES:
            for bearing in BEARINGS:
                states = states_for(base, distance, bearing)
                commands = model.predict(states, batch_size=256, verbose=0)
                angular = commands[:, 0]
                speed = commands[:, 1] * 0.26
                yield dict(input_group=name, distance_m=distance, absolute_bearing_deg=bearing,
                           paired_inputs=len(states), turning_away=int(np.sum(angular * states[:, -3] < 0))
                           if bearing else None, median_linear_mps=float(np.median(speed)),
                           median_absolute_turn_rps=float(np.median(np.abs(angular))),
                           speed_below_005=int(np.sum(speed < 0.05)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_project / "turtlebot_ddpg/scripts/voronoi_exploration"))
    from multi_robot_explorer import K, WEIGHTS_PATH, build_actor_model, tf

    weights = Path(WEIGHTS_PATH)
    demonstrations = weights.parent / "human_data_1000.mat"
    original = loadmat(demonstrations)["data"][:, :28].astype(np.float32)
    finite = np.flatnonzero(np.isfinite(original).all(axis=1))
    indices = finite[::max(1, len(finite) // 150)][:150]
    clear = np.ones((1, 28), dtype=np.float32)
    clear[:, -4:] = 0.0
    args.output.mkdir(parents=True, exist_ok=False)
    preserve_json(args.output / "inputs.json", dict(
        ddpg_sha256=digest(weights), demonstrations_sha256=digest(demonstrations),
        demonstration_rows=len(original), demonstration_row_indices=indices.tolist(),
        demonstration_distances_below_1m=int(np.sum(original[:, -4] < 1.0)),
        median_demonstration_distance_m=float(np.median(original[:, -4])),
        distances_m=DISTANCES, absolute_bearings_deg=BEARINGS))
    session = tf.Session()
    K.set_session(session)
    try:
        model = build_actor_model()
        model.load_weights(str(weights))
        write_csv(args.output / "actor_response.csv", list(probe(
            model, (("archived_laser", original[indices]), ("uniform_clear_laser", clear)))))
    finally:
        session.close()


if __name__ == "__main__":
    main()
