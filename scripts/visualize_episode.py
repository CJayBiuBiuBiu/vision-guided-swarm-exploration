from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from eavf_mappo.envs.multi_robot_exploration_env import MultiRobotExplorationEnv
from eavf_mappo.evaluation.visualization import record_episode


def main() -> None:
    args = parse_args()
    env = MultiRobotExplorationEnv(
        {
            "env": {
                "n_robots": args.n_robots,
                "map_size_m": args.map_size,
                "resolution": args.resolution,
                "sensor_range_m": args.sensor_range,
                "sensor_n_rays": args.sensor_rays,
                "map_type": args.map_type,
                "max_episode_steps": args.episode_steps,
            },
            "high_level": {
                "frontier_K": args.frontier_k,
                "max_high_level_interval": args.max_high_level_interval,
            },
        }
    )
    output = Path(args.output)
    record_episode(
        env=env,
        policy_name=args.policy,
        max_steps=args.render_steps,
        output_path=output,
        fps=args.fps,
        seed=args.seed,
        checkpoint=args.checkpoint,
        frontier_k=args.frontier_k,
        frame_stride=args.frame_stride,
    )
    print(f"wrote {output}")


def parse_args():
    parser = argparse.ArgumentParser(description="Render an exploration episode to GIF.")
    parser.add_argument("--policy", default="nearest_frontier", choices=["nearest_frontier", "round_robin", "constant"])
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--episode-steps", type=_positive_int, default=200)
    parser.add_argument("--render-steps", "--max-steps", dest="render_steps", type=_positive_int, default=200)
    parser.add_argument("--frame-stride", type=_positive_int, default=1)
    parser.add_argument("--fps", type=int, default=6)
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--map-size", type=float, default=8.0)
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--sensor-range", type=float, default=1.5)
    parser.add_argument("--sensor-rays", type=int, default=36)
    parser.add_argument("--map-type", default="office", choices=["open_room", "corridor", "maze", "office", "cluttered"])
    parser.add_argument("--frontier-k", type=int, default=8)
    parser.add_argument("--max-high-level-interval", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", default="results/viz/episode.gif")
    return parser.parse_args()


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return parsed


if __name__ == "__main__":
    main()
