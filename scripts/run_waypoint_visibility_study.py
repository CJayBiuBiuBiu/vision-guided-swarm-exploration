"""Compare standard and scan-visible lookahead with frozen policy checkpoints."""

import argparse
import fcntl
from pathlib import Path
import subprocess
import sys

from run_clearance_study import digest, preserve_json
from run_dynamic_passage_study import ARENA, create_world
from run_matched_vision_training import write_csv
from run_recovery_scope_study import trial
from run_route_recovery_study import schedule as route_schedule
from run_wait_margin_study import ACTOR_SHA256, RGB_SHA


SOURCE_TREE = "d7de9992ed5aaaa4033c7c6ee94ce88614dd1512"
MODES = ("observe", "shorten")
HOLDOUT = "split_approach_lanes"
HOLDOUT_SEEDS = (71601, 71602)


def schedule(stage):
    previous, geometries = route_schedule(stage)
    modes = dict(zip(("legacy", "guarded"), MODES))
    runs = [dict(run, route_recovery_mode="guarded", recovery_frontier_scope="shared",
                 recovery_motion_mode="fixed", waypoint_visibility_mode=modes[run["route_recovery_mode"]],
                 layout_group="diagnostic" if stage == "smoke" else "repeated") for run in previous]
    if stage == "formal":
        openings = [(-2.82, -1.70), (0.68, 1.67)]
        obstacles = [
            (ARENA[0], 0.25, openings[0][0], 0.45),
            (openings[0][1], 0.25, openings[1][0], 0.45),
            (openings[1][1], 0.25, ARENA[2], 0.45),
            (-1.70, 1.60, -0.40, 1.82), (2.25, 0.45, 2.47, 2.70),
            (-2.62, -2.75, -2.42, -0.98), (0.38, -1.84, 2.28, -1.64),
        ]
        geometries[HOLDOUT] = dict(arena=ARENA, obstacles=obstacles, openings=openings,
                                   wall_y=0.35, blocked_exit=None, spawn_y=4.19, spawn_yaw=-1.58)
        for repeat, seed in enumerate(HOLDOUT_SEEDS):
            order_modes = MODES if repeat == 0 else MODES[::-1]
            for order, mode in enumerate(order_modes, 1):
                runs.append(dict(runs[0], scene=HOLDOUT, environment_seed=seed, order=order,
                                 waypoint_visibility_mode=mode, layout_group="holdout"))
    return runs, geometries


def run_study(source_tree, plan, stages, dependencies, description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("stage", choices=stages)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("actor", type=Path)
    parser.add_argument("rgb", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    project, actor, rgb, output = (path.resolve() for path in (args.ros_project, args.actor, args.rgb, args.output))
    package = project / "turtlebot_ddpg"
    workspace_package = subprocess.check_output(["rospack", "find", "turtlebot_ddpg"], text=True).strip()
    if Path(workspace_package).resolve() != package or digest(actor) != ACTOR_SHA256 or digest(rgb) != RGB_SHA:
        raise ValueError("Workspace or checkpoint fingerprint differs")
    tree = subprocess.check_output(["git", "-C", str(project), "write-tree"], text=True).strip()
    subprocess.run(["git", "-C", str(project), "diff", "--quiet", "--",
                    "turtlebot_ddpg", "turtlebot3_description", "laser_filters"], check=True)
    if tree != source_tree:
        raise ValueError("Use the audited source tree for this study")
    sys.path.insert(0, str(package / "scripts/voronoi_exploration"))
    runs, geometries = plan(args.stage)
    dependencies = tuple(dict.fromkeys((*dependencies, "run_waypoint_visibility_study", "audit_waypoint_visibility",
                    "run_recovery_scope_study", "run_route_recovery_study",
                    "audit_recovery_displacement", "audit_recovery_scope", "audit_route_recovery",
                    "audit_gain_passage", "run_dynamic_passage_study", "run_wait_margin_study",
                    "run_policy_goal_retention", "run_matched_vision_training", "run_clearance_study",
                    "run_rgb_clearance_guard_study", "run_rgb_passage_study")))
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
        for scene in geometries:
            if len({row["coverage_mask_sha256"] for row in results if row["scene"] == scene}) != 1:
                raise ValueError(f"Reference masks differ for {scene}")
        write_csv(output / "summary.csv", results)


def main():
    run_study(SOURCE_TREE, schedule, ("smoke", "formal"), (), __doc__)


if __name__ == "__main__":
    main()
