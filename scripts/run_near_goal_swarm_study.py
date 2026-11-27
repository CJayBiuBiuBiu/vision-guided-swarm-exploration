"""Test the near-goal tracker in three-robot exploration with fixed perception and policies."""

from run_dynamic_passage_study import ARENA
from run_route_recovery_study import schedule as route_schedule
from run_waypoint_visibility_study import run_study


SOURCE_TREE = "dbf713b17948bfdf304e8cca2f2a6222c275ae15"
MODES = ("ddpg", "near_goal")
HOLDOUT = "offset_chicane"
HOLDOUT_SEEDS = (71801, 71802)


def schedule(stage):
    previous, geometries = route_schedule(stage)
    modes = dict(zip(("legacy", "guarded"), MODES))
    runs = [dict(run, route_recovery_mode="guarded", recovery_frontier_scope="shared",
                 recovery_motion_mode="fixed", waypoint_visibility_mode="shorten",
                 local_tracking_mode=modes[run["route_recovery_mode"]],
                 layout_group="diagnostic" if stage == "smoke" else "repeated") for run in previous]
    if stage == "formal":
        openings = [(-2.19, -1.10), (1.68, 2.65)]
        obstacles = [
            (ARENA[0], -0.60, openings[0][0], -0.40),
            (openings[0][1], -0.60, openings[1][0], -0.40),
            (openings[1][1], -0.60, ARENA[2], -0.40),
            (-2.70, 0.48, -1.30, 0.68), (0.96, 0.35, 1.15, 2.45),
            (-0.80, -2.70, -0.61, -0.90), (1.80, -2.20, 3.52, -2.0),
        ]
        geometries[HOLDOUT] = dict(arena=ARENA, obstacles=obstacles, openings=openings,
                                   wall_y=-0.50, blocked_exit=None, spawn_y=4.25, spawn_yaw=-1.60)
        for repeat, seed in enumerate(HOLDOUT_SEEDS):
            order_modes = MODES if repeat == 0 else MODES[::-1]
            for order, mode in enumerate(order_modes, 1):
                runs.append(dict(runs[0], scene=HOLDOUT, environment_seed=seed, order=order,
                                 local_tracking_mode=mode, layout_group="holdout"))
    return runs, geometries


if __name__ == "__main__":
    run_study(SOURCE_TREE, schedule, ("smoke", "formal"),
              ("run_near_goal_swarm_study", "audit_local_tracking"), __doc__)
