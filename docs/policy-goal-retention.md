# MAPPO goal retirement and short retention

The [guarded RGB study](rgb-low-clearance-study.md) recorded goal
abandonment at a median age of 1.5 simulation seconds. The coordinator
removed each accepted `WAIT` goal from its own state. The robot-side
DDPG node retained the last waypoint because the coordinator sent no
retirement message. This left an execution mismatch between the
coordinator state and the commanded robot. The frozen actor also
needed a direct audit of its active-goal action preferences.

## Execution modes

The new launch option `policy_wait_mode` defaults to `legacy`.

| Mode | Action on a consumed WAIT |
| --- | --- |
| `legacy` | Clears the coordinator goal and uses the original waypoint handling |
| `cancel` | Clears the goal and retires the old waypoint at the robot |
| `hold` | Retains a young goal if an independent route and clearance check succeeds |

A hold can defer an early `WAIT` until the goal reaches an age of
4.5 s. The check requires a fresh low-level LiDAR clearance reading
within 0.5 s, clearance above 0.25 m, a goal cell free of mapped
obstructions and an available grid path. Arrival, ownership changes,
mapped goal obstructions and stuck events still follow the coordinator
invalidation and recovery paths. After retirement the robot idles
when there is no active recovery. An ongoing stuck recovery can
finish before the robot idles. All six experiment arms use the same
front-facing LiDAR speed shield and existing DDPG weights.

## Matched experiment

The factorial design crosses all three execution modes with two
frontier-rule cues. One uses LiDAR alone. The other uses the
[calibrated RGB checkpoint](rgb-low-clearance-study.md) and guarded
fusion. Both use the same frozen LiDAR MAPPO actor and the same
features, inference interval, spawns and horizon. The rule cue can
change frontier ownership and the resulting action set. Actor
telemetry records its selected action and the normalized-logit
difference between `WAIT` and `CONTINUE` whenever both are enabled.

Four new seed blocks run 60 s per condition. Each layout has two
seeds. The second seed in a layout reverses the six-method order.
The smoke test uses two 25-second trials and a separate seed. The
formal comparisons are explicit cancellation versus legacy
execution, hold versus explicit cancellation, and guarded RGB
versus LiDAR at the same execution mode. Coverage and goal arrivals
are reported with each fallback, retirement and retained-target outcome.

Build the prepared workspace with patch 14 and source ROS. Select the
seed-62600 [actor](../ros_gazebo/experiments/2026-11-24-rgb-passage/lidar_mappo_62600_update32.pt)
and the [calibrated encoder](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/encoder.pt).
Use distinct output directories. On a headless host prefix each
command with `xvfb-run -a`.

```sh
python3 scripts/run_policy_goal_retention.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_policy_goal_retention.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
```

## Implementation checks

All 122 ROS-side Python tests passed. Both 25 s smoke trials
finished and passed the clearance, actor and decision audits.
The explicit-cancel arm reached 3 of 25 assigned goals and the
robot nodes recorded 22 retirements. The hold arm reached 6
of 21 goals and deferred 14 WAIT actions with valid route and
clearance checks. The formal comparisons use four different seeds.

[Sealed smoke results and formal schedule](../ros_gazebo/experiments/2026-11-24-policy-goal-retention/)

The ninth formal trajectory completed normally, but the merged
launch stream omitted one robot's startup line. The verifier now
uses each robot's own ROS log for the shield and retirement checks.
It revalidated the eight earlier trajectories and recovered the
ninth from its existing CSV and logs. The original schedule and
raw attempts remain archived with a verifier revision record.
