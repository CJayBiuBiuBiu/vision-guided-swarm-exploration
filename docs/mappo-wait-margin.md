# Checking weak MAPPO wait preferences

The [goal-retention study](policy-goal-retention.md) found 1,450
deterministic WAIT decisions with an active goal and both WAIT and
CONTINUE enabled. The WAIT logit exceeded CONTINUE by at most
0.095. When only those two actions were enabled, the actor selected
WAIT in all 120 observations. A narrow margin can result in
frequent abandonment under deterministic inference.

## Runtime change

Set `policy_wait_mode:=cancel policy_wait_margin:=0.10` to try the
rule with deterministic inference. The default margin is zero.
An active-goal WAIT can request a route recheck if both actions
are allowed and the logit difference is between zero and 0.10.
The upper bound corresponds to a 52.5% softmax probability when
conditioned on the two actions. This threshold was set from the
previous study. It is a score comparison between two available
actions.

The coordinator accepts a recheck if the goal is still active,
less than 12 s old, and reachable by a new grid path. The same
low-level LiDAR reading must be fresh within 0.5 s and exceed
0.25 m in the study. If the check fails, the coordinator handles
WAIT and explicitly retires the waypoint. Strong WAIT preferences,
ordinary CONTINUE actions, and idle-robot action masking follow
the existing decision path. Arrival, ownership and stuck detection
still run. The controller can record the raw choice, the published
action and the result of the recheck.

## Experiment

The formal runs cross LiDAR and guarded RGB frontier cues with
explicit cancellation, a 4.5 s short hold, and the 0.10 margin
with a 12 s recheck limit. The six conditions share the seed-62600
MAPPO actor, DDPG weights, fixed spawns and front LiDAR speed shield.
All use LiDAR for actor features and observed free area for
control. The four seed blocks replay the two layouts and seeds
from the goal-retention study. Each condition runs for 60
simulation seconds. Within each layout the second seed reverses
the condition order. The two smoke runs use a separate seed and
a 25 s horizon.

The comparison assesses the complete recheck rule against
explicit cancellation and the earlier short hold. Comparing the
margin arm with short hold changes both the scoring rule and the
goal-age limit. A matched time-limit control would be needed to
attribute the difference to the score margin alone.

Build and source the prepared ROS workspace with patch 15. Set
`ROS_PROJECT` to that checkout, `ACTOR` to the
[seed-62600 actor](../ros_gazebo/experiments/2026-11-24-rgb-passage/lidar_mappo_62600_update32.pt),
and `RGB` to the
[calibrated encoder](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/encoder.pt).
Use distinct output directories. On a headless host, prefix the
commands with `xvfb-run -a`.

```sh
python3 scripts/run_wait_margin_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_wait_margin_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_wait_margin_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_wait_margin_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
```

## Smoke results

All 123 ROS-side Python tests passed. Both 25 s smoke runs completed
in a fresh workspace after rebuilding the ROS messages. The short
hold reached one of 17 assigned goals and covered 25.92%. The
margin arm reached two of 17 and covered 17.81%. It accepted 13
of the 27 rechecks the coordinator received. Twelve failed the
clearance check and two arrived after the goal was gone. The smoke
run verifies the route check and rejection paths on a separate seed.

[Smoke summaries and formal schedule](../ros_gazebo/experiments/2026-11-24-mappo-wait-margin/)
