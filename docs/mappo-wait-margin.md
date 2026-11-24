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
python3 scripts/report_wait_margin_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

## Smoke results

All 123 ROS-side Python tests passed. Both 25 s smoke runs completed
in a fresh workspace after rebuilding the ROS messages. The short
hold reached one of 17 assigned goals and covered 25.92%. The
margin arm reached two of 17 and covered 17.81%. It accepted 13
of the 27 rechecks the coordinator received. Twelve failed the
clearance check and two arrived after the goal was gone. The smoke
run verifies the route check and rejection paths on a separate seed.

## Formal results

All 24 formal trajectories completed their 60 s horizon. The
coordinator sent 1,025 waypoint retirements and the robot logs
recorded all 1,025. The table shows per-condition means and pooled
tracked-goal arrivals.

| Frontier cue | WAIT strategy | Coverage | Goals reached / assigned | Team path (m) | Near-collision events |
| --- | --- | ---: | ---: | ---: | ---: |
| LiDAR | Explicit cancellation | 28.12% | 3/230 (1.30%) | 15.80 | 95.25 |
| LiDAR | 4.5 s short hold | 31.00% | 26/159 (16.35%) | 19.79 | 119.50 |
| LiDAR | Margin and route recheck | 34.80% | 37/148 (25.00%) | 21.04 | 117.75 |
| Guarded RGB | Explicit cancellation | 28.83% | 1/231 (0.43%) | 15.89 | 90.50 |
| Guarded RGB | 4.5 s short hold | 36.85% | 18/156 (11.54%) | 20.00 | 118.50 |
| Guarded RGB | Margin and route recheck | 36.32% | 45/148 (30.41%) | 21.35 | 133.75 |

The recheck rule improved tracked arrival rates in all eight
comparisons with short hold. Relative to short hold, LiDAR coverage
increased by a mean of 3.81 percentage points across four seeds.
Three increased and one decreased. Near-collision events decreased
by 1.75 per trial on average. With guarded RGB, coverage decreased
by 0.53 points on average and near-collision events increased by
15.25 per trial. This metric counts crossings of the LiDAR
threshold with hysteresis.

At the same recheck setting, substituting guarded RGB for the
LiDAR frontier cue changed coverage by +0.60, +9.99, −4.67 and
+0.15 points in seed order. The average change was +1.52 points.
The guarded arm also averaged 16 more near-collision events than
the LiDAR arm. This variation limits the evidence for a consistent
navigation gain from the RGB cue.

The actor requested a recheck for all 571 active-goal WAIT
decisions in the margin arms. The coordinator accepted 376
requests across 193 goals, of which 66 were reached. It rejected
164 requests on clearance, 26 after a goal cleared, four on age
and one because the grid route was unavailable. All observed
active-goal WAIT scores in the margin arms fell inside the margin,
so these trials quantify the combined effect of gated continuation
and the 12 s limit. The study reuses four environment seeds and one
actor training lineage. All runs ended below the 90% and 97%
coverage thresholds.

[Per-run results, paired contrasts and figures](../ros_gazebo/experiments/2026-11-24-mappo-wait-margin/)
