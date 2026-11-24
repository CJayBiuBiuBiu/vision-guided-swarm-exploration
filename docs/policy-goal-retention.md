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
python3 scripts/run_policy_goal_retention.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_policy_goal_retention.py "$FORMAL_OUT" "$ANALYSIS_OUT"
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

## Results

All 24 formal trajectories reached the fixed 60 s horizon and passed
the audit. The four seed blocks share one frozen MAPPO actor, DDPG
controller and speed shield. The table shows means over four runs
per condition and pooled counts for goal arrival.

| Frontier cue | WAIT handling | Coverage at 60 s | Tracked goals reached | Team path (m) | Near-collision events |
| --- | --- | ---: | ---: | ---: | ---: |
| LiDAR | Legacy | 45.38% | 0/225 (0.00%) | 30.36 | 170.75 |
| LiDAR | Explicit cancellation | 31.23% | 7/238 (2.94%) | 15.80 | 92.25 |
| LiDAR | Short hold | 36.46% | 21/147 (14.29%) | 21.04 | 122.75 |
| Guarded RGB | Legacy | 45.97% | 4/227 (1.76%) | 31.11 | 174.00 |
| Guarded RGB | Explicit cancellation | 25.26% | 6/225 (2.67%) | 14.04 | 89.50 |
| Guarded RGB | Short hold | 38.55% | 14/160 (8.75%) | 22.00 | 125.50 |

In all eight within-cue pairs, short hold increased final coverage
relative to explicit cancellation. The mean increase was 5.23
percentage points with LiDAR and 13.29 with guarded RGB. The two
hold arms deferred 327 WAIT decisions across 200 assigned goals.
The robots reached 28 of those retained goals. The low-level logs
recorded receipt of all 739 waypoint retirements sent in the cancel
and hold arms. Near-collision events increased from the cancel
arms to the hold arms as team paths also grew. These counts record
LiDAR crossings of the 0.18 m threshold with a 0.25 m reset.

Legacy execution achieved higher coverage than short hold in
every matched seed and cue. In 61 of the 65 legacy intervals with
sufficient position samples, robots moved more than 0.05 m between
a WAIT closure and the next assignment. The corresponding counts
were 21 of 67 for explicit cancellation and 10 of 30 for short
hold. Motion can include recovery. The trajectory analysis excludes
intervals with fewer than two position samples at least 0.25 s after
closure and before reassignment. A tracked arrival requires the
coordinator to keep the goal active through arrival, so legacy
coverage and tracked arrivals capture different outcomes.

The actor chose WAIT in all 120 observed states with an active goal
and only WAIT and CONTINUE available. The mean conditional softmax
probability of WAIT was 51.13%. Deterministic action selection
turned this small preference into recurring WAIT commands.

With short hold, guarded RGB changed coverage relative to LiDAR
by +1.65, +6.04, +3.66 and −3.00 percentage points in seed order.
The mean paired difference was +2.09 points. The guarded RGB group
also had a lower pooled tracked arrival rate. The study covers
two simulated layouts and one training lineage. All runs ended
below 90% coverage, leaving the 90% and 97% times unobserved.
These results support short hold as an optional execution mode
and motivate testing the actor's WAIT policy at longer horizons.

[Per-run results, paired contrasts and figures](../ros_gazebo/experiments/2026-11-24-policy-goal-retention/)
