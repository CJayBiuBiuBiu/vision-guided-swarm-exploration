# Scan-visible waypoint lookahead

An offline replay reconstructed 670 initial lookahead
points from logged occupancy maps and matched route lengths.
In 191 cases the straight ray to the waypoint crossed a cell
marked occupied. A shorter waypoint on the same path passed
the raster visibility check in each replayed case. The
replay reconstructs the initial geometry. It does not
reconstruct the robot's subsequent motion.

[Offline replay evidence](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/diagnosis/).

The coordinator checks the original lookahead point against a
snapshot of its live laser occupancy map. When the ray
crosses an occupied cell, it walks back along the path to
the farthest candidate within the original 2.5 m lookahead
that passes the check. If it finds no such candidate, it
retires the active goal and waypoint so the next cycle can
select another route. Unknown cells retain their existing
status, and the LiDAR speed shield continues to govern
the motor command. This check concerns grid visibility.
It does not estimate the continuous swept robot footprint.

The comparison uses `waypoint_visibility_mode:=observe`
for standard lookahead and `:=shorten` for the guarded
point. Both arms log the same diagnostic snapshot for each
waypoint check. The audit restores each map, recomputes the
requested and eligible endpoints and checks the recorded
published point. The `off` setting disables this check.
Scan mapping and waypoint retirement support the `shorten`
setting.

Both arms use the same frozen RGB encoder, MAPPO actor and
DDPG weights. They also share gain-region RGB guidance,
the shared frontier exclusion and fixed-duration guarded
recovery. The formal schedule pairs each arm on six seeds
across three previous layouts and two additional seeds
on a new split-approach layout. Each run has a 120 s
simulation horizon. The diagnostic pair uses seed 71322.
Coverage, passage crossing, stuck reports and laser
near-collision threshold events are reported alongside the
number of shortened and withheld points.

Use repository revision `06e4969` to match the archived study
scripts. Prepare the ROS workspace through patch 23 and source it.

```sh
python3 scripts/run_waypoint_visibility_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_waypoint_visibility_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_waypoint_visibility_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_waypoint_visibility_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_waypoint_visibility_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

## Diagnostic pair

Both 120 s runs passed the waypoint replay audit. The
standard controller issued 68 points whose rays crossed
occupied cells. The scan-visible arm shortened 84 commands
and published zero such points. Final coverage was 98.28%
and 86.23%, respectively. The standard and scan-visible
arms reported four and six stuck events. They triggered
the laser near-collision threshold once and zero times.
The pair validates the live command check. The formal
schedule measures the exploration trade-off.

## Runtime correction

The eighth formal attempt stopped when waypoint retirement
accessed a depth-only cache with depth checking disabled.
The coordinator now clears the cache only when it exists by
configuration. The runner ignores incomplete attempts on
resume, including ROS launches that return zero after a
node stops. The earlier seven valid runs never entered the
retirement branch and passed the same audits after the fix.
The interrupted logs and both source trees are retained.

[Runtime tree ledger](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/formal/runtime_revisions.json)
and [resumed manifest](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/formal/verified_manifest.json).

## Formal results

All 16 runs reached the 120 s horizon and passed the repeat
audit. The controller replay checks each command against
the observed map at the time it was issued.

| Outcome across eight runs per arm | Standard | Scan-visible |
| --- | ---: | ---: |
| Published commands through occupied cells | 1,428/7,171 | 0/6,882 |
| Goals with at least one such command | 151 | 0 |
| Shortened commands | 0 | 1,329 |
| Withheld commands | 0 | 2 |
| Mean final coverage | 69.00% | 66.96% |
| Runs with at least one passage crossing | 5 | 5 |
| Mean team distance | 41.19 m | 37.60 m |
| Stuck reports | 31 | 47 |
| Stuck reports per 10 m | 0.94 | 1.56 |
| Laser near-collision threshold events | 12 | 12 |

The scan-visible arm improved coverage in three of eight
pairs. Its mean coverage fell by 2.04 percentage points
across all layouts. On the new split-approach layout the
mean rose from 65.30% to 68.86%. The two changes there
were +29.62 and −22.49 points. Two scan-visible runs and
one standard run crossed a passage on that layout. Both
arms stayed below 90% coverage in the formal schedule.

The guard changed which points reached the local controller.
Among shortened commands, the median target distance fell
from 1.67 m to 0.60 m. There were 345 issued targets closer
than 0.35 m among these commands. Repeated commands can
refer to the same waypoint or assignment. The results
measure the combined response of waypoint selection and
the frozen local controller. They do not establish the
cause of an individual stuck episode.

The selectable scan-visible mode removes the observed
geometry error. The default continues to use standard
lookahead because the scan-visible mode recorded more
stuck events and lower mean coverage in this comparison.

[Paired results](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/analysis/paired_deltas.csv)
· [Per-goal audit](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/analysis/attempt_waypoints.csv)
· [Target distance measurements](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/analysis/shortened_distances.csv)
· [Comparison figure](../ros_gazebo/experiments/2026-11-27-waypoint-visibility/analysis/waypoint_visibility_comparison.pdf).
