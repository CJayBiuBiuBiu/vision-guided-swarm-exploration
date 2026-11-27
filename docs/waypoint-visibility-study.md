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

Prepare the ROS workspace through patch 23 and source it.

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
