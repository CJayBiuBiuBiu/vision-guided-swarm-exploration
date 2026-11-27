# Displacement-aware guarded recovery

The [frontier exclusion comparison](recovery-scope-study.md)
recorded nine same-robot repeat reports within 0.90 m and 20 s.
For eight of those repeats, every recorded position between
the reports stayed within 0.65 m of the earlier stuck position.
The recovery log at seed 71422 also showed several successful
maneuvers that moved the robot by only 0.23–0.26 m. These
observations motivated a longer escape when the robot shows
measurable displacement.

The fixed controller uses a 2.50 s guarded maneuver. The
displacement-aware controller shares its scan freshness checks,
turn clearance, forward clearance and speed limits. It can
continue beyond 2.50 s if it has moved at least 0.12 m. It
ends at 0.65 m of displacement, when displacement falls below
0.12 m after the initial window, or at 6.25 s. The 0.65 m
target matches the radius of the existing route-cost penalty.
The coordinator holds new goals while a robot recovers.
It resumes on receipt of the measured outcome plus a 0.25 s
guard interval, capped by a 6.50 s pause.

Both study arms use the same shared frontier exclusion,
gain-region RGB strategy, MAPPO actor, DDPG motor policy,
RGB encoder, start poses and 120 s budget. The diagnostic
pair uses seed 71322. The formal schedule uses six paired
seeds across the three previously evaluated layouts. A fourth
layout with offset approaches to two openings uses two fresh
seeds, 71501 and 71502. Report that layout separately as well
as in the pooled comparison. The robots map each interior
online. Ground-truth masks supply the coverage measure.

The primary comparisons are coverage, passage crossings,
total stuck reports and same-robot repeats. Recovery
displacement is measured at the end of the local maneuver.
Near-collision counts refer to the project's laser threshold
during navigation and inspection. The audit separately checks
all logged guarded recovery commands against scan and speed
limits. The repeat measure uses simulation time, so
a longer maneuver can take up part of its 20 s observation
window. The figure reports sample counts for each pair.

Prepare the workspace through patch 22. Run the study in the
sourced ROS environment.

```sh
python3 scripts/run_recovery_displacement_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_recovery_displacement_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_recovery_displacement_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_recovery_displacement_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_recovery_displacement_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

The explorer and coordinator accept
`recovery_motion_mode:=fixed` or
`recovery_motion_mode:=displacement`. The second setting
requires `route_recovery_mode:=guarded`.

## Diagnostic pair

Both arms completed seed 71322 and passed the trace audits.
Mean recovery displacement was 0.24 m with fixed recovery and
0.66 m with displacement-aware recovery. The latter cleared
0.65 m in all five logged maneuvers. Final coverage changed
from 65.57% to 86.07%. Stuck reports rose from two to five.
This pair verifies that the new termination and release paths
execute in Gazebo. The formal schedule supplies the performance
comparison.
