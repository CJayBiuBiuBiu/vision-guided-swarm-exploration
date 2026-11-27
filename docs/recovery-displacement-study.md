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
window. The analysis also counts later reports within 0.90 m
of the original stuck location in the 20 s after completion.
It uses episodes with a full 20 s of follow-up for this
second measure. The figure reports counts for each pair.

Use repository revision `2d07c4c` to match the verified
script hashes. Prepare the workspace through patch 22 and run
the study in the sourced ROS environment.

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

## Formal results

All 16 runs reached the 120 s horizon and passed a second
audit. The [verified manifest](../ros_gazebo/experiments/2026-11-27-recovery-displacement/formal/verified_manifest.json)
records the source tree and hashes for the checkpoints,
layouts and scripts. The initial audit rejected a completed
run when the explorer and coordinator clocks differed by
3 ms. The revised audit allows the same 0.15 s tolerance in
both directions. It rechecked the original output. The
comparison manifest changed only in the audit script hash.

| Outcome across eight runs per arm | Fixed | Displacement-aware |
| --- | ---: | ---: |
| Mean final coverage | 65.31% | 70.13% |
| Mean final coverage on the new layout | 56.00% | 60.60% |
| Runs with at least one passage crossing | 5 | 5 |
| Mean team distance | 41.36 m | 39.25 m |
| Recoveries clearing 0.65 m | 0/38 | 36/37 |
| Mean displacement at recovery completion | 0.23 m | 0.64 m |
| Stuck reports | 38 | 39 |
| Same-robot repeat reports | 2 | 3 |
| Repeats in fully observed 20 s post-recovery windows | 2/32 | 3/35 |
| Near-collision threshold events | 11 | 13 |
| Near-collision threshold events per 10 m | 0.33 | 0.41 |

Coverage improved in four of eight matched pairs. The mean
gain was 4.82 percentage points across all pairs and 4.60
points on the new layout. On that layout, seed 71501 lost
13.86 points and seed 71502 gained 23.06 points. Each arm
crossed a passage in one of the two runs. All runs ended
below 90% coverage.

The extended controller cleared the failed-route radius in
36 of its 37 logged outcomes. One ended for insufficient
progress. Two displacement-aware reports near the horizon
had no completed maneuver in the analysis window. The
increase in local displacement coexisted with a similar
number of subsequent stuck reports. Of 24 logged clearance
events, 23 occurred during navigation and one occurred
during inspection. The event stream does not cover the
recovery phase, whose commands have a separate audit.

The guarded controller defaults to fixed recovery. The
displacement-aware option increased local escape distance
across the evaluated layouts. The coverage results span gains and
losses, with two paired seeds on one new layout.

[Paired results](../ros_gazebo/experiments/2026-11-27-recovery-displacement/analysis/paired_deltas.csv)
and [comparison figure](../ros_gazebo/experiments/2026-11-27-recovery-displacement/analysis/recovery_displacement_comparison.pdf).
