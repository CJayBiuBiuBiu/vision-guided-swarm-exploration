# Robot-scoped frontier exclusion after a navigation failure

The preceding route-recovery comparison exposed a trade-off in
staggered partitions. At seed 71421, the guarded run spent 36.5
seconds with at most two clustered frontiers while the shared
stuck blacklist was active. The run still had 87 raw frontier
cells at the horizon. The shared blacklist removes cells up to
1.8 m from a reported stuck position for 90 s. The same report
can retire a nearby teammate's assignment. In a map with few
passages, that scope can temporarily restrict the whole team.

The robot-scoped setting leaves raw frontiers available for
clustering. For 20 s, the reporting robot excludes targets
within 0.75 m of its failed goal and targets within 0.65 m of
its stuck position. The existing path cost around the stuck
position still applies to that robot. The coordinator can
reassign eligible frontiers to teammates and preserve their
current goals. Reporter goal retirement, the 2.75 s recovery
pause, and the forward laser recovery controller run in both
conditions. The default shared scope continues to use the
original filters.

The new per-robot clearance log records the time, scan stamp,
position, measured clearance and navigation or inspection
context when the existing near-collision threshold triggers.
The counts still describe threshold events. The log records
the branch sampling the event and can associate it with an
earlier stuck report by the same robot.

## Matched comparison

Use `route_recovery_mode:=guarded` in both arms. Select
`recovery_frontier_scope:=shared` or
`recovery_frontier_scope:=robot`. Both arms use the gain-region
visual strategy, the same policy checkpoints, and the same
Gazebo start poses and sensor configuration.

The smoke pair repeats the 120 s diagnostic at seed 71322.
The formal comparison repeats the six matched seeds from the
[route-recovery study](route-recovery-study.md). It uses the
same three layouts and two simulation seeds per layout.
For each seed, both arms run for 120 s. The study compares the
two exclusion rules as configured, including their radii and
durations. The robots build the interior map from their scans.

The audit checks the source tree, launch parameters, checkpoint
hashes, reference masks, robot-specific exclusions and recovery
commands. It also checks the number of raw cells available
after the shared filter and counts teammate goals retired by
the stuck handler. Report coverage, clustered frontier
availability, travelled distance, new area per metre, stuck
reports per ten metres and clearance events. Time with at
most two clustered frontiers includes samples with raw
frontier cells available and excludes the terminal row.
The analysis also counts later stuck reports within 0.90 m
and 20 s of an earlier report. It records whether both reports
came from the same robot. This is a proximity measure based
on reported positions.

Use repository revision `247691b` to reproduce the archived
manifest and script hashes. Prepare the workspace through
patch 21 from that revision and source it.

```sh
python3 scripts/run_recovery_scope_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_recovery_scope_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_recovery_scope_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_recovery_scope_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_recovery_scope_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

## Smoke test

Both 120 s trials passed their audits. At seed 71322, coverage
was 86.15% with shared exclusion and 96.07% with robot-scoped
exclusion. Both arms recorded three stuck reports and completed
three recoveries with sufficient displacement. The clearance
threshold triggered twice in the shared arm and once in the
robot-scoped arm. The source tree passed 153 ROS tests, including
checks for local exclusion expiry and retaining a nearby
teammate's goal.

## Formal results

All 12 runs reached the 120 s horizon and passed replay audits.
Both arms used the same MAPPO, DDPG and RGB checkpoints. The
[experiment manifest](../ros_gazebo/experiments/2026-11-26-recovery-scope/formal/experiment_manifest.json)
records the checkpoints, seeds and world hashes.

| Outcome across six runs per arm | Shared | Robot-scoped |
| --- | ---: | ---: |
| Mean final coverage | 74.70% | 69.62% |
| Mean time with at most two clustered frontiers | 7.54 s | 0.00 s |
| Runs with at least one passage crossing | 6 | 4 |
| Runs reaching 90% coverage | 1 | 0 |
| Mean team distance per run | 39.83 m | 42.82 m |
| Mean newly mapped area per metre | 0.663 m²/m | 0.557 m²/m |
| Stuck reports | 25 | 33 |
| Same-robot repeats within 0.90 m and 20 s | 3 | 6 |
| Cross-robot repeats within 0.90 m and 20 s | 0 | 1 |
| Near-collision threshold events | 10 | 12 |
| Recoveries with sufficient displacement | 16/23 | 31/32 |

Robot-scoped exclusion improved coverage in two of six pairs.
In staggered partitions, seed 71421 gained 16.80 percentage
points and seed 71422 lost 15.46. Seed 71422 produced three
same-robot repeats under robot-scoped exclusion. In offset
approaches, seed 71425 fell by 36.15 points. Neither arm spent
time with at most two frontiers on that seed. The shared arm
recorded two robots crossing a passage.

The additional frontier candidates did not consistently
produce shorter or more productive routes. Across both arms,
the coordinator retired zero teammate goals through the stuck
handler. Of the 22 logged threshold events, 21 occurred during
navigation and one during visual inspection. None occurred
within five seconds after a stuck report by the same robot.
This event log covers navigation and inspection callbacks.
Recovery success here measures displacement during the local
maneuver. It cannot establish subsequent exploration progress.

The default remains shared. The local variant gives a
diagnostic comparison for frontier availability and approach
selection. This comparison uses two seeds per layout and
changes the spatial and temporal exclusion parameters together
with scope. An independent comparison would fix those
parameters across both arms and use new layouts.

[Paired results](../ros_gazebo/experiments/2026-11-26-recovery-scope/analysis/paired_deltas.csv)
and [comparison figure](../ros_gazebo/experiments/2026-11-26-recovery-scope/analysis/recovery_scope_comparison.pdf).
