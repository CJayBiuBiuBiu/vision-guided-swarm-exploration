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
same three map families and two simulation seeds per family.
For each seed, both arms run for 120 s. The comparison
measures the change in exclusion scope on these known evaluation
layouts. The robots still build the unknown interior map from
their scans.

The audit checks the source tree, launch parameters, checkpoint
hashes, reference masks, robot-specific exclusions and recovery
commands. It also checks the number of raw cells available
after the shared filter and counts teammate goals retired by
the stuck handler. Report coverage, clustered frontier
availability, travelled distance, new area per metre, stuck
reports per ten metres and clearance events. Time with at
most two clustered frontiers includes samples with raw
frontier cells available and excludes the terminal row.

Prepare the workspace through patch 21 and source it.

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
