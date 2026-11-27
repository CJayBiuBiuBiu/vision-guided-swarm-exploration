# Near-goal tracking in three-robot exploration

The [fixed-waypoint benchmark](ddpg-waypoint-benchmark.md)
found that adding geometric tracking helped the local
controller reach nearby targets. This comparison measures
how the same option affects the complete exploration loop.
The two arms share the MAPPO actor, calibrated RGB encoder,
DDPG checkpoint, scan-map waypoint visibility check,
laser speed shield, and fixed-duration guarded recovery
with shared frontier exclusion. The only controller setting
that differs is `local_tracking_mode`, set to `ddpg` or
`near_goal`.

Each navigation step records the target distance and
bearing, filtered scan timestamp and clearance, active
controller, requested speed and published command. The audit
replays eligibility, checks the geometric command and repeats
the shared laser scaling. Both arms emit the same trace
fields. Diagnostic traces cover navigation steps. They
exclude recovery and camera inspection commands.

Each run uses three robots and a 120 s simulation horizon.
The formal schedule pairs the controllers on six seeds
across three previously used layouts. A fourth layout,
offset chicane, uses new seeds 71801 and 71802. The
order alternates within the schedule. The diagnostic pair
uses seed 71322. All interiors are mapped online with
the evaluation mask reserved for measuring coverage.

The summary reports final coverage, crossings, path length,
stuck reports and laser near-collision threshold events.
It also tracks how often the geometric controller takes
over and counts WAIT endings within 3 s of assignment.
Command counts can include repeated updates of one target.
The geometric tracker is eligible only with a short target,
a fresh scan and sufficient filtered laser clearance.

Prepare the workspace through patch 25 and source it.

```sh
python3 scripts/run_near_goal_swarm_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_near_goal_swarm_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_near_goal_swarm_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_near_goal_swarm_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_near_goal_swarm_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

## Diagnostic pair

Both runs on seed 71322 reached the 120 s horizon and
passed the command audit. The near-goal controller was
active in 1,048 of 2,521 navigation steps in its arm.
Final coverage was 65.57% with DDPG and 88.03% with
near-goal tracking. Stuck reports fell from eight to
five. The laser threshold triggered once with DDPG
and zero times in the near-goal arm. The diagnostic
pair checks integration. The formal schedule measures
behavior across layouts and seeds.
