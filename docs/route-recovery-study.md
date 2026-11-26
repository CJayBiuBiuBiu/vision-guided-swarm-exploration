# Route-aware recovery and exploration efficiency

The [information-aware passage study](gain-passage-study.md) found a
coverage improvement alongside longer paths and slightly more stuck
reports. It motivates separating movement expenditure from repeated
failures. This stage retains the information-aware RGB selector and
both frozen policy checkpoints.

## Diagnosis from the previous run archive

Match each low-level stuck warning to the coordinator's received
position and to the active goal-attempt trace in simulation time.
In the 21 stuck reports from the four information-aware runs,
nine reports left the reporting robot's active goal unretired.
Eight reports revisited a position within 0.90 m of an earlier
report by that robot, all within twenty seconds. In seed 71322
alone, seven of ten reports retained the active target. Robot 3
reported three failures near (4.4, 4.9) while its target remained
near (3.9, 1.9). Robot 2 reported repeated failures near
(-1.1, 0.5) while continuing to target (-4.3, -0.5).

The original callback excluded targets within 1.8 m of the
failure position, but a distant target could remain active and
lead the reporter back along the same route. The original
recovery movement also used a fixed reverse-and-turn command.
The diagnosis does not require modifying the historical results.

## Optional route-aware recovery

The new `route_recovery_mode:=guarded` setting queues stuck reports
for processing on the coordinator's control thread. It retires the
reporter's current goal and path even when the target is far from
the failure position, cancels any active passage inspection or
continuation, and sends waypoint retirement. The coordinator
pauses assignments for 2.75 seconds, allowing the bounded 2.50 s
low-level recovery to complete. Actions received before the
pause ends cannot issue a new target.

Each robot records its failed target and failure position in
separate temporary memory for twenty seconds. A target within
0.75 m of the remembered target is excluded for the reporting
robot. Eligible teammates may take ownership when its original
owner cannot use it. The planner adds three graph-cost units to
cells within 0.65 m of a remembered failure position. These are
soft penalties, so the robot can still leave a failure region.
Neither the occupancy grid nor the exploration denominator
changes. The existing shared stuck-position blacklist remains
in effect in both conditions.

The Waffle Pi model provides 24 laser rays over a forward
180-degree arc. The forward escape therefore starts by turning
toward the clearer observed side, using at most 0.80 rad/s. It
enables straight forward motion only after at least 0.65 s of
turning, at least 0.35 rad of measured heading change toward
the selected side, and a front-sector clearance above 0.20 m.
Forward speed is capped at 0.15 m/s. The corresponding scan
sector scales the turn or forward command linearly from zero
at 0.20 m to full speed at 0.35 m. A missing or older-than-0.40 s
scan produces zero motion. The DDPG policy, 24-ray observation,
and everyday forward-speed shield remain the same. The stuck
definition remains displacement below 0.15 m over three
seconds. Near-collision telemetry retains its original
navigation sample cadence.

The recovery-event trace records the active goal, failure
position, report time, retirement, pause deadline and memory
deadline. The verifier reconstructs target memory and checks
that subsequent goals respect it. Individual robot logs record the turn and forward phases,
measured heading change, clearances, and commands for replay.

## Rejected rear-scan prototype

The first matched diagnostic pair exposed a sensor mismatch.
The rear-sector guard yielded zero reverse speed in all 70
guarded samples because the simulated 24-ray sensor does not
observe the rear sector. The guarded diagnostic achieved
65.57% coverage and zero of five displacement successes; legacy
recovery achieved 95.74% and two of four. The first complete
holdout pair under that prototype was preserved, and its next
partial rollout was stopped. These records are diagnostic and
excluded from the revised evaluation. The revised source adds
a front-only sensor regression test and runs all matched
conditions in a separate archive.

## Matched evaluation

Both conditions use the gain-region selector introduced in
patch 19 and the same MAPPO, RGB encoder, DDPG and weak-WAIT
settings. The new patch adds optional route recovery. The
baseline uses legacy recovery with all of the previous visual
gain behaviour.

A separate diagnostic pair repeats seed 71322 and its earlier
narrow-west layout for 120 s per condition. The formal experiment
uses three new topology families, each with two independent
simulation seeds (71421–71426), for twelve formal 120 s trials:

- Staggered partitions: two distinct internal horizontal
  barriers with offset openings.
- Divided rooms: a primary partition and a vertical room
  divider with a side opening and internal obstacles.
- Offset approaches: internal obstructions before the primary
  partition and secondary downstream barriers.

Within each matched seed the world, starting poses and model
fingerprints agree. The paired condition order alternates.
The two seeds per family share the same world geometry and
starting poses. This tests repeatability of simulation rollouts
and generalisation beyond a single internal partition. The
interior map is unknown at the start; the control information
omits reference coverage and still uses Gazebo pose. The
crossing audit measures the primary partition, while coverage
measures the full environment.

Report coverage, total path length, newly explored reference-free
area per metre after the initial sample, stuck reports per ten
metres, repeat reports within 0.90 m (including a twenty-second
subset), recovery displacement success, near-collision events
and crossing outcomes. The separate evaluation mask is used for
coverage metrics only. A lower absolute path length is not in
itself an improvement if the robot explores less space.

Prepare the patched workspace, source the environment and use the
same frozen actor and RGB checkpoint as the previous stage.

```sh
python3 scripts/run_route_recovery_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_route_recovery_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_route_recovery_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_route_recovery_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_route_recovery_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

Use a virtual display on a headless host. The historical
failure-diagnosis script can also reproduce the event alignment
from the previous gain-region archive:

```sh
python3 scripts/diagnose_route_failures.py "$PREVIOUS_FORMAL_OUT" "$DIAGNOSIS_OUT"
```

## Revised diagnostic pair

The front-only revision passed all 151 ROS tests. Both seed
71322 trials reached the horizon and passed the replay audits.
The revised guarded run reached 96.80% and crossed with two
robots, while the matched rerun of legacy recovery reached
64.51% with no crossing. The respective stuck counts were
two and seven; guarded recovery translated at least 0.15 m
on both attempts, while legacy achieved that threshold in
four of seven. The trace verified 36 forward escape samples
and retired both failed goals, including one 4.20 m from its
stuck pose. The guarded run travelled 59.97 m versus 36.33 m.
Its observed-area gain per metre was 0.788 versus 0.813 m²/m,
so the single diagnostic demonstrates mechanism operation
and higher coverage without a path-efficiency gain. This pair
is separate from the formal unknown-topology analysis.

## Formal unknown-topology results

All twelve trials completed the 120 s horizon and passed both
verification passes. Each topology's four runs shared its world
and coverage mask, and matched seeds used identical checkpoints,
spawn poses and evaluation options apart from recovery mode.
The three topology masks were distinct. The comparison excludes
both prototype diagnostics and the revised seed 71322 pair.

| Topology | Seed | Legacy coverage | Guarded coverage | Stuck reports (legacy / guarded) |
| --- | ---: | ---: | ---: | ---: |
| Staggered partitions | 71421 | 95.15% | 79.79% | 6 / 6 |
| Staggered partitions | 71422 | 53.81% | 47.11% | 2 / 2 |
| Divided rooms | 71423 | 60.06% | 93.92% | 3 / 5 |
| Divided rooms | 71424 | 61.43% | 58.39% | 4 / 2 |
| Offset approaches | 71425 | 56.99% | 76.35% | 5 / 9 |
| Offset approaches | 71426 | 56.62% | 88.60% | 6 / 5 |
| Mean | | 64.01% | 74.03% | 4.33 / 4.83 |

The paired mean coverage difference was +10.02 percentage points
with gains in three of six seeds. The guarded strategy improved
both offset-approach seeds and lost both staggered-partition
seeds. Its mean coverage in divided rooms was higher with one
win and one loss. At least one robot crossed the primary
partition in four of six guarded trials and one of six legacy
trials. Team crossing counts were six and two, respectively.
Each method reached 90% once, and neither reached 97%.

| Formal measure | Legacy | Guarded |
| --- | ---: | ---: |
| Mean team path (m) | 34.50 | 36.84 |
| Mean new area after initial sample (m²) | 22.19 | 25.14 |
| Mean new observed area per metre (m²/m) | 0.657 | 0.695 |
| Mean stuck reports per ten metres | 1.283 | 1.256 |
| Total stuck reports | 26 | 29 |
| Repeat reports within 0.90 m and 20 s | 2 | 2 |
| Reports that retained the active goal | 13 | 0 |
| Completed recoveries translating at least 0.15 m | 23/25 | 26/27 |
| Mean near-collision threshold events | 1.50 | 2.00 |
| Mean robot-seconds without an assigned goal | 149.12 | 155.23 |

On average the guarded teams explored 5.9% more new area per
metre and travelled 2.34 m farther. Per-metre gains occurred
in three of the six pairs. The motion telemetry therefore
supports a modest mean efficiency increase alongside an
increase in total distance and raw stuck reports. The
near-collision measure tracks clearance threshold events
from the existing cadence. It does not count physical contacts.
Completed recovery success uses the same displacement criterion
with different allowed recovery durations.

### Recovery traces and limits

All 29 guarded reports retired the reporter's active assignment.
Ten targets were at least 1.8 m from the failure location and
would have escaped the individual goal-distance check in the
legacy callback. The command audit verified 674 guarded samples,
including 461 forward-escape samples with measured turn and
valid front clearance. No stale sample was recorded in the
formal rollouts. The newer strategy had two short-window repeat
reports, the same total as legacy. Coverage and repeats can
vary independently because map topology and high-level goal
selection affect where the robot goes after recovery.

The goal-free measure integrates the number of robots without an
assigned goal over sampled simulation time. It includes active
recovery and does not imply the robot was physically stopped.
For example, the 71421 guarded run gained more area per metre
than legacy but had approximately 119 additional robot-seconds
without an assigned goal and lower coverage. The difference
is descriptive and does not isolate the pause or memory effect.

The experiment addresses the stale-goal mechanism and extends
evaluation to three previously unused layout topologies with two
seeds each. The combined intervention is still opt-in because
the paired coverage results vary and raw stuck and near-collision
counts increased. The comparison combines target retirement,
memory-based rerouting and forward recovery. Isolating those
components and testing additional maps and independently trained
policies would support stronger attribution.

The workspace passed 151 ROS tests, three passage-controller
checks and 55 project tests. The raw archive contains separate
prototype and revised runs. The revised archive verification
matched 420 SHA-256 entries.

[Formal tables, figures, world files and audit logs](../ros_gazebo/experiments/2026-11-26-route-recovery/)
