# Fixed-waypoint test of the low-level DDPG controller

The preceding [waypoint study](waypoint-visibility-study.md)
shortened 1,329 commands. Their median target distance became
0.60 m. The archived DDPG training environment samples initial
longitudinal target offsets from 11.5 to 16.5 m and resets
when the distance falls below 1.0 m. In the accompanying
human demonstration file, 25 of 1,040 current states have a
target distance below 1.0 m. That file describes one source
of training data.

The frozen-actor probe holds laser beams and previous actions
fixed. It substitutes signed goal bearings and distances in
150 archived demonstration states. The probe also includes a
uniform clear scan with stationary history. A turn with the
opposite sign to the goal bearing can reflect obstacle
avoidance. The physical benchmark checks this issue on clear
and obstacle-adjacent fixtures with known reachable targets.

The [waypoint study](waypoint-visibility-study.md) also has
365 goals closed with the WAIT outcome across 737 assignments.
Of those closures, 266 occurred within 3 s of assignment.
The median age was 1.75 s in each arm. The lower-level
benchmark fixes each goal for its full trial so it can separate
controller behavior from high-level action changes.

## Tracking option

The `near_goal` tracker handles targets within 1 m when the
minimum filtered laser range is at least 0.35 m and the
scan age is at most 0.40 s. It turns toward the goal with a
bounded angular command. It advances only when heading
error is below 45 degrees. Forward speed tapers with heading
error and distance, stays at or below 0.20 m/s, and drops to
zero within 0.08 m. The existing forward laser shield still
applies. The frozen DDPG action handles the other states.
The launch setting is `local_tracking_mode:=near_goal`.
The default is `ddpg`.

## Gazebo protocol

The benchmark calls the production explorer's local step
function with the frozen checkpoint. One robot drives in
an open room and a second layout with offset posts. The
other spawned robots remain parked. Each layout has eight
fixed targets per arm, including mirrored side and rear
targets and a 2 m forward control. The arm order alternates.
The collector resets the robot pose and waits for a fresh
scan before each case. All runs use the same forward laser
shield. A target counts as reached after 0.25 s within
0.12 m. The horizon is 12 simulation seconds. The team
coordinator, RGB decisions and stuck recovery are outside
this controlled test.

Prepare and source the ROS workspace through patch 24.

```sh
python3 scripts/probe_ddpg_tracking.py "$ROS_PROJECT" "$PROBE_OUT"
xvfb-run -a python3 scripts/run_ddpg_waypoint_benchmark.py "$ROS_PROJECT" "$BENCH_OUT"
python3 scripts/run_ddpg_waypoint_benchmark.py "$ROS_PROJECT" "$BENCH_OUT" --audit-only
python3 scripts/report_ddpg_waypoint_benchmark.py "$BENCH_OUT" "$ANALYSIS_OUT"
```

## Results

The frozen actor probe returned a turn in the opposite
direction to the goal bearing for 122 of 300 inputs at
0.60 m and 90 degrees. It did so for 25 of 300 inputs
with the same laser and previous-action features at 2.50 m.
The numbers describe a controlled input change. They do not
classify individual actions as unsafe.

All 32 Gazebo trials passed a second audit. The policies
used identical targets and the same DDPG checkpoint.

| Outcome | Frozen DDPG | Near-goal controller |
| --- | ---: | ---: |
| Reached targets within 1 m | 8/14 | 14/14 |
| Reached 2 m forward targets | 0/2 | 2/2 |
| Reached targets across both layouts | 8/16 | 16/16 |
| Laser threshold events across both layouts | 1 | 1 |

Both 0.25 m targets timed out with frozen DDPG in each
layout. All four reached the tolerance band with near-goal
tracking. For the 0.60 m left target in the open room,
the frozen actor reached the target after 10.5 s and
the near-goal mode after 5.6 s. In the room with posts,
the same case timed out with DDPG and reached in 5.5 s
with near-goal tracking. The 2 m cases start under DDPG
in both arms. The near-goal option changes final approach
after entering its operating range.

The 16 pairs share two layouts and one physics seed per
layout. The results support a limited correction for
fixed-target tracking. DDPG remains the default for team
exploration pending a matched integration comparison.

[Trial summary](../ros_gazebo/experiments/2026-11-27-ddpg-waypoint-benchmark/summary.csv)
· [Frozen actor probe](../ros_gazebo/experiments/2026-11-27-ddpg-waypoint-benchmark/offline/actor_response.csv)
· [Paired results](../ros_gazebo/experiments/2026-11-27-ddpg-waypoint-benchmark/analysis/paired_results.csv)
· [Trajectory figure](../ros_gazebo/experiments/2026-11-27-ddpg-waypoint-benchmark/analysis/fixed_waypoint_trajectories.pdf).
