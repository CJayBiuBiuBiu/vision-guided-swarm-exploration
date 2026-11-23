# Depth guidance below the LiDAR scan plane

The Waffle Pi Gazebo model has a horizontal scanner 0.132 m above its
nominal footprint. The benchmark adds three 0.09 m boxes within the
Hu-aligned arena. Their bodies intersect the robot base. The existing
RGB encoder estimates directional LiDAR clearance. This experiment adds
a depth image to measure the low boxes directly.

The optional camera shares the pose of the existing RGB camera and
publishes a separate 160 × 120 depth image at up to 8 Hz. The coordinator
projects ranges between 0.25 and 2.5 m into world coordinates using the
reported intrinsics and a Gazebo pose received within 0.10 s. It keeps
returns between 0.035 and 0.116 m above the simulated floor. Sensor samples
expire after 0.40 s. For the depth–LiDAR diagnostic, the two samples must
also be within 0.22 s of each other. Both images and raw scans retain
their sensor timestamps.

The coordinator tests the direct segment to the first commanded
waypoint. Three or more depth returns within 0.175 m of that segment
flag a candidate. The pilot tests three modes on the same physical
scene with the same cameras running:

| Condition | Existing clearance feature | Depth action |
| --- | --- | --- |
| LiDAR | Common-scan LiDAR | Log the depth test |
| Fusion | Fixed 50:50 RGB–LiDAR | Log the depth test |
| Depth | Fixed 50:50 RGB–LiDAR | Try the next owned candidate when depth flags the proposal |

The depth condition tries alternatives in the existing rule cost order
when MAPPO proposes a blocked target. On rule-driven ticks it keeps the
refined rule order. It withholds a new assignment if every evaluated
candidate is flagged. Missing or stale depth leaves the proposal in
place. The current implementation checks new assignments, uses
Gazebo poses, and keeps the scan occupancy grid and DDPG motion
controller. It is a high-level guidance experiment. The depth image,
contact telemetry, and known box coordinates serve different purposes.
The coordinator uses only projected sensor returns when it selects a
target. The offline audit uses box coordinates to identify depth-only
detections near the test objects. The Gazebo contact sensors record
physical contact with the boxes.

## Run and verify

Prepare a fresh ROS workspace, build it, and source it. On a headless
host, prefix each command with `xvfb-run -a`.

```sh
study=ros_gazebo/experiments/2026-11-23-depth-corridor
python3 scripts/run_depth_corridor_study.py "$ROS_PROJECT" "$study/smoke_schedule.csv" "$SMOKE_OUTPUT" --horizon 28
python3 scripts/run_depth_corridor_study.py "$ROS_PROJECT" "$study/pilot_schedule.csv" "$PILOT_OUTPUT"
```

The pilot uses two matched seed blocks, one with the low boxes and one
with the clear reference scene. Each block runs three conditions at the
same 75-second budget. The two blocks use opposite condition orders.
The reference coverage denominator is the original Hu-aligned floor
mask for all runs. It includes the small footprints of the added boxes,
which matters when comparing coverage across the scenes. Interpret each
three-condition block separately.

The runner records the source tree, checkpoint and world hashes,
metrics, launch logs, and matched depth and LiDAR evidence at each
selection. It saves the preferred candidate, tested alternatives,
shadow decision and applied assignment. The audit reconciles applied
assignments with the outcome trace and counts contacts with each
added box, grouping observations of the same robot and box within
0.30 s into one episode. Repeating a command revalidates completed
conditions and resumes the schedule. The same Gazebo lock protects
both this pilot and the clearance studies.

## Smoke test

The build and 115 ROS Python tests passed. The 28-second matched smoke
block used seed 62490. Both observation-only conditions recorded a
depth-only hit near a low box. The depth condition retargeted three
assignments and reached all three replacement goals. It withheld one
other assignment. The contact trace contained six contact episodes
for LiDAR, eight for fixed fusion and zero for depth guidance. This
single seed verifies the instrumentation and target-selection path.

[Smoke results](../ros_gazebo/experiments/2026-11-23-depth-corridor/smoke_summary.csv)
· [Recorded inputs](../ros_gazebo/experiments/2026-11-23-depth-corridor/smoke_manifest.json).
