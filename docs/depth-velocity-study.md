# Low-obstacle depth at the DDPG velocity output

The [active-waypoint study](active-depth-recheck.md) used depth to
interrupt and reassign exploration goals. This study applies the
same low-height band locally at each robot before publishing its
forward velocity. The controller projects depth in the robot frame
using the Waffle Pi camera calibration. It checks a 0.35 m-wide
forward corridor for groups of three returns in an 8 cm longitudinal
band. Two successive frames within 0.30 s must agree within
0.15 m. Two fresh clear frames release a previous detection.
The controller uses recent samples up to 0.40 s old.

The speed factor decreases linearly from full speed at 0.72 m
to zero at and below 0.42 m, measured from the robot-frame origin.
The stop range leaves margin above the camera's usable near limit.
The controller retains DDPG steering, laser speed limiting and
the existing stuck recovery. When fresh depth evidence is
unavailable, the LiDAR-only command is used. The local check
targets forward translation. The camera configuration and
projection assume a calibrated Waffle Pi on a level surface.

| Mode | Use of the camera |
| --- | --- |
| `off` | Default. |
| `observe` | Log the candidate speed scale. |
| `limit` | Apply the same scale to forward velocity. |

The study runs the same cameras and the same LiDAR shield in
both arms. The coordinator uses the same frozen MAPPO checkpoint
with route rechecking, common LiDAR frontier cues and the same
laser occupancy grid. The high-level depth corridor is off.
The low-box scene adds three 9 cm obstacles below the planar scan.
Gazebo contact sensors count physical box contacts. The clear
Hu-aligned scene checks interference in the original environment.

The first diagnostic run used asynchronous physics and camera
updates. The three robots had fresh depth for 68–74% of audited
DDPG ticks. The verifier required at least 75% on every robot.
The formal protocol uses Gazebo lockstep in both arms and a
short frame history to align image time with the controller clock.
The original attempt and logs remain in the diagnostic archive.

Four low-box seeds and two clear seeds run 60 s per arm. Within
each scene, the first arm is balanced. The smoke pair uses a
different seed and a 28 s limit. Each run records the timestamp
and depth estimate, predicted and applied speed, scan clearance,
coverage and the box contact trace. The verifier reconstructs
the expected velocity and checks model, geometry and seed hashes.

After building patch 16 and sourcing the ROS workspace, set
`ACTOR` to the frozen seed-62600 MAPPO checkpoint and `RGB`
to the [calibrated RGB checkpoint](rgb-low-clearance-study.md).
Use a distinct directory for each stage. Prefix the commands
with `xvfb-run -a` on a headless host.

```sh
python3 scripts/run_depth_velocity_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_depth_velocity_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_depth_velocity_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_depth_velocity_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
```

## Smoke test

The fresh workspace built successfully and passed 126 ROS-side
tests. All controller ticks in both lockstep smoke trials had
fresh depth. The observation-only trial logged eight box
contact episodes. The limiting trial logged zero. It reduced
forward commands on 53 ticks when the LiDAR shield allowed
full speed, with seven full stops. Coverage was 17.95% and
22.66%, respectively. This single seed checks the end-to-end
sensor and velocity path.

[Smoke results and formal schedule](../ros_gazebo/experiments/2026-11-24-depth-velocity/)
