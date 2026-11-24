# RGB floor continuation beyond the LiDAR horizon

The Waffle Pi's planar scan reaches 3.5 m in this setup. The passage
module samples the floor at 3.85–5.65 m using the RGB image, camera
intrinsics and the robot pose. A near-floor sample supplies a colour
reference for each frame. The algorithm checks floor continuity at
2.0–3.1 m before scoring the farther samples. An invalid or unavailable
sample leaves the selector with its original preference.

At a new goal assignment, the selector can redirect a frontier whose
far score is at most 0.35 to a same-owner candidate with a score of at
least 0.80. It requires a grid path for both options and allows up to
0.60 m of extra path. A frame must be at most 0.40 s old with at most
0.25 m of translation since capture. These prototype thresholds apply to
this Gazebo setup with a flat matte floor and a calibrated low camera.
Semantic free space, uneven ground and generalisation to new floor
materials have yet to be evaluated.

| Launch mode | Behaviour |
| --- | --- |
| `off` | Default coordinator behaviour. |
| `observe` | Logs the proposed substitution and keeps the selected frontier. |
| `rerank` | Applies the substitution when the evidence and route qualify. |

## Stationary test

Both 9.6 × 10 m scenes start three Waffle Pi robots north of a partition,
with a 1.5 m opening on one side. The colour and lighting stay fixed
between scenes. The middle robot is 4.30 m from the partition centre
line. The two raw scan beams bracketing each door direction report no
return in both scenes. In each image, the floor score is 1.00 through
the opening and 0.00 toward the wall. The four directions come from two
mirrored scenes, so this establishes discrimination in the probe and
supplies no estimate of performance in other environments.

[Probe CSV, source frames and synchronized sensor records](../ros_gazebo/experiments/2026-11-24-rgb-passage/probe/)

## Matched navigation protocol

Each condition uses the same frozen LiDAR MAPPO policy from training
seed 62600 at update 32, low-level DDPG, laser occupancy grid and spawn
poses. The fixed fusion condition uses 50:50 RGB–LiDAR clearance in the
frontier rules while the actor sees LiDAR in every condition. The
passage condition adds the new assignment check to that same fusion
condition. Both control conditions log the counterfactual passage
decisions. Four independent seeds pair the three conditions across the
two mirrored layouts, with a fixed 90-second simulation budget per run.
The 40-second smoke seed is separate. Choice, occupancy coverage,
navigation and near-collision events can be audited against the raw run
output. During the 40-second smoke test, all three conditions finished
and passed the trace check. The passage condition made no substitutions
in this smoke seed. Its coverage value supplies a pipeline check, not an
estimate of the method's effect. One early Gazebo crash was retried with
the same seed and the failed attempt was retained.
[Smoke manifest and CSV](../ros_gazebo/experiments/2026-11-24-rgb-passage/smoke/)

Prepare and build the [ROS workspace](ros-gazebo.md), source the project
environment and run these commands from this repository on the ROS host.
Gazebo camera rendering requires a working display. On a headless host
with Xvfb, prefix each Python command with `xvfb-run -a`.

```sh
ACTOR="$PWD/ros_gazebo/experiments/2026-11-24-rgb-passage/lidar_mappo_62600_update32.pt"
python3 scripts/run_rgb_passage_study.py probe "$ROS_PROJECT" "$RUN_DIR/probe"
python3 scripts/run_rgb_passage_study.py smoke "$ROS_PROJECT" "$RUN_DIR/smoke" --actor "$ACTOR"
python3 scripts/run_rgb_passage_study.py pilot "$ROS_PROJECT" "$RUN_DIR/pilot" --actor "$ACTOR"
```

The runner takes the shared Gazebo experiment lock and records the source
tree, scene and model hashes in each stage manifest. When a stage resumes,
the runner verifies completed runs and uses a new attempt directory for
any unfinished condition. All launch and sensor logs remain on disk.

## Navigation results

All 12 trials reached 90 simulation seconds and passed the checkpoint,
coverage-mask and decision-trace checks. A run counts as a crossing when
at least one robot's recorded position moves 0.25 m south of the partition
centre line. Runs with no observed crossing are censored at 90 s in the
plot. The table reports four trials per condition across the two mirrored
layouts.

| Condition | Final coverage, mean | Runs with a crossing | Runs at 90% | Runs at 97% | Near-collision events, total |
| --- | ---: | ---: | ---: | ---: | ---: |
| LiDAR only | 92.49% | 4/4 | 3/4 | 0/4 | 12 |
| Fixed fusion | 89.62% | 4/4 | 3/4 | 0/4 | 13 |
| Fusion with RGB passage | 65.39% | 2/4 | 1/4 | 0/4 | 6 |

The passage condition finished with lower coverage in all four pairs
against each control. Its mean paired difference was −27.10 percentage
points against LiDAR and −24.23 points against fixed fusion. Near-collision
events use the laser clearance threshold and depend on how far the
robots travel and which parts of the arena they reach.

The passage condition assigned 243 goals. In 48 decisions the camera
classified the preferred direction as closed. Only two of those
decisions offered a qualifying alternate frontier, and the module
applied both substitutions in right-opening scenes. The MAPPO actor
cancelled each new goal with a WAIT action about 1.49–1.75 simulation
seconds later. No substituted goal was reached. WAIT ended 220 of 243
assignments in the passage condition and 449 of 507 assignments in the
controls. The large coverage differences also occur in pairs with zero
substitutions, so the pilot cannot attribute those differences to
individual overrides. The result identifies a gap between the visual
measurement, availability of eligible frontier choices and execution
of the replacement goal.

[Trial table, paired differences, decision events and figures](../ros_gazebo/experiments/2026-11-24-rgb-passage/pilot/)

To recreate the tables and figure from the raw pilot directory:

```sh
python3 scripts/summarize_rgb_passage_study.py "$ROS_PROJECT" "$RUN_DIR/pilot" "$RUN_DIR/analysis"
python3 scripts/plot_rgb_passage_study.py "$RUN_DIR/analysis/navigation.csv" "$RUN_DIR/analysis"
```
