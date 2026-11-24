# Closed-loop retest of RGB direction alignment

The [encoder audit](rgb-encoder-audit.md) identified opposite side orders
in two historical training manifests. This retest loads a raw export and
a side-aligned export of the same robust checkpoint. The export reorders
only the last layer's output rows and retains its format-2 inputs and
weights. Both models load through the same production inference code.
Across 715 archived test images, the raw export and reordered export
match their expected scores exactly in the tested runtime.

## Locked navigation protocol

Four conditions share the 32-update LiDAR MAPPO actor from seed 62600,
DDPG, three starting poses, simulation horizon and laser occupancy grid.
The policy sees LiDAR in every condition. In the three camera conditions
the frontier rules use the same buffered 50:50 rule fusion, which can
change frontier ownership and the set of actions offered to the actor.
The recent RGB doorway reranker stays off.

| Condition | Rule clearance input |
| --- | --- |
| LiDAR | Common-scan LiDAR |
| Legacy RGB | Common-scan LiDAR and the published RGB encoder |
| Raw robust RGB | Common-scan LiDAR and the original robust score order |
| Aligned robust RGB | Common-scan LiDAR and the same robust scores with the sides exchanged |

The schedule uses two fresh seeds in Hu-aligned and two in Layout C,
with 90 simulation seconds per run. Each of the four conditions takes
each order position once. The within-seed aligned-minus-raw coverage
change is the primary comparison for the direction correction. The
aligned-minus-LiDAR comparison tests whether the corrected cue adds
exploration value in this protocol. Other outcomes include time to
90% and 97% coverage, arrivals, path length, near-collision events
and recovery. The runs do not retest independent long-budget MAPPO
training.

The runner takes the shared Gazebo lock. It verifies cue arithmetic
at every assignment, the actor checkpoint and configuration, coverage
mask and simulation horizon. Existing attempts keep their original
logs. The runner assigns a new directory to any unfinished attempt.

[Schedule, exact weights and export verification](../ros_gazebo/experiments/2026-11-24-rgb-encoder-retest/)

## Reproduce

Prepare and source the ROS workspace as described in [the guide](ros-gazebo.md).
Use the seed-62600 [LiDAR actor](../ros_gazebo/experiments/2026-11-24-rgb-passage/lidar_mappo_62600_update32.pt)
and the exports in the protocol directory. On headless hosts, prefix the
runner with `xvfb-run -a`.

```sh
STUDY="$PWD/ros_gazebo/experiments/2026-11-24-rgb-encoder-retest"
python3 scripts/run_rgb_encoder_retest.py "$ROS_PROJECT" "$ACTOR" "$STUDY/models" "$RUN_DIR"
```
