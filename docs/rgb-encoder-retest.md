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

## Verified navigation results

All 16 runs reached the 90 s horizon. A second audit verified the
completed schedule without launching another trial.

| Rule cue | Mean final coverage | Mean path length | Mean traced arrivals |
| --- | ---: | ---: | ---: |
| LiDAR | 73.29% | 56.87 m | 2.75 |
| Legacy RGB fusion | 66.97% | 56.01 m | 3.25 |
| Raw robust RGB fusion | 67.09% | 55.52 m | 1.75 |
| Aligned robust RGB fusion | 67.58% | 59.12 m | 1.75 |

The aligned-minus-raw mean difference was +0.49 percentage points.
The aligned-minus-LiDAR mean difference was −5.71 points. Alignment
led in two of four matched seeds against each baseline. No run reached
90% or 97% coverage within the horizon. These four blocks provide a
short-budget comparison with one fixed actor. The effect varies by seed.

The runtime traces confirm that the residual RGB error entered frontier
scoring. Across the aligned arm, 109 recorded sector values had common
LiDAR clearance of 0.25 or lower. Fusion increased the rule score by
0.350 on average for those sectors. Seventeen assigned goals had a
positive projected LiDAR score of at most 0.25. The corresponding rule
bonus was 0.392 higher on average. These comparisons reconstruct cue
values from the same sensor pair at each decision. They show what the
rules received and do not identify the coverage effect of an individual
assignment.

Across the 715 independent frames, the aligned encoder scored only
2 of 263 low-LiDAR sectors at or below the same 0.25 cutoff. AUROC for
distinguishing low clearance from the remaining scores ranged from
0.531 to 0.635 by sector. This weak separation persists after correcting
the left and right order.

[Run data, paired differences, calibration and no-grid figure](../ros_gazebo/experiments/2026-11-24-rgb-encoder-retest/analysis/)

```sh
python3 scripts/summarize_rgb_encoder_retest.py "$RUN_DIR" "$SUMMARY_OUT" \
  --offline-predictions ros_gazebo/experiments/2026-11-24-rgb-encoder-audit/offline/predictions.csv
python3 scripts/plot_rgb_encoder_retest.py "$RUN_DIR/summary.csv" "$SUMMARY_OUT"
```
