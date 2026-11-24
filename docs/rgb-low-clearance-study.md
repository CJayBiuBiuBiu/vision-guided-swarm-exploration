# Low-clearance supervision and guarded fusion

The [checkpoint retest](rgb-encoder-retest.md) found that RGB still
overestimated clearance in front of close obstacles after aligning its
left and right outputs. This study changes training supervision and the
frontier-rule fusion separately.

## Training design

The factorial test uses the same three-layer-output TinyOpennessCNN,
three initialisation seeds, 320 updates per run, and 24 frames per
update. It pairs two data sources with two objectives.

| Factor | Comparison |
| --- | --- |
| Data | Historical aligned corpus versus November captures isolated by run and image hash |
| Objective | Uniform regression and ranking versus a loss that upweights low-clearance targets and penalises overestimation on them |

The data comparison changes collection runs and label distribution as
well as image isolation. The objective comparison changes the low-target
weights and overestimation penalty together. Both comparisons hold the
network, sampling budget, augmentations and optimiser fixed. Each
recipe receives 7,680 sampled frame presentations per seed. The fit
uses brightness jitter and horizontal flips with matching side labels.

The four existing held-out capture runs serve as development data for
this new training. A recipe is chosen by mean development score across
its three initialisations. The score weights overall MAE and low-class
MAE equally at 0.40 each, with weight 0.20 on balanced best-direction
error. The second seed of the chosen recipe supplies the representative
checkpoint. This choice is written before reading test labels.

The independent test consists of 150 paired images and scan records
from each of three fresh Gazebo runs in Hu-aligned, Layout C and Layout B.
Exact image hashes shared with training or development are excluded
from the test. The closed-loop candidate must beat the archived
side-aligned robust encoder on pooled overall MAE, low-clearance MAE
and mean per-sector low-clearance AUROC. The check uses 0.25 as the
low-clearance cutoff on the normalized scan score.

## Rule guard

The new `guarded_fusion` rule cue uses the same synchronized LiDAR and
RGB pair as fixed fusion. Its camera weight is zero when LiDAR
clearance is at most 0.25. The weight increases linearly to the
configured fusion weight between 0.25 and 0.50. With an unavailable
camera pair the cue uses the common fresh LiDAR fallback. The guard
affects frontier scoring. The low-level DDPG controller and the
occupancy-grid sensor are unchanged.

## Navigation design

The predefined four-way comparison uses a fresh seed twice per
layout, Hu-aligned and Layout C, and a fixed 75 s budget. The comparison
includes pure LiDAR, archived RGB with fixed fusion, the selected RGB
with fixed fusion and the same selected RGB with the guard. One frozen
seed-62600 LiDAR MAPPO checkpoint and one DDPG checkpoint serve all
conditions. The policy always receives LiDAR directly. The rule cue
can change frontier ownership and hence the action set. The audit
reconstructs every recorded assignment score and checks that the
guard preserves every observed low LiDAR sector.

The two primary comparisons are new versus archived RGB with fixed
fusion, and guarded versus fixed fusion of the same new checkpoint.
The pure LiDAR arm anchors the practical interpretation. Coverage at
75 s is the navigation endpoint. The paired results also report paths,
goals reached and near-collision events.

## Factorial test results

The three runs yielded 450 image and scan records. One image matched
an earlier input hash, leaving 449 test frames. The table averages
the three trained initialisations for each recipe.

| Data | Objective | Overall MAE | Low-clearance MAE | Low-clearance AUROC |
| --- | --- | ---: | ---: | ---: |
| Historical | Uniform | 0.311 | 0.935 | 0.642 |
| Historical | Low-clearance weighted | 0.373 | 0.343 | 0.581 |
| Run-isolated | Uniform | 0.173 | 0.466 | 0.737 |
| Run-isolated | Low-clearance weighted | 0.435 | 0.162 | 0.737 |

The isolated captures improved overall and low-clearance estimates
under the uniform objective with the same architecture and budget.
Heavy low-clearance weighting reduced error on the blocked class and
increased error elsewhere. The selected weighted candidate scored
0.170 low-clearance MAE versus 0.706 for the archived baseline. Its
overall MAE was 0.422 versus 0.265, so it failed the deployment check.

[Training design, raw recipe metrics and first test gate](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/factorial/)

## Moderate score calibration

The follow-up uses seed 66702 from the isolated-data uniform recipe.
It uses one positive scale and one bias shared across the three
output logits. The transform preserves direction ordering and uses
the same inference architecture. The search uses the development
runs only. Candidate parameters must improve the three deployment
metrics on development and keep the non-low MAE within 0.05 of the
archived encoder. Among eligible parameters, selection minimises
0.55 times overall MAE plus 0.45 times low-clearance MAE. The frozen
export uses scale 0.75 and bias −0.85.

The second test repeats the capture protocol in three new runs
using seeds 66314–66316. Its test loader excludes image hashes shared
with training, development or the first test. The navigation stage
uses the same deployment check on this new test.

[Calibration grid and frozen checkpoint](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/)

### Independent calibration result

The second test retained 447 of 450 captured frames after hash
exclusion. The calibrated checkpoint passed the independent check.

| RGB encoder | Overall MAE | Low-clearance MAE | Low-clearance AUROC |
| --- | ---: | ---: | ---: |
| Archived aligned encoder | 0.229 | 0.674 | 0.622 |
| Calibrated encoder | 0.223 | 0.426 | 0.674 |

The guard addresses the remaining optimistic error when the paired
scan already measures close obstacles. The navigation schedule
keeps the new encoder fixed across the guarded and fixed conditions.

[Independent test metrics and deployment check](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/)

## Reproduce the closed-loop test

Build and source the ROS workspace with patch 13. Use the actor from
the [doorway study](rgb-passage-study.md), the archived aligned robust
checkpoint from the [encoder retest](rgb-encoder-retest.md), and the
calibrated model from this study. Use a distinct run directory.
On a headless host prefix the command with `xvfb-run -a`.

```sh
STUDY="$PWD/ros_gazebo/experiments/2026-11-24-rgb-low-clearance"
python3 scripts/run_rgb_clearance_guard_study.py "$ROS_PROJECT" "$ACTOR" \
  "$ARCHIVED_ALIGNED_RGB" "$STUDY/calibrated" "$RUN_DIR"
```

## Verified closed-loop results

All 16 trials reached 75 simulation seconds. A second audit
recalculated the recorded fusion scores and verified the completed
schedule. The ROS test suite passed all 120 tests.

| Frontier rule cue | Mean coverage | Mean path length | Low scan scores increased |
| --- | ---: | ---: | ---: |
| LiDAR | 56.11% | 45.79 m | 0/120 |
| Archived RGB, fixed fusion | 60.60% | 48.48 m | 108/108 |
| Calibrated RGB, fixed fusion | 54.95% | 47.83 m | 109/128 |
| Calibrated RGB, guarded fusion | 56.54% | 47.29 m | 0/108 |

The calibrated fixed fusion finished 5.65 percentage points below
archived fixed fusion on average. The guard raised the calibrated
configuration's mean coverage by 1.59 points relative to fixed fusion.
Against LiDAR, the guarded configuration led by 0.43 points on average
and won one of the four seed pairs. These outcomes show no repeatable
coverage gain within the tested layouts and budget. None of the
conditions reached 90% or 97% coverage within 75 s.

Archived fixed fusion raised low scan scores by 0.362 on average.
Calibrated fixed fusion reduced this increase to 0.200. The guard
preserved all 108 low scores seen in its assignment traces. In the
five previously recorded close-obstacle views, the replay used the
same image and scan to compare old RGB, calibrated RGB and the
guard. For the two obstacles flanking an open centre, side scores
changed from 0.821 and 0.802 to 0.560 and 0.397. The guarded fusion
scores were 0.339 and 0.319, close to the scan scores of 0.309.

The shared policy's `WAIT` action still ended most assignments.
In the guarded arm it ended 241 of 265 goals, at a median of 1.5 s
after assignment. Across all four conditions the median was 1.5 s.
The traces show that target retention remains a limitation during
these runs. They do not quantify how changing it would affect coverage.

Set `clearance_rules_cue:=guarded_fusion` with
`vision_scoring_mode:=matched_directional_clearance` to opt into the
guard. The published comparison keeps `clearance_actor_cue:=lidar`.

[Verified runs, paired metrics and coverage figure](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/formal_analysis/)
· [Controlled image replay](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/obstruction_replay.csv)
