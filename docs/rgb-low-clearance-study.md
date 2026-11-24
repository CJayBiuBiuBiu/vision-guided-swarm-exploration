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
