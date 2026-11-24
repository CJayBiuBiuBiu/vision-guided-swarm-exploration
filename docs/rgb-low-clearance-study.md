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
