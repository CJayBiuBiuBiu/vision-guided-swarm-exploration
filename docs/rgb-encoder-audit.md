# RGB clearance encoder diagnostic

The audit compares the published RGB encoder used in the recent ROS/Gazebo
experiments with the September robustness checkpoint. It also compares
the results against a camera-free median predictor fitted to separate
capture runs. The audit leaves the navigation code and checkpoint files intact.

## Opposite left and right orders

The checkpoint metadata identifies two training manifests. The published
encoder came from `manifest_camera_aligned.csv`. The September robust
checkpoint came from `manifest.csv`. The two files describe the same
720 rows in the same order. The aligned file exchanges the left and
right fields in every row, changing 315 numeric label pairs. Both
format-2 checkpoints declare the same output names. This lets a
loader accept either checkpoint even though the side definitions differ.

The stationary test checks each opening inside and beyond the 3.5 m
planar LiDAR horizon. In both near scenes the published encoder gives
the higher side score to the actual opening. The raw robust checkpoint
gives the higher score to the wall. Exchanging its two side scores
restores the open-side preference. The same pattern appears in both
archived far scenes. In these scenes the runtime clearance fields use a
positive robot-relative bearing for the field named `right`, which points
to the physical left. The LiDAR feature and frontier projection use that
same order.

[Stationary measurements, images and scans](../ros_gazebo/experiments/2026-11-24-rgb-encoder-audit/geometry/)

## Held-out image replay

The audit verified 715 clean frames and their LiDAR targets from four
separate capture runs across Hu-aligned and Layout C. A ranking frame
needs a gap of at least 0.10 between the largest and second-largest
recorded LiDAR scores. The 245 qualifying frames contain 85, 36 and
124 examples in the recorded left, centre and right fields. MAE uses
all frames on the normalized clearance scale. The shuffled control
reassigns predictions within each capture run.

| Cue | MAE | Best-sector agreement | Balanced agreement |
| --- | ---: | ---: | ---: |
| Fit-run median per sector | 0.251 | 50.6% | 33.3% |
| Published encoder, clean | 0.229 | 69.8% | 55.0% |
| Published encoder, shuffled | 0.269 | 44.5% | 35.7% |
| Published encoder, darkened | 0.285 | 35.5% | 33.9% |
| September robust, raw | 0.290 | 15.9% | 12.5% |
| September robust, side scores exchanged | 0.236 | 69.4% | 54.2% |

The published encoder contains useful directional information on clean
frames. Its average directional spread is 0.183, compared with 0.339
from the reference scan. Both RGB checkpoints chose the centre in
zero of the 36 clear centre-best cases, including after the robust side
exchange. When the scan reports clearance of 0.25 or less, the published
encoder overestimates the scan score by 0.738 on average. The 50:50 blend
with the same scan still increases that low-clearance score by 0.369 on
average. In the same blend, exchanging the raw robust scores raises
agreement from 78.8% to 98.8%. This replay measures changes to a known
scan cue. Exploration effects require a separate closed-loop test.

The original corpus also has 58 image paths shared across runs. Forty-five
carry conflicting labels. In the legacy and robust splits, respectively,
24 of 120 and 44 of 240 validation rows refer to a path also used in
training. The independent captures above use separate run directories and
verified image hashes.

The layout and range checks distinguish two issues. A checkpoint-specific
side convention explains the robust encoder's mirrored signs. The published
encoder already uses the project runtime order, so that mismatch does not
explain its weaker matched policy results. Its errors on low clearance,
centre directions and darkened images remain after aligning the conventions.
Both encoders learn a LiDAR clearance target. The replay therefore measures
how well RGB estimates that target. The two fixed far-opening scenes show
that RGB can convey information outside the scan horizon, where this
target cannot supervise the distinction.

[Per-frame predictions, run metrics and checkpoint provenance](../ros_gazebo/experiments/2026-11-24-rgb-encoder-audit/offline/)

## Reproduce

Use the prepared ROS source, the [isolated camera dataset](adaptive-clearance-study.md)
and the two original training checkpoints with the hashes in the offline
manifest. Use distinct output directories. Source the ROS workspace before
the range probe, which uses the shared Gazebo lock.

```sh
python3 scripts/audit_rgb_encoder.py "$ROS_PROJECT" "$ISOLATED_CAPTURES" \
  "$QUALITY_PROFILE" "$HISTORICAL_DATASET" "$ORIGINAL_RGB" "$ROBUST_RGB" "$OFFLINE_OUT"
python3 scripts/probe_rgb_encoder_geometry.py "$ROS_PROJECT" "$FAR_PROBE" \
  "$ROBUST_RGB" "$GEOMETRY_OUT"
```
