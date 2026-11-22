# Mixed-exposure clearance study

The constant-dark test validated the LiDAR fallback. This study measures
what happens when the same run returns to clean RGB. It uses the frozen
[camera-quality profile](../ros_gazebo/experiments/2026-11-22-camera-quality/calibration_results/camera_quality.json).

The coordinator selects the camera profile from each image timestamp on
the Gazebo clock. The experiment follows five 60-second windows. The
camera is clean in windows 0, 2 and 4. In windows 1 and 3, the
coordinator multiplies the RGB values by 0.28 before feature extraction
and inference. The gate uses brightness and detail features from the
resulting image. The exposure schedule also appears in the run evidence.

The pairing buffer can select a recent image from the preceding window.
Each traced assignment records the control timestamp and the paired
image timestamp. The audit verifies the corresponding window labels,
counts cross-window pairs, and reports acceptance separately for frames
captured in the current window. This distinguishes exposure transitions
from errors in the quality gate.

## Evaluation

The formal schedule uses four new seeds, two per layout. Each seed has
common-scan LiDAR, fixed fusion, and adaptive fusion as three conditions.
The settings apply to both MAPPO observations and the frontier rules.
The simulation time limit is 300 seconds for all twelve runs. We use
the same fixed model weights, start poses, matching rules and exposure
schedule in all three conditions.

The primary comparison is final coverage within each seed block.
Secondary results include time-averaged coverage, traced arrivals,
stuck events, distance, near-collision notifications and RGB use.
The report lists each seed before taking layout means. The two seeds
per layout support a replication screen for this exposure schedule.

Run the three-condition smoke schedule for 85 seconds to verify the
first exposure transition. The smoke check requires at least 70%
acceptance of paired clean frames and at most 5% acceptance of paired
dark frames in the current window. The run audit checks the effective
sensor cues and exposure counters before it writes a completion marker.
The runner stops its own launch on a reported segmentation fault. It
retries an early segmentation fault at most once under the same seed and
settings. It retains both attempt directories.

The smoke run retained every paired clean frame and rejected every paired
dark frame captured in the current window. In the dark window, the
adaptive run used RGB for three robot ticks from buffered clean images
captured before the transition. The
[smoke telemetry](../ros_gazebo/experiments/2026-11-22-mixed-exposure/smoke_exposure_windows.csv)
and [input manifest](../ros_gazebo/experiments/2026-11-22-mixed-exposure/smoke_manifest.json)
record those checks.

```sh
study=ros_gazebo/experiments/2026-11-22-mixed-exposure
profile="$PWD/ros_gazebo/experiments/2026-11-22-camera-quality/calibration_results/camera_quality.json"
python3 scripts/run_clearance_study.py "$ROS_PROJECT" "$study/smoke_schedule.csv" "$SMOKE_OUTPUT" \
  --horizon 85 --quality-profile "$profile"
python3 scripts/run_clearance_study.py "$ROS_PROJECT" "$study/paired_schedule.csv" "$STUDY_OUTPUT" \
  --quality-profile "$profile"
python3 scripts/summarize_clearance_quality_study.py "$ROS_PROJECT" "$STUDY_OUTPUT" "$profile" "$SUMMARY_OUTPUT"
```

Build and source the prepared ROS workspace. On a headless machine,
prefix each run command with `xvfb-run -a`. Resuming with the same
inputs revalidates completed runs and starts at the next condition.

## Verified results

All twelve runs completed 300 simulation seconds. The audit verified
the four matched seed blocks, all model and map fingerprints, image
timestamps, and the logged cue at each traced assignment.

| Layout | LiDAR | Fixed fusion | Adaptive fusion | Adaptive − LiDAR | Adaptive − fixed |
| --- | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 88.32% | 91.96% | 93.25% | +4.92 pp | +1.28 pp |
| Layout C | 97.17% | 90.01% | 80.40% | -16.77 pp | -9.61 pp |

Final coverage means give the two seeds in each layout equal weight.

| Layout | Seed | LiDAR | Fixed fusion | Adaptive fusion | Adaptive − LiDAR | Adaptive − fixed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 62300 | 94.87% | 94.32% | 89.84% | -5.04 pp | -4.49 pp |
| Hu-aligned | 62302 | 81.78% | 89.61% | 96.66% | +14.88 pp | +7.05 pp |
| Layout C | 62301 | 95.14% | 85.33% | 88.15% | -6.99 pp | +2.81 pp |
| Layout C | 62303 | 99.20% | 94.68% | 72.64% | -26.56 pp | -22.04 pp |

### Gate behaviour across exposure changes

| Exposure | Robot ticks | Adaptive RGB use | Gate acceptance within matched window | Accepted pairs from prior window |
| --- | ---: | ---: | ---: | ---: |
| Clean | 8,619 | 98.38% | 98.58% | 0 |
| Dark | 5,760 | 0.21% | 0.00% | 12 |

Window statistics pool the control ticks of the four adaptive runs.
Within-window acceptance uses matched images captured during the same
window as the control decision.

| New exposure | Transitions with a logged in-window decision | Median observed delay | Maximum observed delay |
| --- | ---: | ---: | ---: |
| Dark | 24/24 | 0.64 s | 1.20 s |
| Clean | 24/24 | 0.69 s | 1.39 s |

The transition observation is the first metrics snapshot that records
rejection of a paired dark frame or acceptance of a paired clean frame
for a robot after a transition. The delay includes the logging cadence.

All 12 instances of RGB use during dark windows came from buffered
clean images. The adaptive arm exceeded both controls on one of four
seeds. Its Hu-aligned mean was higher than both controls, with opposite
directions across the two seeds. On Layout C, the adaptive arm finished
behind LiDAR on both seeds. The seed-level results show that a reliable
camera gate can still produce variable exploration outcomes.

This study applies synthetic attenuation on two layouts with two seeds
each and fixed model weights from one training lineage. The observed
switching delays describe this simulation and sampling schedule.
The coverage results leave the effect across additional seeds, camera
conditions, and independent training lineages open.

See the [input manifest](../ros_gazebo/experiments/2026-11-22-mixed-exposure/paired_results/experiment_manifest.json),
[paired metrics](../ros_gazebo/experiments/2026-11-22-mixed-exposure/paired_results/paired_camera_differences.csv),
[per-window counts](../ros_gazebo/experiments/2026-11-22-mixed-exposure/paired_results/exposure_windows.csv),
[transition observations](../ros_gazebo/experiments/2026-11-22-mixed-exposure/paired_results/transition_observations.csv)
and [figure](../ros_gazebo/experiments/2026-11-22-mixed-exposure/paired_results/mixed_exposure.png).
