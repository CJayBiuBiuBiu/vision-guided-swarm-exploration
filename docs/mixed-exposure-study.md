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
