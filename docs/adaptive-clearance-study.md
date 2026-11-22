# Camera-quality gated clearance

The coordinator selects one camera and LiDAR pair per control tick. It
measures mean luminance and Laplacian variance at the encoder input scale.
With an accepted frame, the adaptive cue uses the existing 50:50 fusion.
With an unusable frame, it uses the common LiDAR cue. A missing pair uses
the latest fresh LiDAR scan under the matched-clearance rules.

Each decision pathway can use the adaptive cue. The initial comparison
applies the same setting to both the policy and the frontier rules. This
isolates the quality gate from the choice of pathway. All conditions run
the same encoder and log the same camera-quality information. In the
stress condition, the coordinator multiplies camera pixel values by 0.28
before feature extraction and RGB inference.

## Calibration

Collect 180 frames per run at a 1.3-second per-robot interval for the
[eight scheduled captures](../ros_gazebo/experiments/2026-11-22-camera-quality/capture_schedule.csv).
Four runs fit the thresholds. Four independent runs check them. Each
split includes both layouts. The source audit can create a fresh view
that excludes exact image duplicates from both splits. The raw capture
manifests and frames remain available for verification. The calibration
audits the input view separately. It evaluates clean frames and synthetic
dark and blurred copies. Threshold candidates come from the first ten percentiles of clean
fit-run features and zero. The selected pair maximizes mean stress
rejection across fit runs and corruptions, with at least 95% clean
acceptance in each fit run. Ties favor higher clean acceptance and lower
thresholds. Held-out frames provide a separate check.

The paired Gazebo study targets underexposure. The offline evaluation
also reports blur separately because camera appearance and motion blur
can overlap in these features. The calibration uses simulated cameras.

```sh
python3 scripts/audit_clearance_calibration.py "$RAW_DATA_ROOT"/*/manifest.csv \
  --held-out-run held_out_62102 --held-out-run held_out_62103 \
  --held-out-run held_out_62106 --held-out-run held_out_62107 \
  --isolated-output "$DATA_ROOT"
python3 scripts/calibrate_clearance_quality.py "$ROS_PROJECT" "$DATA_ROOT" "$CALIBRATION_OUTPUT" \
  --held-out-run held_out_62102 --held-out-run held_out_62103 \
  --held-out-run held_out_62106 --held-out-run held_out_62107
```

The eight captures yielded 1,440 frames. Three image hashes occurred in
both splits. The isolated view removes the nine associated rows and
retains 1,431 frames, including 715 held-out frames. A second audit
confirmed zero shared paths, image hashes, or layout–seed groups.

The chosen minimum luminance is 54.25 on a 0–255 scale. The minimum
Laplacian variance is 4.37. Across the two held-out runs per layout, the
gate retained 99.7% of clean frames in Hu-aligned and 97.7% in Layout C,
averaging runs equally. It rejected all darkened copies in these runs.
It accepted 93.8–100.0% of the blurred copies by run. These measurements
describe the tested corruptions and camera scale.

[Frozen quality profile](../ros_gazebo/experiments/2026-11-22-camera-quality/calibration_results/camera_quality.json)
· [Per-run offline rates](../ros_gazebo/experiments/2026-11-22-camera-quality/calibration_results/camera_gate_metrics.csv)
· [Isolation audit](../ros_gazebo/experiments/2026-11-22-camera-quality/calibration_results/isolated_audit.json)

## Paired Gazebo test

The smoke test exercises six settings. The pilot uses two layouts and
one new seed per layout for 12 runs of 300 simulation seconds. Each seed
uses common-scan LiDAR, fixed fusion and adaptive fusion under clean
and darkened RGB. The second layout reverses the six-condition run order.
All conditions use the same model weights, start poses and 300-second
budget. The primary outcome is final coverage. The summaries also log
time-averaged coverage, arrivals, stuck counts and the fraction of robot
ticks using RGB.

The run audit reconstructs both cue vectors from the trace, checks the
logged gate decision against the frozen thresholds, and reconciles the
camera-quality counts with the RGB–LiDAR pairing counts.

The pilot start requires at least 85% clean acceptance and 95% dark
rejection in each held-out layout, and validates adaptive RGB use of at
least 70% for clean and at most 5% for dark in the smoke run.

```sh
study=ros_gazebo/experiments/2026-11-22-camera-quality
profile="$CALIBRATION_OUTPUT/camera_quality.json"
python3 scripts/run_clearance_study.py "$ROS_PROJECT" "$study/smoke_schedule.csv" "$SMOKE_OUTPUT" \
  --horizon 20 --quality-profile "$profile"
python3 scripts/run_clearance_study.py "$ROS_PROJECT" "$study/paired_schedule.csv" "$STUDY_OUTPUT" \
  --quality-profile "$profile"
python3 scripts/summarize_clearance_quality_study.py "$ROS_PROJECT" "$STUDY_OUTPUT" "$profile" "$SUMMARY_OUTPUT"
```

Build and source the prepared workspace first. On headless hosts, prefix
the run commands with `xvfb-run -a`. Repeating a run command with identical
inputs resumes the schedule and preserves existing attempts.

## Verified results

All twelve scheduled runs completed 300 simulation seconds. The audit
reconstructed each traced assignment and verified the exact inputs.
Gazebo failed during the first launch. The next attempt used the same
condition and seed. The archive retains the failed launch logs.

| Layout | RGB state | LiDAR | Fixed fusion | Adaptive fusion | Adaptive − LiDAR | Adaptive − fixed | RGB used by adaptive |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | clean | 80.77% | 86.17% | 82.33% | +1.56 pp | -3.85 pp | 96.75% |
| Hu-aligned | dark | 89.74% | 90.38% | 79.62% | -10.12 pp | -10.76 pp | 0.00% |
| Layout C | clean | 90.43% | 89.06% | 84.12% | -6.31 pp | -4.94 pp | 97.91% |
| Layout C | dark | 89.40% | 83.13% | 92.74% | +3.34 pp | +9.61 pp | 0.00% |

Coverage is measured at the end of the run. RGB use is the fraction of robot
control ticks that applied the camera contribution in the adaptive arm.
The six-run smoke test recorded 99.6% RGB use with clean images and 0.0%
with darkened images in its adaptive conditions.

## Held-out encoder diagnostic

After freezing the gate, the diagnostic compared the RGB model's
three scores with the matched LiDAR-derived labels on the 715
held-out frames. The table gives mean absolute error by layout.
Each value gives both held-out runs equal weight.

| Layout | Clean | Darkened | Blurred |
| --- | ---: | ---: | ---: |
| Hu-aligned | 0.2391 | 0.2933 | 0.2391 |
| Layout C | 0.2197 | 0.2766 | 0.2197 |

The tested darkening increased RGB prediction error in both layouts.
Blur changed the rounded error by less than 0.0001. The matched LiDAR
labels measure directional agreement with the scanner. The diagnostic
uses the frozen encoder throughout.

In both dark adaptive runs, the gate applied LiDAR at every control tick.
The observed coverage change relative to fixed fusion was negative in
Hu-aligned and positive in Layout C. The dark LiDAR and dark adaptive
conditions both use the LiDAR cue. Their coverage differences show that
the individual trajectories vary even when the cue is the same. The
closed-loop pilot estimates one seed per layout and uses synthetic
underexposure. The evidence supports the measured fallback behavior
and gives a reason to replicate coverage across more seeds.

[Formal manifest](../ros_gazebo/experiments/2026-11-22-camera-quality/paired_results/experiment_manifest.json)
· [Per-run results](../ros_gazebo/experiments/2026-11-22-camera-quality/paired_results/quality_runs.csv)
· [Offline prediction error](../ros_gazebo/experiments/2026-11-22-camera-quality/paired_results/camera_prediction_error.csv)
· [Coverage figure](../ros_gazebo/experiments/2026-11-22-camera-quality/paired_results/adaptive_clearance.png)

```sh
python3 scripts/diagnose_clearance_camera.py "$ROS_PROJECT" "$DATA_ROOT" "$profile" \
  ros_gazebo/checkpoints/rgb_clearance_inference.pt "$DIAGNOSTIC_OUTPUT"
python3 scripts/plot_clearance_quality_study.py "$SUMMARY_OUTPUT/paired_camera_differences.csv" \
  "$SUMMARY_OUTPUT/adaptive_clearance.pdf"
```
