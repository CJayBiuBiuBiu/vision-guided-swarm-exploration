# Calibration data for adaptive clearance fusion

The frozen RGB encoder comes from the original training checkpoint
`3caa21c9b79254bda5cc957c90f99c23d6e6a50566a72ebb321fa1a8c6071fab`.
That checkpoint records `vision_dataset_v3/manifest_camera_aligned.csv`
with 600 training rows and 120 validation rows from `seed_3304`. An audit
of this manifest found 58 image paths referenced by multiple runs. Of
these, 45 had conflicting clearance labels. The training and validation
splits shared 24 image paths, affecting 24 of the 120 validation rows.
The archived manifest SHA-256 is
`a24e7f4adc2675293e48a620a552aa45c5c2001880a772883c4cc2991e9a7e68`.

The role ablation uses the same frozen weights in all four conditions,
so its contrasts describe this encoder. Its image-level validation
figures require the split-overlap qualification above. Adaptive fusion
calibration requires new run-isolated images and held-out evaluation runs.

## Record and audit

Source the prepared ROS workspace. During a dedicated data collection run,
call the collector with its actual layout and simulator seed. The script
creates a new directory for each run ID. It stores lossless camera frames,
SHA-256 fingerprints, camera and scan timestamps, and labels from the
runtime directional LiDAR scorer. It accepts images only when a scan
falls within the recorded timestamp tolerance.

```sh
python3 scripts/collect_clearance_calibration.py "$ROS_PROJECT" "$DATA_ROOT" \
  --run-id calibration_62011 --layout hu_aligned --seed 62011 --samples 120
```

The same image timestamp in two runs produces two separate files. The
held-out audit checks paths, file hashes, layout and seed groups, label
ranges, and the recorded synchronization. Pass the complete set of
manifests and repeat `--held-out-run` for each held-out run.

```sh
python3 scripts/audit_clearance_calibration.py \
  "$DATA_ROOT"/calibration_62011/manifest.csv "$DATA_ROOT"/held_out_62012/manifest.csv \
  --held-out-run held_out_62012
```

Run the audit before choosing any camera-quality threshold. Split by
whole runs and keep a separate evaluation set for the final gate.

## Live collection check

Two 70-second Gazebo runs collected 120 camera frames apiece. The
calibration run used Hu-aligned and seed `62011`. The held-out run used
Layout C and seed `62012`. The audit found zero shared image paths, image
hashes or layout–seed groups across the split. This sample verifies the
collection and audit workflow. The [capture audit](../ros_gazebo/experiments/2026-11-22-clearance-roles/calibration_capture_smoke_audit.json)
records the two manifest fingerprints.
