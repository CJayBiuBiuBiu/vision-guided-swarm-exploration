# Matched MAPPO training to 16 and 32 updates

The [eight-update pilot](matched-vision-training.md) began two separate
MAPPO lineages per training seed. This extension restores each actor,
critic and their optimizers from its verified pilot checkpoint. Every
continuation episode uses a new environment seed and a new sampling
seed shared by the two arms. Each episode contributes two updates of
12 team transitions. Both arms use the same rules, action mask,
decision interval, mapping, frozen RGB encoder and DDPG controller.
The actor receives LiDAR clearance or fixed RGB–LiDAR fusion as
specified by its original lineage.

The [frozen plan](../ros_gazebo/experiments/2026-11-23-vision-training-extension/plan.json)
defines three stages before training. At 16 and 32 updates, we replay
the pilot's two 65-second evaluation cases under the same conditions.
These validation cases give comparable coverage measurements at 8,
16 and 32 updates. At 32 updates, six new cases form the final test.
Each layout uses three new seeds at 120 seconds. Training, validation
and final test seed ranges are disjoint. The endpoint budget is
prespecified. Validation does not select a checkpoint.

| Stage | New training runs | Evaluation runs | Updates and transitions per lineage |
| --- | ---: | ---: | ---: |
| 16-update checkpoint | 24 | 12 | 16 and 192 |
| 32-update checkpoint | 48 | 12 | 32 and 384 |
| Final 120 s test | 0 | 36 | 32 and 384 |

The runner reads the archived pilot and its published endpoint
fingerprints. It records the parent checkpoint and verifies the
loaded tensor hash on every episode against the preceding accepted
model. It also checks that both trained optimizers have saved state.
Each attempt saves its own logs and endpoint. This allows resuming
without replacing accepted outputs and preserves evidence of failed
launches. A shared lock serializes Gazebo use.

## Run and audit

Prepare the ROS source and environment as in the pilot. Supply the
source tree that produced the eight-update source manifest. Use the
pilot's raw directory as the first parent. The script compares source,
sensor, map and DDPG hashes with that pilot.

```sh
runner=scripts/run_vision_training_extension.py
python3 "$runner" update_16 "$ROS_PROJECT" "$PILOT_OUTPUT" "$EXTENSION_OUTPUT/update_16"
python3 "$runner" update_32 "$ROS_PROJECT" "$EXTENSION_OUTPUT/update_16" "$EXTENSION_OUTPUT/update_32"
python3 "$runner" test_32 "$ROS_PROJECT" "$EXTENSION_OUTPUT/update_32" "$EXTENSION_OUTPUT/test_32"
```

Use `xvfb-run -a` on headless hosts. Run the same command with
`--audit-only` to verify accepted runs without launching Gazebo.

## 16-update validation

All 24 continuation episodes and 12 validation runs passed. Each
lineage reached 16 total updates and 192 team transitions. The parent
checkpoint and the loaded weight fingerprint matched at every step.

| Layout | Updates | LiDAR coverage | RGB–LiDAR coverage | Mean paired difference |
| --- | ---: | ---: | ---: | ---: |
| Hu-aligned | 8 | 51.63% | 46.83% | −4.81 pp |
| Hu-aligned | 16 | 48.84% | 49.48% | +0.64 pp |
| Layout C | 8 | 41.17% | 38.73% | −2.44 pp |
| Layout C | 16 | 48.53% | 36.26% | −12.27 pp |

The Hu-aligned gap narrowed and changed direction. The Layout C gap
grew. Each comparison uses three paired lineages on one fixed
validation seed per layout. The 32-update stage follows the frozen
schedule.

[Validation pairs](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_16_validation_pairs.csv)
· [Checkpoint fingerprints](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_16_lineage_endpoints.csv)
· [Validation figure](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_16_vision_training_budgets.png).

## 32-update validation

The six lineages completed the additional 48 episodes, reaching 32
updates and 384 team transitions each. All 12 fixed-case evaluations
passed. Coverage means use the three training seeds per layout.

| Layout | LiDAR coverage | RGB–LiDAR coverage | Mean paired difference | RGB–LiDAR ahead |
| --- | ---: | ---: | ---: | ---: |
| Hu-aligned | 51.01% | 48.64% | −2.37 pp | 1/3 |
| Layout C | 51.05% | 41.38% | −9.68 pp | 1/3 |

In Hu-aligned, the positive difference at 16 updates did not persist.
In Layout C, the difference remained negative across all three
budgets. The next test uses the prespecified 32-update endpoints
and the six reserved environment seeds.

[Validation across budgets](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_32_validation_pairs.csv)
· [32-update checkpoints](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_32_lineage_endpoints.csv)
· [Manifest](../ros_gazebo/experiments/2026-11-23-vision-training-extension/update_32_manifest.json).
