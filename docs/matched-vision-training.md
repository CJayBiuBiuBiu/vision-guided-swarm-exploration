# Training MAPPO with matched LiDAR and RGB inputs

This experiment tests whether a frontier policy learns differently when
its input includes RGB. Both arms run the frozen RGB encoder and the
same buffered camera–scan matcher. The LiDAR arm gives the actor the
common-scan clearance cue. The fusion arm gives the actor a 50:50
blend of RGB-predicted and common-scan clearance when a pair is
available. Both use common-scan clearance otherwise. The ownership
and ranking rules use common-scan LiDAR in both arms.

Both arms use the same MAPPO architecture, shared observation adapter
and state-dependent action mask at training and inference. The
reference coverage field is omitted from the actor and critic inputs
in both phases. The simulator reference coverage supplies the training
reward and evaluation metric. Both arms share scanning, mapping,
Gazebo poses, the DDPG weights and the rule configuration. The
RGB encoder is the same frozen encoder used in earlier studies.
Its [historical data audit](clearance-calibration-data.md) describes
overlap in the original encoder-validation split.

Each paired training seed starts two independent optimizers and
networks from scratch. The audit checks identical initial actor and
critic fingerprints within each pair. Each episode uses shared
simulator and sampling seeds across its two arms. The sampling seed
is applied after model initialization or optimizer restore.
Evaluation seeds are disjoint from model, environment and sampling
seeds used in training. Each arm evaluates its own endpoint under
its training input profile.

The fixed-budget runner caps the number of updates in each episode.
It verifies the update sequence, the transition count of each rollout
and the final checkpoint metadata. Each accepted episode has its own
checkpoint, metrics and diagnostics. Retrying uses a new attempt
directory and copies the prior accepted endpoint. The evaluation only
starts after all training episodes verify.

## First matched batch

The smoke plan checks initial random weights, the checkpoint chain
across two episodes, and endpoint loading for both arms. The matched
plan uses three paired training seeds. Each arm runs four 52 s
episodes with two updates per episode and 12 team transitions per
update. Both phases use a 1.5 s decision interval. The training
budget is eight updates or 96 team transitions per arm and seed.
The simulation horizon is fixed, including time after the last update.

Each endpoint runs twice for 65 s. The first evaluation uses an
unseen seed in Hu-aligned. The second uses another unseen seed in
Layout C, which is absent from this batch's training. Each evaluation
case uses both arms under the same conditions. This initial budget
checks the protocol and estimates early-training differences. A
learning-curve plateau and robust generalization require larger
budgets and additional independent seed blocks.

## Run and verify

Prepare and build the ROS checkout and source its environment. Use
distinct output directories. On a headless host, prefix the commands
with `xvfb-run -a`.

```sh
study=ros_gazebo/experiments/2026-11-23-matched-vision
python3 scripts/run_matched_vision_training.py "$ROS_PROJECT" "$study/smoke_plan.json" "$SMOKE_OUTPUT"
python3 scripts/run_matched_vision_training.py "$ROS_PROJECT" "$study/paired_plan.json" "$STUDY_OUTPUT"
```

The commands use the same Gazebo lock as the clearance experiments.
The manifest records the expected schedule and source, map, adapter
and model hashes. Repeating a command with identical inputs reaudits
the completed chain and runs the next pending episode.

## Smoke validation

The ROS build and 119 Python tests passed. The two arms shared the
same initial model fingerprint for seed 62690. Each trained for
two 50 s episodes and exactly four updates. The audit verified
48 team transitions per arm, the checkpoint handoff into episode
two, and that each frozen endpoint loaded with reference coverage
omitted from its observation. Both completed the 45 s evaluation
on unseen seed 65800. The coverage values were 40.38% for LiDAR and
32.65% for RGB–LiDAR.

[Smoke manifest](../ros_gazebo/experiments/2026-11-23-matched-vision/smoke_experiment_manifest.json)
· [Smoke training](../ros_gazebo/experiments/2026-11-23-matched-vision/smoke_training_runs.csv)
· [Smoke evaluation](../ros_gazebo/experiments/2026-11-23-matched-vision/smoke_evaluation_runs.csv).
