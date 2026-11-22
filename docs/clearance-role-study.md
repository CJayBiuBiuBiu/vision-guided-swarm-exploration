# Clearance cue ablation by decision pathway

The preceding study found different coverage effects across two layouts.
It changed both the policy's observations and the rules that assign and
rank candidate frontiers. It also compared a latest-scan LiDAR cue with
a fusion cue that can use a buffered scan. This study isolates the two
ways RGB enters the decision with a common LiDAR sample.

Set `vision_scoring_mode:=matched_directional_clearance` and
`vision_role:=both`. Set `clearance_actor_cue` and
`clearance_rules_cue` independently to `lidar` or `fusion`.
Every condition runs the RGB encoder and timestamp matcher. Within each
control tick, both pathways use the same selected LiDAR scan. When the
matcher cannot pair the streams, both cues use the latest fresh LiDAR
scan. If it is unavailable, both openness terms are zero. The coordinator
records the common scan timestamp and source values alongside the actor
and rule scores in each assignment trace. The runner reconstructs those
scores during validation.

The actor setting affects its openness feature and the cost feature it
receives. The rules setting affects frontier ownership and ranking. The
ownership mask still constrains the actor, so a rules change can also
change the set of actions available to the actor.

| Condition | MAPPO observation | Frontier rules |
| --- | --- | --- |
| `matched_lidar_lidar` | LiDAR | LiDAR |
| `matched_fusion_lidar` | RGB–LiDAR fusion | LiDAR |
| `matched_lidar_fusion` | LiDAR | RGB–LiDAR fusion |
| `matched_fusion_fusion` | RGB–LiDAR fusion | RGB–LiDAR fusion |

Four 20-second Gazebo smoke runs passed the cue reconstruction check
under seed `61390`. The [smoke manifest](../ros_gazebo/experiments/2026-11-22-clearance-roles/smoke_manifest.json)
records the inputs and source tree.

The formal schedule uses two new seeds per layout and four conditions
per seed, for 16 runs of 300 simulation seconds. Each condition occupies
each order position once across the four seed blocks. The runs share the
same frozen MAPPO, DDPG and RGB weights, robot positions and simulator
settings. The primary outcome is final coverage. Secondary outcomes are
time-averaged coverage, traced arrivals and outcomes, stuck events,
near-collision notifications, and distance travelled.

The [calibration data audit](clearance-calibration-data.md) records a
training–validation image overlap in the historical RGB dataset. The role
contrasts use that same encoder in all four conditions.

The actor effect averages the actor-only change under each fixed rule
cue. The rule effect averages the rule-only change under each fixed
actor cue. Their interaction measures how the actor effect changes when
the rule cue changes. Report these contrasts per seed and layout. Two
seeds per layout provide a mechanism screen for a later independent
evaluation.

## Run and verify

Build and source the prepared ROS workspace and activate its Python
environment. Set `ROS_PROJECT` to the prepared source tree. On a
headless host, prefix the runner with `xvfb-run -a`.

```sh
study=ros_gazebo/experiments/2026-11-22-clearance-roles
python3 scripts/run_clearance_study.py "$ROS_PROJECT" \
  "$study/smoke_schedule.csv" "$SMOKE_OUTPUT" --horizon 20
python3 scripts/run_clearance_study.py "$ROS_PROJECT" \
  "$study/role_schedule.csv" "$STUDY_OUTPUT" --horizon 300
python3 scripts/summarize_clearance_role_study.py "$STUDY_OUTPUT" "$SUMMARY_OUTPUT" \
  --verify-raw "$ROS_PROJECT"
```

Each stage uses a distinct output directory. The runner preserves all
attempts. Repeating the same command verifies completed runs and starts
the remaining conditions.
