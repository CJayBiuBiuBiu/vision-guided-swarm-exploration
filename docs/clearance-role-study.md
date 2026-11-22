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

## Formal results

All 16 runs completed the 300-second horizon. The audit reconstructed
both cue vectors at each traced assignment from the same LiDAR scan.
It also verified the schedule, source tree, models and maps.

| Layout | Both LiDAR | MAPPO fusion | Rule fusion | Both fusion | Actor effect | Rule effect | Interaction |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 70.08% | 92.99% | 86.61% | 87.29% | +11.80 pp | +5.41 pp | -22.23 pp |
| Layout C | 95.84% | 85.20% | 93.52% | 98.31% | -2.93 pp | +5.40 pp | +15.43 pp |

Each layout mean uses two complete seed blocks. Each effect is a paired
contrast in percentage points. An interaction measures how much the
effect of MAPPO fusion changes when the rules also use fusion.

| Layout | Seed | Both LiDAR | MAPPO fusion | Rule fusion | Both fusion | Interaction |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 61300 | 78.30% | 86.26% | 84.98% | 81.64% | -11.31 pp |
| Hu-aligned | 61302 | 61.86% | 99.73% | 88.23% | 92.95% | -33.15 pp |
| Layout C | 61301 | 95.44% | 79.52% | 96.16% | 97.19% | +16.95 pp |
| Layout C | 61303 | 96.24% | 90.88% | 90.88% | 99.43% | +13.91 pp |

### Interpretation

Using fusion only in MAPPO gave the highest final coverage in both
Hu-aligned seed blocks. Using fusion in both pathways led in both
Layout C blocks. The MAPPO-only condition trailed common-scan LiDAR by
15.92 and 5.36 percentage points in Layout C. Switching both pathways
to fusion raised coverage by 1.75 and 3.19 points in those two blocks.

The interaction had the same sign in each layout's two seed blocks.
The sign differed across layouts. The rule pathway changes frontier
ownership and the available actions in MAPPO, which is one mechanism
the next evaluation can test. The results compare four fixed settings
under one frozen encoder and policy pair. The earlier two-condition
study used different seeds and a latest-scan LiDAR baseline. This
experiment uses its own common-scan baseline.

[Formal manifest](../ros_gazebo/experiments/2026-11-22-clearance-roles/formal_results/experiment_manifest.json)
· [Per-run results](../ros_gazebo/experiments/2026-11-22-clearance-roles/formal_results/role_runs.csv)
· [Seed contrasts](../ros_gazebo/experiments/2026-11-22-clearance-roles/formal_results/seed_contrasts.csv)
· [Interaction figure](../ros_gazebo/experiments/2026-11-22-clearance-roles/formal_results/role_interaction.png)

## Run and verify

Use revision `03dc9f303d966f959303f4f261054ffba9b2e4ff` and its prepared
ROS tree to audit the archived role study. Its manifest fingerprints the
runner at that revision.

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
python3 scripts/plot_clearance_role_study.py "$SUMMARY_OUTPUT/seed_contrasts.csv" \
  "$SUMMARY_OUTPUT/role_interaction.pdf"
```

Each stage uses a distinct output directory. The runner preserves all
attempts. Repeating the same command verifies completed runs and starts
the remaining conditions.
