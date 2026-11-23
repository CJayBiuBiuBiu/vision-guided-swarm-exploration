# ROS/Gazebo experiments

This implementation supports the dissertation *Vision-Guided Multi-Agent
Reinforcement Learning for Swarm Robotic Exploration in Unknown Environments* by
Yujie Cao. It uses MAPPO for frontier selection in Gazebo. The project studies
frontier coordination, the effect of visual input, and recovery when a robot
gets stuck. The low-level controller uses the DDPG reproduction from the
[TurtleBot3 baseline](https://github.com/hanlinniu/turtlebot3_ddpg_collision_avoidance).
That repository credits Hanlin Niu and cites the prior work on DDPG obstacle
avoidance and Voronoi exploration.

The base snapshot is [`39ee535`](https://github.com/CJayBiuBiuBiu/turtlebot3_ddpg_collision_avoidance/tree/39ee535f99ba6e061b1d7db4658a03d009a61dcc),
which already contains the dissertation's Noetic adaptation and archived
experiment artifacts. The follow-up changes are stored as patches in this
project. The preparation script downloads the base and applies the patches in
stage order. The earlier ROS/Gazebo repository serves as an archive and the
source of that fixed base.

## Prepare the ROS workspace

On Ubuntu with ROS Noetic and Gazebo Classic, run this from the root of a clone
of this repository:

```sh
python3 scripts/prepare_ros_gazebo.py \
  "$HOME/catkin_ws/src/turtlebot3_ddpg_collision_avoidance"
```

Supply a new destination. The script prints the derived Git revision and tree
hash. Record the revision of this repository together with the tree hash when
reporting new experiments. Follow the
[Noetic environment and build instructions](https://github.com/CJayBiuBiuBiu/turtlebot3_ddpg_collision_avoidance/blob/39ee535f99ba6e061b1d7db4658a03d009a61dcc/MIGRATION.md)
from the base snapshot. The command above performs the checkout step.
The prepared project's
[`REPRODUCIBILITY.md`](https://github.com/CJayBiuBiuBiu/turtlebot3_ddpg_collision_avoidance/blob/39ee535f99ba6e061b1d7db4658a03d009a61dcc/REPRODUCIBILITY.md)
lists experiment commands and artifacts.

The [matched control-progress pilot](control-progress-pilot-2026-11-21.md)
records the first 300-second Gazebo check of the reference-coverage and
observed-area modes.

## Compare RGB and laser clearance

The three modes put the same left, centre and right clearance cue into the
existing frontier feature. Laser mapping and the low-level DDPG controller
operate in all three conditions.

| `vision_scoring_mode` | Directional cue |
| --- | --- |
| `lidar_openness` | Lower-quartile clearance from the current raw laser scan in each camera sector. |
| `learned_openness` | Single-frame prediction from the frozen dissertation RGB encoder. |
| `rgb_lidar_openness` | Per-sector mean of both cues with `clearance_rgb_weight:=0.5`. |

The laser score uses the scan's reported range limits and the same sectors,
quantile and normalisation as the RGB training labels. The mapping range
cap remains a separate control setting. The live score requires a usable
reading in each sector. Samples expire after 0.75 s and the fusion pair may
differ by at most 0.25 s. Fusion first checks the latest samples. If they
cannot pair, it checks the last 32 samples from each stream for a pair that
still meets both limits. Set `fusion_pairing_mode:=latest` for the earlier
pairing rule. An unavailable required cue contributes zero openness for
that control tick.

See the [buffered clearance study](buffered-clearance-study.md) for the
synchronization pilot, matched schedules and result checks.
Use `vision_scoring_mode:=matched_directional_clearance` to assign LiDAR or
RGB–LiDAR fusion independently to MAPPO and the frontier rules. The
[clearance role study](clearance-role-study.md) describes the independent
settings and common scan.
The [adaptive clearance study](adaptive-clearance-study.md) adds a quality
profile and a LiDAR fallback for underexposed camera frames.
The [mixed-exposure study](mixed-exposure-study.md) checks how that gate
switches as the camera exposure changes within a run.
The [depth corridor pilot](depth-corridor-study.md) compares camera depth
with the horizontal scan on obstacles below the scan plane.
The [active recheck study](active-depth-recheck.md) tests an additional
waypoint check while a robot is following its current assignment.
The [matched vision training study](matched-vision-training.md) trains two
MAPPO actors independently with the same budget and observation mask.
The [training extension](vision-training-extension.md) follows the same
lineages through fixed checkpoints and a new held-out test.

The RGB modes accept `learned_vision_checkpoint` as an absolute path. Use
[`rgb_clearance_inference.pt`](../ros_gazebo/checkpoints/rgb_clearance_inference.pt)
to reproduce the original encoder. This file retains its model weights and
input and output format. Its SHA-256 is
`3ddcf69b973bef4aebaaddddc15e537bc25cd82b8c024fe5a797f082bd6f8df5`.
The original training checkpoint has SHA-256
`3caa21c9b79254bda5cc957c90f99c23d6e6a50566a72ebb321fa1a8c6071fab`.

After building and sourcing the ROS workspace, run the following from the
formal repository root to give a Gazebo launch a specific modality. Set
`ROS_PROJECT` to the prepared ROS/Gazebo checkout.

```sh
RUN_DIR=$(mktemp -d)
roslaunch turtlebot_ddpg gazebo_research_experiment.launch \
  selection_mode:=mappo_hybrid start_network_policy:=true \
  mappo_checkpoint:="$ROS_PROJECT/research_artifacts/dissertation_v1/checkpoint/mappo_selected_validation_v1.pt" \
  vision_scoring_mode:=rgb_lidar_openness \
  learned_vision_checkpoint:="$PWD/ros_gazebo/checkpoints/rgb_clearance_inference.pt" \
  max_simulation_seconds:=300 fixed_horizon_only:=true shutdown_on_done:=true \
  metrics_path:="$RUN_DIR/metrics.csv"
```

For a frozen-policy comparison, hold the MAPPO and DDPG checkpoints, seed,
map, initial poses, progress mode and simulation horizon fixed. Use
`vision_role:=both`, change only the cue source, and save each condition
to its own CSV. The CSV records the selected mode and the number of
control ticks with a valid cue for every robot. In observed-area mode, a
distinct `frontier_trace_path` records the cue and projected bonus for
each assigned goal along with its eventual outcome. The logged cue describes
the coordinator state at assignment time. The projected bonus precedes
feature scaling and role-specific routing.

On 22 November 2026, all three modes completed a 20-second Gazebo smoke
test with seed 61122. The runs confirmed three-robot input availability,
policy actions and matched assignment/outcome records. These checks establish
that the modes run through the same navigation stack. The 300-second
comparisons will measure exploration performance.

The [three-seed directional-clearance comparison](directional-clearance-2026-11-22.md)
records the matched 300-second runs and the fusion input availability issue.
