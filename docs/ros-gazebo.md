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
