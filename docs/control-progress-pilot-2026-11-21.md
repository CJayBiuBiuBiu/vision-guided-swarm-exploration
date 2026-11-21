# Matched Gazebo control-progress pilot

On 21 November 2026, two runs used the Hu-aligned arena, three TurtleBot3 robots,
and Gazebo and experiment seed `61121`. Both runs ended at 300 simulated
seconds. They used the same starting poses, frozen MAPPO and DDPG weights,
laser mapping, Gazebo ground-truth poses and camera scoring. The reference
condition ran first. The second run selected `observed_free_area` as its
control-progress mode. Early completion, tail completion and frontier trace
logging were disabled for both conditions.

| Metric | Reference coverage | Observed free area |
| --- | ---: | ---: |
| Final reference coverage | 77.84% | 84.16% |
| Mean coverage over 300 s | 62.70% | 64.33% |
| Total path length | 134.78 m | 120.26 m |
| Laser-based near-collision events | 911 | 885 |
| Stuck events | 30 | 59 |
| Successful recoveries / reported recoveries | 22 / 30 | 40 / 59 |
| Hybrid fallback activations | 0 | 0 |
| Time to 90% and 97% coverage | Neither threshold reached | Neither threshold reached |

The observed-area condition covered 6.32 percentage points more at 300 s and
travelled 14.52 m less. It reported 29 more stuck events. Both conditions
recorded 360 policy actions and received laser and camera data from all three
robots. Near-collision counts track laser clearance threshold crossings.

This single matched seed with a fixed execution order gives a feasibility
check. The mode change also zeros the legacy coverage input to the frozen
actor, and neither run activated the fallback. The pilot therefore leaves
the cause of the coverage difference unresolved. Multiple matched seeds and
a separate test of the actor coverage input would help distinguish those
effects.

## Recorded inputs

| Input | Fingerprint |
| --- | --- |
| Formal repository revision | `fe29e8e00c9d0b5b05d945fec54fe2e12970c26d` |
| Prepared ROS/Gazebo tree | `38794ee97d009a01b34b16c6ce98c3b34f0fdfab` |
| Frozen MAPPO checkpoint SHA-256 | `a3ba6f3a29ea70c39788ddfb9ab9aabf4d9fc07ab7e6a0a3c277bba29e443dc9` |
| DDPG actor SHA-256 | `2eae93434e7cc755f15992a73edec561ae2569e0573c4b9faf6b4d85a443bc21` |
| World SHA-256 | `ce359b4f610a93165142bb39546dc4a7d631c36cd9c48a9b967ddfdd1e81d0c7` |
| Reference coverage mask SHA-256 | `b18fec85fc3a188df3297d6c581c70c341fca72002c2a104c04359e7a298f3a5` |
