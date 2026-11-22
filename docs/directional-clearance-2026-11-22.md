# Matched directional clearance in Gazebo

On 22 November 2026, each of three matched seeds ran all three clearance
conditions for 300 simulation seconds. The runs used three TurtleBot3 robots
in the Hu-aligned arena. Initial poses, laser mapping, Gazebo pose registration,
the frozen MAPPO policy and the frozen DDPG controller stayed fixed. The
conditions changed the cue supplied to both the policy and the rules. The
fusion mode weighted RGB and LiDAR equally. The planned order rotated across
seeds.

## Results

| Seed | Direct LiDAR | Learned RGB | RGB + LiDAR | Fusion minus LiDAR |
| --- | ---: | ---: | ---: | ---: |
| 61123 | 81.41% | 77.79% | 88.51% | +7.10 pp |
| 61124 | 85.90% | 81.73% | 91.94% | +6.04 pp |
| 61125 | 90.11% | 91.16% | 76.24% | −13.87 pp |
| Mean | 85.81% | 83.56% | 85.56% | −0.24 pp |
| Sample SD | 4.35 pp | 6.87 pp | 8.26 pp | — |

| Metric | Direct LiDAR | Learned RGB | RGB + LiDAR |
| --- | ---: | ---: | ---: |
| Mean total path | 129.23 m | 131.52 m | 141.78 m |
| Mean stuck events | 38.67 | 40.00 | 32.33 |
| Mean laser near-collision events | 873.67 | 866.33 | 889.33 |
| Mean arrival rate among closed trace attempts | 62.29% | 66.91% | 57.59% |
| Reached 90% coverage | 1/3 | 1/3 | 1/3 |
| Reached 97% coverage | 0/3 | 0/3 | 0/3 |

Arrival rates group outcomes by assignment ID. Attempts still active at the
simulation horizon are censored. The study averages each run's rate equally.
The fusion condition finished above direct LiDAR on two seeds. The reversal on
the third seed left mean final coverage nearly equal. Fusion used a longer
mean path and recorded fewer stuck events. The experiment uses one arena,
three seeds and one fixed policy, which limits the strength of any ranking.

## Input availability and recorded exceptions

All nine planned first attempts completed the horizon and passed the metric
and trace checks. The fraction of control ticks with a usable cue for all
three robots ranged from 99.7% to 100% for direct LiDAR, from 96.5% to 100%
for learned RGB, and from 80.0% to 89.6% for fusion. Every fusion run fell
below the original 90% readiness threshold. The deployed fusion mode
contributes zero openness for a robot when the required streams fail its
timing checks. This makes availability part of the performance of the
implemented condition. The data leave open how continuous fusion would rank.

The original gate stopped the batch after the first fusion attempt for
seed 61123. A diagnostic retry reached 88.97% coverage with 88.5% readiness.
The continuation kept the original launch settings and treated readiness as
a qualification flag. The tables use the first completed attempt for every
condition and exclude the diagnostic retry. The raw runs and logs retain
both attempts.

The guest system clock stepped forward during the direct-LiDAR run for
seed 61124. This run completed its simulation horizon. The table and
threshold times use simulation time, and the wall-time jump stays visible
in the recorded data.

## Evidence

The tested release was [`9aa3359`](https://github.com/CJayBiuBiuBiu/vision-guided-swarm-exploration/commit/9aa3359).
The prepared ROS/Gazebo tree was `c076d4d8e1428cf6ffcf9bbb2f383a68ec9a3c36`.
The [experiment manifest](../ros_gazebo/experiments/2026-11-22-directional-clearance/experiment_manifest.csv)
contains the model, world and launch fingerprints. The
[run schedule](../ros_gazebo/experiments/2026-11-22-directional-clearance/run_schedule.csv)
records the planned order. The
[continuation record](../ros_gazebo/experiments/2026-11-22-directional-clearance/continuation_manifest.csv)
identifies the analysis population and validator used to finish the batch.
The [per-run summary](../ros_gazebo/experiments/2026-11-22-directional-clearance/run_summary.csv)
contains readiness, coverage, paths and trace outcomes for each condition.
