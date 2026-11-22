# Buffered RGB and LiDAR clearance study

The earlier three-seed comparison found less than 90% usable team input for
each fusion run. The new fusion policy tries the latest RGB and LiDAR
samples first. When those cannot form a pair, it recovers the newest
compatible pair from bounded histories. Both samples still must be present,
no more than 0.75 seconds old, and no more than 0.25 seconds apart. The
reported timestamps use the ROS control clock.

The metrics CSV counts accepted latest pairs, recovered buffered pairs,
final rejection causes and invalid directional scans. Per-robot diagnostics
report sample ages and matched timestamp differences at each statistics row.
The assignment trace includes the diagnostic state for the selected robot.
Input availability remains a measured outcome of the tested controller.

## Synchronization pilot result

Both conditions completed 300 simulation seconds in Hu-aligned with seed
`61124`. The buffered run used cached pairs on 181 of 3,594 robot ticks
and reported one missing input. Latest-only fusion rejected 588 robot
ticks for timestamp skew and one for a missing input.

| Outcome | Latest only | Buffered |
| --- | ---: | ---: |
| Team input availability | 83.24% | 99.92% |
| Final coverage | 81.96% | 78.21% |
| Time-averaged coverage | 63.94% | 54.80% |
| Path travelled | 151.38 m | 131.20 m |
| Stuck events | 28 | 36 |
| Near-collision notifications | 927 | 787 |

The pilot confirms the improvement in input availability for this seed.
Coverage and traced arrivals decreased. One pair cannot establish whether
the coverage change persists across seeds. The expanded study compares
buffered fusion with direct LiDAR to measure the contribution of adding RGB
under reliable pairing.

[Pilot run summary](../ros_gazebo/experiments/2026-11-22-buffered-clearance/pilot_run_summary.csv)
and [input fingerprints](../ros_gazebo/experiments/2026-11-22-buffered-clearance/pilot_manifest.json).

## Evaluation plan

| Stage | Conditions | Replication |
| --- | --- | --- |
| Synchronization pilot | Latest-only fusion and buffered fusion | One previously used Hu-aligned seed, `61124`, to reproduce the missing-cue condition. |
| Expanded study | Direct LiDAR and buffered RGB–LiDAR fusion | Five new seeds per layout across Hu-aligned and Layout C, with both conditions at each seed. |

The expanded schedule uses ten distinct seeds, `61200–61209`. Direct LiDAR
goes first in five pairs and fusion goes first in five. Both stages use
300 simulation seconds with fixed initial poses, a frozen MAPPO checkpoint,
a frozen DDPG controller and identical scanner and camera settings. The
study uses Gazebo poses for scan registration. It evaluates the frozen
controller and the two planned layouts. The expanded primary outcome is
the paired difference in final coverage within each layout. The report
also gives paired differences in time-averaged coverage, path length,
near-collision notifications, stuck recovery, input availability and traced
arrivals. Threshold summaries report how many runs reach 90% and 97%
coverage. Time differences use only pairs that both reach the threshold.

## Expanded study results

All 20 runs completed the 300-second simulation horizon. The audit recomputed
each result from its metrics CSV and frontier trace. It confirmed the recorded
schedule, source tree, map and checkpoint fingerprints.

Coverage values are mean ± sample standard deviation. The differences
use buffered fusion minus direct LiDAR within each seed and layout.

| Layout | Direct LiDAR final | Buffered fusion final | Mean paired difference | Range of paired differences | Fusion ahead | Mean time-averaged difference |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 87.22 ± 7.08% | 92.88 ± 6.63% | +5.66 pp | -5.13 to +15.66 pp | 4/5 | +1.99 pp |
| Layout C | 96.19 ± 2.78% | 83.48 ± 6.38% | -12.71 pp | -25.87 to -4.07 pp | 0/5 | -9.03 pp |

| Layout | Mean path difference | Mean stuck-event difference | Mean near-collision notification difference | Mean arrival difference |
| --- | ---: | ---: | ---: | ---: |
| Hu-aligned | +3.96 m | -3.4 | +7.4 | +6.0 |
| Layout C | -15.63 m | +2.8 | -5.4 | -11.2 |

The clearance detector counts near-collision notifications using a laser
range threshold and a separate reset threshold.

| Layout | Coverage threshold | Direct LiDAR reached | Fusion reached | Both reached | Mean time difference when both reached |
| --- | ---: | ---: | ---: | ---: | ---: |
| Hu-aligned | 90% | 1/5 | 4/5 | 1 | +6.25 s |
| Hu-aligned | 97% | 0/5 | 2/5 | 0 | — |
| Layout C | 90% | 5/5 | 0/5 | 0 | — |
| Layout C | 97% | 2/5 | 0/5 | 0 | — |

Threshold counts use all five matched seeds per layout. The conditional
time differences use the subset in which both runs reach the target.

### Interpretation

Buffered fusion raised final coverage in four of five Hu-aligned pairs.
In Layout C, direct LiDAR finished ahead in all five pairs. Buffered fusion
provided a complete team cue on 99.00–100.00% of control ticks. Layout C
also recorded 11.2 fewer traced arrivals per fusion run on average. The
trace identifies an execution difference alongside the coverage deficit.
The experiment changes the cue for both policy observations and rule
decisions, so it leaves the contribution of each pathway unresolved.

The estimates describe three robots in two layouts, five matched seeds per
layout, and a single frozen MAPPO and DDPG checkpoint pair. The 300-second
horizon describes fixed-budget performance. A single Hu-aligned pair
reached 90% in both conditions, which gives the only paired threshold-time
difference in this batch.

[Experiment manifest](../ros_gazebo/experiments/2026-11-22-buffered-clearance/expanded_results/experiment_manifest.json)
· [Per-run results](../ros_gazebo/experiments/2026-11-22-buffered-clearance/expanded_results/paired_runs.csv)
· [Paired differences](../ros_gazebo/experiments/2026-11-22-buffered-clearance/expanded_results/paired_differences.csv)
· [Coverage figure](../ros_gazebo/experiments/2026-11-22-buffered-clearance/expanded_results/paired_coverage.png)

## Run and resume

Use repository revision `a0725cdff9bd2a9c21b32a827e53048280ae11a8` and its
prepared ROS source tree to reproduce or audit the archived runs in this
report. The archive fingerprints the runner at that revision.

Build and source the prepared ROS workspace and activate its Python environment.
The following command assumes a working X display. On a headless machine,
prefix the Python command with `xvfb-run -a`. Set `ROS_PROJECT` to the
prepared source checkout and `STUDY_OUTPUT` to a new output path.

```sh
python3 scripts/run_clearance_study.py "$ROS_PROJECT" \
  ros_gazebo/experiments/2026-11-22-buffered-clearance/expanded_schedule.csv "$STUDY_OUTPUT"
```

Run the same command with the same arguments to resume. The runner compares
the source, schedule and input fingerprints against the existing manifest.
It audits completed runs again and writes every new attempt to a separate
directory. A shared lock enforces sequential Gazebo execution. The runner
records availability even when it is low. Its completion checks cover
simulation time, input evidence, metric consistency and assignment outcomes.
The summary uses completed seed pairs and reports condition statistics,
paired differences, and threshold counts. Use `--require-complete` for the
final report. Add `--verify-raw "$ROS_PROJECT"` to recompute each result from
the archived CSV and assignment trace with the original source tree and
runner. Supply a new summary directory.

```sh
python3 scripts/summarize_clearance_study.py "$STUDY_OUTPUT" "$SUMMARY_OUTPUT" \
  --require-complete --verify-raw "$ROS_PROJECT"
python3 scripts/plot_clearance_study.py "$SUMMARY_OUTPUT/paired_differences.csv" \
  "$SUMMARY_OUTPUT/paired_coverage.pdf"
```

Each dot in the figure represents one seed and layout. Positive differences
indicate higher coverage with buffered fusion.
