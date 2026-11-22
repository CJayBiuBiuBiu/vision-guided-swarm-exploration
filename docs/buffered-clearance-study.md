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

## Run and resume

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
The summary accepts completed seed pairs only. Supply a new summary directory.

```sh
python3 scripts/summarize_clearance_study.py "$STUDY_OUTPUT" "$SUMMARY_OUTPUT"
```
