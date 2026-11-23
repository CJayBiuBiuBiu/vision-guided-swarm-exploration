# Rechecking the depth corridor during navigation

The [low-obstacle depth pilot](depth-corridor-study.md) tested whether a
depth camera could change a new frontier assignment. One robot contacted
a low box while following an active assignment. This study tests what
happens when the coordinator also rechecks the last commanded waypoint
as the robot moves.

Both arms use the existing depth check for new assignments. Both log the
same active-waypoint check. The active arm uses that check to interrupt
a route and choose another owned frontier.

| Condition | Active-waypoint check |
| --- | --- |
| Assignment only (`avoid`) | Log confirmed obstructions |
| Active recheck (`recheck`) | Log confirmed obstructions and interrupt the assignment |

A return must fall in the original low-height corridor. The monitor
confirms a hit after two distinct fresh depth frames point to locations
within 0.28 m of one another. The stamps must be at most 0.50 s apart.
The monitor checks an existing command before sending the next waypoint.
A confirmed obstruction closes the current frontier attempt. The
coordinator searches the owned frontiers in rule cost order and uses
the depth check to test the proposed replacement. It excludes the
interrupted frontier ID and candidates within 0.30 m for 3 s.
The coordinator can retry the same frontier after the exclusion expires.
The existing follower receives a hold when the coordinator has no
replacement. It pauses motion and the stuck timer during a hold. The
next waypoint resumes exploration. The scan mapper, robot model,
and learned weights are shared across the arms.

The coordinator records the depth and optional paired LiDAR evidence,
the waypoint and the interruption identifier. The auditor reconstructs
the two-frame decision and links the original outcome, replacement
assignment and replacement outcome. It checks each run reached the
fixed simulated horizon. Counts for physical contacts come from the
Gazebo boxes' contact sensors.

## Reproduce

Prepare and build the ROS workspace. Source its environment and use
the same checkpoint files for both arms. On a headless machine, prefix
the commands with `xvfb-run -a`.

```sh
study=ros_gazebo/experiments/2026-11-23-active-depth
python3 scripts/run_depth_corridor_study.py "$ROS_PROJECT" "$study/smoke_schedule.csv" "$SMOKE_OUTPUT" --active-check --horizon 40
python3 scripts/run_depth_corridor_study.py "$ROS_PROJECT" "$study/paired_schedule.csv" "$STUDY_OUTPUT" --active-check --horizon 65
```

The smoke block verifies that both modes complete and emit traceable
active checks. The study uses four new seed blocks. Three contain
9 cm low boxes and one is the clear reference. The arms alternate
which runs first. Compare contacts, paired final coverage, the number
of interrupted goals, and how many replacements reach their frontiers.
A reroute shows a decision effect. Contact differences across these
few runs also include simulator trajectory variability.

Each directory has a separate manifest and retains incomplete attempts.
Re-running the same command audits completed runs and starts at the
next condition. For exact replay of the earlier assignment-only
pilot, use repository revision `738c3f7` and its prepared ROS tree.

## Smoke test

The two 40 s runs at seed 62590 completed after the ROS build and
118 Python tests passed. The assignment-only arm logged one two-frame
confirmation beside a low box. Its paired laser scan did not hit
that corridor. The active arm recorded no confirmation in this seed.
The smoke block checks the logging and execution path under both modes.

[Smoke summary](../ros_gazebo/experiments/2026-11-23-active-depth/smoke_summary.csv)
· [Smoke manifest](../ros_gazebo/experiments/2026-11-23-active-depth/smoke_manifest.json).

## Paired results

All eight runs reached the 65 s horizon. The audit independently
reconstructed active decisions and checked the frontier outcome trace.

| Scene and seed | Assignment-only coverage | Active recheck coverage | Low-box contacts, assignment only / recheck | Active interruptions |
| --- | ---: | ---: | ---: | ---: |
| Low boxes, 62500 | 46.89% | 31.32% | 0 / 8 | 3 |
| Low boxes, 62502 | 34.62% | 38.42% | 3 / 1 | 0 |
| Clear reference, 62501 | 42.58% | 36.81% | 0 / 0 | 0 |
| Low boxes, 62504 | 42.45% | 46.25% | 0 / 1 | 0 |

All three active interruptions occurred in seed 62500. Two had a fresh
paired LiDAR scan with no hit in the sampled corridor. The coordinator
retried the same frontier after the exclusion window in the first two
cases, at 4.50 and 3.50 s. In the third case, it immediately replaced
frontier 24 with frontier 20. Frontier 20 remained active at the horizon.
No post-interruption assignment reached its target within the run.

The active check changed an existing goal in this setting. The tested
response did not demonstrate a contact or coverage benefit. In particular,
the only seed with interruptions had lower coverage and more contacts.
The other three differences occurred without an active interruption and
include ordinary simulator variation. The clear reference is one seed.
Its assignment-only arm logged one shadow confirmation, and its recheck
arm recorded none.

The feature is available as an experiment option. The three low-box
seed blocks share the same world layout, frozen checkpoints, and Gazebo
pose registration. These results support a mechanism check under that
configuration.

[Per-run results](../ros_gazebo/experiments/2026-11-23-active-depth/paired_summary.csv)
· [Matched differences](../ros_gazebo/experiments/2026-11-23-active-depth/paired_deltas.csv)
· [Interruption evidence](../ros_gazebo/experiments/2026-11-23-active-depth/interventions.csv)
· [Pilot manifest](../ros_gazebo/experiments/2026-11-23-active-depth/paired_manifest.json)
· [Comparison figure](../ros_gazebo/experiments/2026-11-23-active-depth/active_depth_pairs.png).

Regenerate the differences, interruption table and figure from the raw
study directory.

```sh
python3 scripts/summarize_active_depth_study.py "$STUDY_OUTPUT" "$SUMMARY_OUTPUT"
```
