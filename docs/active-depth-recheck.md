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
