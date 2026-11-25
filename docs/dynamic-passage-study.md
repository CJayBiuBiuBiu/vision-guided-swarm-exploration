# Live RGB passage-width guidance in unknown layouts

The [stationary probe](doorway-traversability.md) projected the
floor just beyond a surveyed doorway plane. The dynamic controller
projects a series of cross-sections in the current camera frame.
It searches from 1.0 to 5.75 m along sampled bearings, in 0.25 m
steps. The robot is aligned locally using the camera calibration
and its simulated pose. The width in each section follows the
contiguous floor-colour support around the sample ray.
The body-width samples must be visible. The first valid section
must be within 1.75 m, and missing later sections end the
measurement.

Two adjacent sections below 0.366 m indicate a candidate
bottleneck. The evidence must agree across two recent RGB frames
within 0.40 s, with at most 0.25 m of translational drift. A wider
direction must extend unobstructed at least 0.75 m farther.
The coordinator can choose it only when it already owns that
frontier and has a scan-map path. The route may add at most
0.60 m over the original preference. RGB changes the selection
among available frontiers. All commanded paths and DDPG settings
use the same navigation stack in the two arms.

## What the robots know

The four formal layouts are absent from the archived MAPPO
training. Partition positions, door centres, widths, spawn poses and
downstream obstacles differ. Two layouts offer a narrow alternative
beside a wider opening. Two provide a wider alternative with an
obstructed exit. The smoke layout uses a separate geometry and seed.

The finite map bounds are configured. The mapping grid starts with
unknown interior cells, then fills through LiDAR scans. None of
the doorway coordinates or obstacle boxes enter the path or the
vision score. A separate geometry file supplies the reference
coverage mask for scoring. Loading it requires fixed-horizon,
observed-area control and scan mapping. The actor omits the
reference-coverage feature. During offline scoring, the geometry
also identifies gate crossings in recorded trajectories. Pose
still comes from Gazebo. This study isolates map uncertainty while
holding localization and communication fixed.

## Matched evaluation

The observer computes the same passage score as the reranker and
logs alternative choices. The reranker applies those choices when
the conditions hold. Both use the same frozen seed-62600 MAPPO
actor and DDPG weights, LiDAR feature and assignment cues, and
12 s route rechecking for weak WAIT decisions. Each formal
layout and seed runs once per arm for 90 s. The order alternates.
The smoke pair uses 40 s.

The audit verifies the applied frontier and its trace outcome.
It reports camera availability, eligible replacements and their
arrival outcomes, first crossing time and final coverage.
A crossing requires a recorded passage through the wall at an
opening and subsequent motion at least 0.55 m past the wall.

Prepare ROS/Gazebo with patch 17 and source the workspace.
Use the [seed-62600 actor](../ros_gazebo/experiments/2026-11-24-rgb-passage/lidar_mappo_62600_update32.pt)
and the [calibrated encoder](../ros_gazebo/experiments/2026-11-24-rgb-low-clearance/calibrated/encoder.pt).
Use distinct output directories. On a headless host, prefix each
command with `xvfb-run -a`.

```sh
python3 scripts/run_dynamic_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_dynamic_passage_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_dynamic_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_dynamic_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
```

## Field-of-view correction and smoke results

The initial diagnostic used visibility across the full sample strip.
Its first formal trajectory finished, and the verifier rejected it
because no preferred frontier had a usable camera score. A replay
of the ten archived doorway images exposed the restriction. Only
3 of 29 sampled bearings could produce a score. Requiring visibility
of the robot-width envelope increased this to 23 of 29 on each
image. Extending the sampled distance to 5.75 m also exposed the
two obstructed exits in those archived images. The raw initial
attempt remains in the diagnostic archive. A different first layout
and new seeds define the revised formal schedule.

The revised workspace passed 130 ROS-side tests. The two 40 s
smoke runs recorded 16 preferred-route bottleneck decisions
combined. Each such decision had the original grid path. None
had a simultaneously scored alternative frontier for that robot.
The smoke runs completed and verified the moving-camera and
selection traces. The observation arm crossed once, and the
reranking arm had zero crossings.

[Smoke summaries and revised schedule](../ros_gazebo/experiments/2026-11-25-dynamic-passage/)
