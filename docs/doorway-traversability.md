# From visible opening to usable passage

The [RGB passage pilot](rgb-passage-study.md) found that the camera
could distinguish a wide opening from a wall beyond the LiDAR
horizon. That pilot measured floor continuation along a direction.
This audit measures the width of the contiguous visible floor and
checks for free space immediately beyond the opening.

The controlled probe uses the surveyed partition plane, the
candidate opening centre and synchronized camera pose and
intrinsics. It projects two cross-sections onto the floor.
The nearer section estimates the apparent entry width. The
farther section checks clearance behind the opening. A floor-colour
reference comes from the same RGB frame. An aligned Waffle Pi
requires 0.366 m for this test, including a 5 cm margin on each
side of its 0.266 m nominal base width. The decision uses the
smaller of the two estimates.

The ten stationary cases mirror five layouts. Four have opening
widths of 0.22, 0.46, 0.90 and 1.50 m. The fifth has a 0.90 m
opening with an obstruction immediately behind it. This label
describes an aligned crossing through the designated opening.
The surveyed wall plane is an input to the probe. The true width
and obstruction label enter only the audit. The evaluation uses
stationary acquisitions on a level floor under fixed lighting.

## Archived replay

Both archived 1.50 m openings produced an estimated entry width
of 1.425 m and a rear width of 1.400 m. The two bracketing LiDAR
beams were at the 3.5 m horizon in each direction. This replay
checks the projection pipeline using existing synchronized
camera images.

Build patch 16 and source the ROS workspace. The offline replay
uses the archived camera images and does not start Gazebo.
Use a separate directory for the ten-scene capture.

```sh
python3 scripts/probe_doorway_traversability.py "$ROS_PROJECT" "$REPLAY_OUT" \
  --archived-probes ros_gazebo/experiments/2026-11-24-rgb-passage/probe
xvfb-run -a python3 scripts/probe_doorway_traversability.py "$ROS_PROJECT" "$PROBE_OUT"
```

## Stationary results

All ten cases were captured and reconstructed from synchronized
frames. In each case the two LiDAR beams bracketing the candidate
direction reported no return within the configured range.

| Case | Nominal opening | Projected entry width | Projected width behind opening | Width-rule decision, mirrors |
| --- | ---: | ---: | ---: | --- |
| Narrow | 0.22 m | 0.175 m | 0.125 m | Withhold, 2/2 |
| Tight | 0.46 m | 0.425 m | 0.375 m | Passable, 2/2 |
| Wide | 0.90 m | 0.850–0.875 m | 0.800 m | Passable, 2/2 |
| Reference | 1.50 m | 1.450 m | 1.400 m | Passable, 2/2 |
| Obstructed exit | 0.90 m | 0.850–0.875 m | 0.000 m | Withhold, 2/2 |

Both the width rule and the previous floor-continuation rule agreed
with the designed labels in all ten cases. The width rule also
reports where visible clearance is lost. The obstructed-exit
scenes have a visible entrance and no supported floor in the
rear cross-section. The tight scene is near the acceptance
boundary. Its estimated rear width is 9 mm above the requirement
and the lateral sampling interval is 25 mm.

These captures establish the stationary measurement in a known
partition geometry. They give a reference for training on actual
entry and crossing outcomes. The current audit uses the surveyed
plane and known candidate placement. It has no planner
intervention.

[Scene definitions, synchronized frames and results](../ros_gazebo/experiments/2026-11-24-doorway-traversability/)
