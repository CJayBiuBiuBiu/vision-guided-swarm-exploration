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
