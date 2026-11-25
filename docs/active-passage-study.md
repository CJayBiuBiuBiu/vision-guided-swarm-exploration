# Active camera inspection and bounded passage continuation

The [previous study](dynamic-passage-study.md) found a recurring
visibility gap. The camera often scored the current goal as a
bottleneck but could not simultaneously score another owned frontier.
The one applied redirection reached a nearby frontier on the approach
side of the partition.

The new opt-in strategy inspects one additional candidate when a
pair of fresh frames confirms a bottleneck and passive reranking
has no eligible replacement. It chooses a currently unscored,
owned frontier with a mapped route at most 1.20 m longer than the
preferred route. The turn from the current heading must lie between
25 and 100 degrees. The robot rotates in place for at most 4 s,
with a minimum 12 s interval between inspections for each robot.
Translation is zero during this operation. The turning controller
stops when the command or scan is stale or the all-around clearance
fails the configured threshold. The original exploration stack and
DDPG speed shield resume afterward.

Two distinct frames captured after the inspection started must
agree, with camera yaw within 8 degrees of the candidate direction.
The robot must remain within 0.25 m of its original position. The
two candidate clusters may be rebound to currently owned
frontiers within 0.60 m of their original anchors. The preferred
bearing must remain within 15 degrees of its initial bearing. The route is recalculated against the updated scan
map before a redirect. The required gain in visible clear distance
remains 0.75 m. An unavailable or unconvincing alternative falls
back to the original goal if it is still valid.

For a successful visual redirect, the controller stores the
selected direction. On approach-goal arrival, it can extend that
direction by up to 1.2 m at a time, for at most two segments within
45 s of selection. Each extension needs recent paired RGB evidence
and a body-width envelope consisting entirely of cells that the
laser mapping has marked free. No extension enters unknown or
occupied map cells. During execution, the controller rechecks the
corridor and its known-free grid path. A WAIT may retain an
extension for the first 4.5 s while independent laser clearance
and the corridor remain valid. This bounded hold does not apply
to ordinary frontiers. Stuck recovery, invalid ownership and stall
fallback can terminate the continuation.

These rules depend on specialised simulated floor appearance,
camera calibration, exact robot pose and heuristic thresholds.
They add active perception and short-horizon execution to the
previous RGB floor-projection rule. They do not train a new visual
encoder or policy. A paired passive-versus-active comparison
measures their combined effect; an additional scan-only ablation
would be needed to isolate the cost of turning from the effect of
goal continuation.

## Locked evaluation protocol

Both arms use projected-width reranking, the same seed-62600
MAPPO actor, DDPG weights, calibrated encoder, LiDAR policy and
assignment cues, and weak-WAIT rechecking. The active arm adds
the strategy above. Four new layouts vary the partition location,
opening centres and widths, obstructed exits, spawn orientation
and downstream fixtures. The four seeds are 71221–71224. Each
layout runs once per arm for 120 simulation seconds, alternating
the order. A separate layout and seed 71211 provide two 45 s smoke
runs.

As before, the controller knows the outer map extent and receives
Gazebo pose. The interior map starts unknown and is populated from
scans. Door and obstacle geometry enter the separate evaluation
mask and offline crossing audit only. No reference coverage is
available to the actor.

Prepare the patched ROS workspace, source its environment and
use the frozen actor and calibrated encoder linked in the
[previous study](dynamic-passage-study.md).

```sh
python3 scripts/run_active_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_active_passage_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_active_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_active_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_active_passage_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

Run with a virtual display (for example, `xvfb-run -a`) on a
headless Gazebo host. Each output directory preserves an exact
configuration manifest, per-trial launch arguments, raw metrics
and actions, and ROS logs. The independent decision audit checks
freshness, inspection duration, visual gain, route tolerance,
assignment IDs, continuation distances and bounded holds. The
verifier also checks the shared policy, shield, reference mask
and full simulation horizon.

## Initial diagnostic and spatial rebinding

The first 45 s smoke pair (seed 71210) ran before spatial
rebinding. Both runs completed. The active run began four
inspections, of which one acquired a paired score. Two other
inspections stopped when a tracked frontier disappeared and
one stopped on pose or candidate drift. No redirect resulted.
The passive and active coverages were 58.53% and 41.91%,
respectively, with one and zero crossing robots. These pilot
rollouts exposed cluster instability as an additional execution
issue. The revised implementation retains a bounded spatial
association after clustering changes. The initial raw runs and
source tree remain archived; the formal protocol runs separately
from the revised source tree.

The revised workspace passed 138 ROS tests. The second smoke
pair used seed 71211. Both trials reached the full 45 s horizon
and passed the trace audit. Spatial rebinding was available but
the two attempted inspections still lost a candidate; neither
returned a usable score. The passive run covered 59.71% and
crossed the partition with one robot. The active run covered
41.10% and had no crossing. This remains an experimental opt-in
strategy. The eight-run held-out comparison was then started
using the locked revised protocol.
