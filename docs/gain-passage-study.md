# Information-aware visual passage regions

The [active-passage study](active-passage-study.md) obtained a usable
paired RGB score in four of twenty inspections. Twelve inspections
lost a candidate, and two successful redirects did not establish
that the observer helped a robot cross the partition. The present
strategy changes both the intervention criterion and the lifetime
of an inspected region.

## Information estimate

For a robot at position \(p\) and candidate frontier \(q\), resolve each
scan-grid cell centre into coordinates longitudinal and lateral to
the bearing from \(p\) to \(q\). Restrict the count to a 0.50 m wide
strip, with the longitudinal interval beginning at the greater of
the first camera-supported distance and \(\|q-p\|-0.25\) m. Its far
end is 0.25 m before the minimum of the paired RGB clear distance
and the first observed occupied cell in that strip. Occupied and
observed-free cells contribute zero. The estimate is the number
of currently unknown cell centres in the interval. At 0.20 m
grid resolution, each cell corresponds to 0.04 square metres.

If no camera score exists for a direction, the same calculation
with the nominal 1.0–5.75 m sampling interval is an optimistic
inspection potential. It cannot justify redirecting the robot.
Only two fresh, agreeing RGB frames can support a changed goal.
The estimate describes scan-unknown space in a visually supported
direction. It neither changes the occupancy grid nor establishes
that an unseen cell is safe or physically reachable.

With a valid original route and camera score, a direct alternative
needs at least six unknown cells (0.24 square metres) and at least
four more than the original preference. The route may be at most
0.60 m longer than the original path. In order of priority, rank
eligible alternatives by descending estimated gain, shorter route
and longer visual clear distance. If no direct alternative
qualifies, the same gain thresholds on optimistic potential can
justify inspecting an unscored direction. Keep the existing
25–100 degree turn and 1.20 m additional-route limits, four-second
observation deadline, and twelve-second per-robot cooldown.
Prioritise potential gain before turn angle.

## Fixed world-coordinate regions

During inspection keep the original approach coordinate and
bearing fixed. The vision score is collected for that bearing even
when the associated frontier cluster temporarily disappears.
Stop when pose drift exceeds 0.25 m or the current map indicates
that the potential gain has been depleted. After observing the
region, find current owned frontier candidates within 1.20 m of
the original coordinate and 15 degrees of its original bearing.
Re-evaluate their camera scores using the same fresh frame pair.
Recalculate the original gain from the latest scan map and its
initial RGB evidence. Apply the gain thresholds and the original
route budget to select among the available candidates. If no
alternative is valid, recover a currently owned and reachable
frontier in the original preferred region when possible.

This association gives the observation a stable spatial identity
without retaining obsolete frontier assignments or adding
unmapped movement targets. It preserves the existing navigation,
clearance shield and recovery logic. For a visual redirect, the
bounded continuation procedure from the preceding study also
requires at least six unknown cells in the proposed direction,
in addition to its original camera and known-free corridor gates.

The method logs a compressed snapshot of the observed occupancy
grid and per-candidate gain estimates at every evaluated choice.
The audit restores those grids and recomputes the selection
arithmetic. Geometry describing the layout itself appears only
in the separate evaluation mask and crossing analysis. The
controller still receives exact Gazebo pose and a configured
outer map extent. The RGB score derives from calibrated floor
appearance and projection, with no new encoder or policy training.

## Experimental protocol

Compare three arms with the same seed-62600 MAPPO actor, calibrated
RGB encoder, DDPG weights, LiDAR actor and assignment cues,
clearance shield and weak-WAIT rechecking:

- Passive projected-width reranking.
- Active projected-width inspection and continuation.
- Information-aware inspection and spatial association.

The four new layouts vary the partition location, the opening
centres and widths, blocked exits, downstream obstacles and spawn
orientation. The same scene and seed are used across each group
of three runs. Method order rotates between layouts. Each formal
trial runs for 120 simulation seconds. Seeds 71321–71324 specify
the formal layouts, and seed 71310 specifies the separate 45 s
three-arm smoke comparison. The criteria are fixed before the
formal rollouts. Reference coverage is excluded from the actor
and controller progress input. The trial begins with an unknown
interior occupancy grid, filled by scan mapping.

Prepare the ROS workspace with patch 19. Source that workspace and
use the frozen actor and encoder linked by the [dynamic passage
study](dynamic-passage-study.md). For a headless Gazebo host, use
a virtual display such as `xvfb-run -a`.

```sh
python3 scripts/run_gain_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --plan-only
python3 scripts/run_gain_passage_study.py smoke "$ROS_PROJECT" "$ACTOR" "$RGB" "$SMOKE_OUT"
python3 scripts/run_gain_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT"
python3 scripts/run_gain_passage_study.py formal "$ROS_PROJECT" "$ACTOR" "$RGB" "$FORMAL_OUT" --audit-only
python3 scripts/report_gain_passage_study.py "$FORMAL_OUT" "$ANALYSIS_OUT"
```

Use separate output paths. Each rollout retains its launch inputs,
model fingerprints, metrics, actor actions and decision trace.
The verifier checks model and shield consistency, horizon, map
denominator, camera timestamps, selected goals, outcomes and
the arithmetic of the information-aware choices. Inspection
results also record temporary candidate absence and total control
ticks. The strategy is opt-in through
`rgb_passage_strategy:=gain_region`.

## Smoke validation

All 145 ROS tests passed, including the replayable occupancy
estimate and a coordinator scenario in which an inspected
frontier temporarily disappeared and returned with a different
ID. The three smoke runs finished the 45 s horizon and passed
the decision audits.

| Strategy | Coverage | Robots crossing | Paired inspection scores |
| --- | ---: | ---: | ---: |
| Passive width | 42.96% | 0 | 0/0 |
| Active width | 49.92% | 0 | 0/3 |
| Gain-region | 62.16% | 1 | 4/4 |

The gain-region arm redirected twice, once after five control
ticks without a corresponding current frontier. One redirected
goal was reached. The sum of estimated gain advantages at the
two decisions was 44 cells. The first crossing occurred at
31.752 s, before the crossing robot's own redirect. The smoke
results check execution and the candidate lifetime mechanism;
the locked twelve-run comparison determines the extent to
which the behaviour generalises.

## Formal results

All twelve trials reached 120 s and passed the initial and
repeat audits. Four distinct coverage-mask hashes identify the
four layouts; the mask and policy fingerprints match across
the three conditions within each layout.

| Layout seed | Passive coverage | Active coverage | Gain-region coverage | Robots crossing (passive / active / gain) |
| --- | ---: | ---: | ---: | ---: |
| 71321 | 66.67% | 51.65% | 54.80% | 1 / 0 / 0 |
| 71322 | 99.92% | 84.51% | 95.49% | 2 / 1 / 1 |
| 71323 | 47.88% | 38.73% | 88.07% | 0 / 0 / 2 |
| 71324 | 69.02% | 68.75% | 94.91% | 0 / 0 / 2 |
| Mean | 70.87% | 60.91% | 83.32% | 0.75 / 0.25 / 1.25 |

The gain-region mean exceeded passive width by 12.45 percentage
points and active width by 22.41 points. It improved coverage in
two of four passive comparisons and all four active comparisons.
Three gain-region runs crossed the partition, compared with two
passive and one active run. Five robots crossed across the four
gain-region runs, compared with three and one, respectively.
Two gain-region trials reached 90% at 92.251 and 99.503 s.
The passive method reached 90% once and 97% once (109.264 and
116.253 s in seed 71322). The gain-region and active conditions
did not reach 97%.

| Metric (mean over four layouts) | Passive | Active | Gain-region |
| --- | ---: | ---: | ---: |
| Team path (m) | 40.55 | 31.51 | 47.92 |
| Stuck events | 4.50 | 5.00 | 5.25 |
| Near-collision events | 1.50 | 1.25 | 0.75 |
| Reached goals | 16.25 | 12.00 | 16.75 |

The gain-region strategy explored more area with longer paths
and slightly more stuck events in this sample. Near-collision
events count threshold crossings in the clearance telemetry;
they are not physical contact counts.

### Intervention evidence

The active-width condition initiated 16 observations and obtained
one usable current frontier score. Nine inspections lost a
candidate, three timed out, three failed pose or target stability,
and one redirected. The gain-region condition initiated 15
observations. Twelve obtained an anchor score and eight obtained
a usable score for at least one current owned candidate; the
latter is comparable with the active-width score count. Four
observations redirected, four had insufficient gain, four had
no corresponding region candidate at the end, and three stopped
when map updates depleted the potential. Six encountered a
temporary lack of associated candidates (24 control ticks in
total), and none of those six redirected in the formal runs.
The separate smoke example exercised a successful redirect
after temporary disappearance.

Four inspected and one direct gain-region decisions actually
changed goals. Four of the five redirected attempts reached
their goals. The estimated gains at the five interventions
exceeded the corresponding preferences by 72 cells in aggregate;
this is a sum of local estimates, not a unique-area measurement.

Two same-robot traces show a crossing during the new assignment:

- Seed 71323, robot 3: RGB-based redirect at 97.256 s, first
  crossing at 101.252 s, and goal arrival at 105.504 s. The
  selected goal was on the opposite side of the partition.
- Seed 71324, robot 3: RGB-based redirect at 67.505 s, first
  crossing at 69.252 s, and goal arrival at 73.006 s. The
  selected goal was again beyond the partition.

These traces establish execution of the changed goals through
an opening. The matched aggregate comparisons remain subject
to Gazebo rollout variation and do not identify a counterfactual
effect of each individual decision.

Three bounded continuation segments were issued and reached
within the controller's 0.50 m arrival tolerance, in seeds
71323 and 71324. The segments were 0.80, 1.00 and 1.00 m,
respectively, all with fresh camera and known-free grid evidence.
Neither older arm executed a continuation segment. All 145 ROS
tests, three existing coordinator integration checks and 55
main repository tests passed.

### Interpretation and scope

This is the first matched study in this project in which
information-aware visual selection improved final coverage over
the earlier active-width version in all four held-out scenarios,
and produced traced crossings during applied visual redirects.
The strategy remains an opt-in experimental mode. The four
layouts share a partition-and-opening family, the observations
use exact simulated pose, the comparison freezes one MAPPO
lineage, and the gain and spatial association changes are
combined. Broader layout families, repeated seeds, independent
training lineages and separated component ablations are needed
to establish generalisation and attribute the improvement.

[Matched results, plots, world files, manifests, intervention
traces and audit logs](../ros_gazebo/experiments/2026-11-25-gain-passage/)
