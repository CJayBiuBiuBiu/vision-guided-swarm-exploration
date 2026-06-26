# EA-VF-MAPPO: Residual Execution-Aware Voronoi MAPPO for Multi-Robot Exploration

A Python implementation of a hierarchical multi-robot exploration framework for
unknown environments. An interpretable, multi-criteria **Voronoi-based
frontier assignment** provides an analytical baseline for "which unexplored
region should robot *i* go to next"; a **residual reinforcement-learning
policy** (vision-guided: a CNN over each robot's local occupancy-grid view)
learns a bounded correction on top of that baseline to account for execution
risk, collision risk, and communication constraints the analytical method
can't capture on its own. Trained with **MAPPO** (Multi-Agent PPO) under a
centralized-critic / decentralized-execution (CTDE) scheme.

Currently a simulation-only research codebase; real-robot deployment is
planned as a follow-up phase (see `docs/handoff_2026-06-23.md` for the most
recent engineering/debugging history).

## Architecture

```
eavf_mappo/
├── mapping/          Occupancy grid: world<->grid coordinate conversion,
│                     ray-cast map updates, local + global BEV image
│                     extraction (the "vision" input to the policy).
├── envs/
│   ├── grid_world.py             Hidden ground-truth world generator
│   │                             (open_room/corridor/maze/office/cluttered)
│   │                             and a ray-cast LiDAR-style sensor model.
│   └── multi_robot_exploration_env.py
│                                 MultiRobotExplorationEnv: dict-keyed
│                                 multi-agent reset()/step() API, reward
│                                 aggregation, and high-level decision
│                                 trigger logic (stuck/interval/arrival).
├── planning/         Grid A* search and one-shot BFS distance fields.
├── exploration/
│   ├── frontier_detector.py      Frontier detection, connected-component
│   │                             clustering, FrontierToken construction,
│   │                             top-K candidate selection.
│   ├── frontier_token.py         FrontierToken: per-candidate feature
│   │                             vector (distance, path cost, risk terms,
│   │                             information gain, comm uncertainty).
│   └── execution_aware_voronoi.py
│                                 Six-criterion normalized cost matrix +
│                                 greedy (optionally cross-robot-unique)
│                                 frontier assignment — the analytical
│                                 backbone the residual actor refines.
├── robots/           RobotState dataclass (pose, velocity, target, path,
│                     collision/stuck bookkeeping).
├── communication/    FrontierMessage + a learned-importance, bandwidth-
│                     limited top-k message scheduler.
├── control/          Differential-drive local controller: A*-path
│                     following, collision-recovery behavior.
├── marl/
│   ├── actor.py                  ResidualFrontierActor: CNN (BEV) + token
│   │                             encoder + masked attention -> a bounded
│   │                             residual on the analytical cost, plus two
│   │                             maskable "continue/release" special
│   │                             actions.
│   ├── critic.py                 CentralizedCritic (training-time only;
│   │                             sees the team's currently-explored map,
│   │                             never the hidden ground truth).
│   ├── action_adapter.py         Decodes the actor's discrete action index
│   │                             into the env's integer action contract.
│   ├── rollout.py                Per-robot decision construction and
│   │                             hierarchical rollout collection (one RL
│   │                             transition per high-level decision, reward
│   │                             accumulated over the low-level steps it
│   │                             spans — not one transition per tick).
│   ├── rollout_buffer.py         RolloutBatch/RolloutBuffer plus minibatch
│   │                             slicing.
│   └── mappo.py                  GAE (computed per agent), PPO clipped
│                                 objective, return normalization, and
│                                 value-loss clipping.
├── evaluation/
│   ├── policies.py               Heuristic baselines (nearest_frontier,
│   │                             round_robin, constant) for comparison.
│   ├── runner.py / checkpoint_runner.py
│   │                             Episode/evaluation loops for heuristic
│   │                             policies and for a trained checkpoint.
│   ├── reporting.py              CSV writers + training/evaluation curve
│   │                             plotting.
│   └── visualization.py          Renders an episode (map, robots, paths,
│                                 frontiers) to an animated GIF.
└── utils/            Config merging, seeding, the RetentionRatio metric.

scripts/        CLI entry points: train.py, evaluate.py, evaluate_checkpoint.py,
                visualize_episode.py, diagnose_training.py, plot_*.py
tests/          Unit tests (unittest) covering every module above.
configs/        experiment_config.yaml — canonical experiment configuration.
docs/           Design/debugging history (handoff notes, plans).
```

## Quickstart

Run the test suite:

```bash
python -m unittest discover -v
```

Evaluate heuristic baselines and generate CSV/PNG reports:

```bash
python scripts/evaluate.py --seeds 0 1 --policies nearest_frontier round_robin \
    --max-steps 30 --map-size 8 --sensor-range 1.5 --output-dir results/demo
```

Train the residual actor/critic with MAPPO:

```bash
python scripts/train.py --iterations 200 --rollout-length 32 --n-robots 2 \
    --map-size 8 --map-type office --output-dir results/train_run
```

Evaluate a trained checkpoint against the baselines, or render an episode:

```bash
python scripts/evaluate_checkpoint.py --checkpoint results/train_run/checkpoint.pt ...
python scripts/visualize_episode.py --checkpoint results/train_run/checkpoint.pt \
    --episode-steps 200 --output results/viz/episode.gif
```

## Status

The training pipeline runs end-to-end (env <-> actor/critic <-> MAPPO update
<-> checkpoint). See `docs/handoff_2026-06-23.md` for the detailed history of
a training-stability investigation (policy collapse into a degenerate "do
nothing" exploration strategy, root-caused and fixed) and
`docs/visualization_plan_2026-06-23.md` for the graphical-feedback tooling
design. Real-robot deployment and a full ablation/baseline evaluation pass
are the next phases of work.
