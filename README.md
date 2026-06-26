# Vision-Guided Swarm Exploration

Residual Execution-Aware Voronoi MAPPO for multi-robot exploration in unknown
environments.

This repository implements a simulation-based research prototype for swarm
robotic exploration. The system combines classical frontier exploration,
execution-aware Voronoi assignment, and multi-agent reinforcement learning. A
residual actor learns to adjust an analytical frontier-assignment baseline,
while a centralized critic is used during MAPPO training.

## Core Idea

The project is built around a hierarchical exploration loop:

1. Robots build local occupancy-grid observations from simulated range sensing.
2. Frontier candidates are detected from the explored/unknown boundary.
3. An execution-aware Voronoi module estimates assignment costs for each robot
   and frontier.
4. A residual policy refines that analytical assignment using local BEV
   observations and frontier tokens.
5. A low-level controller plans and follows grid paths toward selected targets.
6. Evaluation scripts report coverage, reward, collision, and trajectory
   behavior across multiple maps and random seeds.

## Project Structure

```text
eavf_mappo/
  envs/           Grid-world map generation and multi-robot exploration env
  mapping/        Occupancy grid updates and local/global BEV extraction
  exploration/    Frontier detection, frontier tokens, Voronoi assignment
  planning/       A* path planning
  control/        Differential-drive local controller
  communication/  Lightweight frontier message scheduling
  robots/         Robot state definitions
  marl/           Residual actor, centralized critic, rollout buffer, MAPPO
  evaluation/     Baselines, checkpoint evaluation, reporting, visualization

scripts/          CLI entry points for training, evaluation, plotting, GIFs
tests/            Unit tests for environment, planning, MARL, and evaluation
configs/          Example experiment configuration
docs/             Development notes, plans, and handoff records
```

## Installation

```bash
pip install -e .
```

Main dependencies are NumPy, PyTorch, and Matplotlib.

## Quick Commands

Run tests:

```bash
python -m unittest discover
```

Visualize one exploration episode:

```bash
python scripts/visualize_episode.py --policy nearest_frontier --map-type office \
    --seed 0 --episode-steps 400 --render-steps 400 --frame-stride 4 \
    --output results/viz/demo.gif
```

Evaluate heuristic baselines:

```bash
python scripts/evaluate.py --policies nearest_frontier round_robin constant \
    --map-type office --seeds 0 1 2 3 4 --max-steps 400 \
    --output-dir results/eval_office
```

Train the residual MAPPO policy:

```bash
python scripts/train.py --iterations 200 --rollout-length 32 --n-robots 2 \
    --map-size 8 --map-type office --output-dir results/train_run
```

Evaluate a trained checkpoint:

```bash
python scripts/evaluate_checkpoint.py --checkpoint results/train_run/checkpoint.pt \
    --map-types office corridor maze cluttered --seeds 0 1 2 \
    --output-dir results/checkpoint_eval
```

## Maps and Reproducibility

`open_room` is a fixed baseline map. `office`, `corridor`, `maze`, and
`cluttered` support seed-controlled variation: the same seed reproduces the
same environment, while different seeds create different layouts. This makes it
possible to use a fixed seed for demos and multiple seeds for robustness
evaluation in unfamiliar environments.

## Outputs

The scripts write CSV metrics, training curves, evaluation plots, checkpoints,
and animated GIFs under `results/`. Typical metrics include final coverage,
time to coverage threshold, episode return, path length, and collision count.

## Status

This is currently a simulation-focused research codebase. The main pipeline
from environment rollout to MAPPO update, checkpointing, evaluation, and GIF
visualization is implemented and covered by unit tests. Real-robot deployment
and larger-scale benchmark studies are future work.
