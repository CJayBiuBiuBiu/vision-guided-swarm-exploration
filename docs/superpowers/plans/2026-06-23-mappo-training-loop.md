# MAPPO Training Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the existing vision-guided actor/critic, execution-aware Voronoi prior, environment, and MAPPO update into a minimal trainable loop.

**Architecture:** Add a small action adapter for `K+2` actor actions, a global BEV view for the centralized critic, a rollout collector that keeps one time-ordered buffer per agent, and a training CLI that runs short MAPPO iterations and checkpoints. Keep the implementation lightweight and smoke-testable.

**Tech Stack:** Python 3.10, NumPy, PyTorch, stdlib unittest, existing 2D exploration environment.

---

### Task 1: Failing Tests

**Files:**
- Create: `tests/test_action_adapter.py`
- Create: `tests/test_global_bev.py`
- Create: `tests/test_rollout_training.py`

- [x] **Step 1: Test action decoding semantics**
- [x] **Step 2: Test global BEV shape and robot/target channels**
- [x] **Step 3: Test rollout collection, real Voronoi `c_analytic`, per-agent GAE update, and CLI smoke**

### Task 2: Action Adapter and Global BEV

**Files:**
- Create: `eavf_mappo/marl/action_adapter.py`
- Modify: `eavf_mappo/mapping/occupancy_grid.py`
- Modify: `eavf_mappo/envs/multi_robot_exploration_env.py`

- [ ] **Step 1: Implement `decode_action(action_idx, topk_tokens)`**
- [ ] **Step 2: Add `OccupancyGrid.get_global_bev()`**
- [ ] **Step 3: Teach env special actions `-1` continue and `-2` release**

### Task 3: Rollout and MAPPO Update

**Files:**
- Create: `eavf_mappo/marl/rollout.py`
- Modify: `eavf_mappo/marl/mappo.py`

- [ ] **Step 1: Implement `build_decision_inputs()` using true execution-aware Voronoi cost**
- [ ] **Step 2: Implement `collect_rollout()` returning one `RolloutBuffer` per agent**
- [ ] **Step 3: Modify `mappo.update()` to accept a list of per-agent buffers and compute GAE per agent before merging**

### Task 4: Train CLI

**Files:**
- Create: `scripts/train.py`
- Modify: `README.md`

- [ ] **Step 1: Add CLI args for rollout length, iterations, robots, map, seed, and checkpoint path**
- [ ] **Step 2: Run a small train smoke and save checkpoint/log CSV**
- [ ] **Step 3: Document the command**

### Task 5: Verification

Run:
- `python -m unittest discover -v`
- `python scripts/train.py --iterations 1 --rollout-length 2 --n-robots 2 --map-size 6 --sensor-range 1.2 --output-dir results/train_smoke`

Expected:
- Tests pass.
- Train smoke writes `checkpoint.pt` and `train_log.csv`.
