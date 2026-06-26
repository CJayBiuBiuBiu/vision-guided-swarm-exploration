# EA-VF-MAPPO Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a concrete, testable Python implementation of the EA-VF-MAPPO specification in this empty workspace.

**Architecture:** Create a small Python package organized around the specification boundaries: mapping, planning, exploration, communication, MARL, control, envs, robots, and utils. Keep PyBullet as an optional integration layer while making the algorithmic core runnable with only NumPy and PyTorch.

**Tech Stack:** Python 3.10, NumPy, PyTorch, stdlib unittest, YAML configuration.

---

### Task 1: Specification-Facing Tests

**Files:**
- Create: `tests/test_occupancy_grid.py`
- Create: `tests/test_astar.py`
- Create: `tests/test_frontier_detector.py`
- Create: `tests/test_voronoi.py`
- Create: `tests/test_residual_actor.py`
- Create: `tests/test_comm_scheduler.py`
- Create: `tests/test_env_step.py`
- Create: `tests/test_reward.py`

- [x] **Step 1: Write failing tests**

The tests import the package modules before the package exists, so the first run must fail with import errors.

- [ ] **Step 2: Run test discovery to verify RED**

Run: `python -m unittest discover -v`
Expected: FAIL/ERROR because `eavf_mappo` modules are missing.

### Task 2: Core Data Structures, Mapping, and Planning

**Files:**
- Create: `eavf_mappo/robots/robot_state.py`
- Create: `eavf_mappo/exploration/frontier_token.py`
- Create: `eavf_mappo/communication/message.py`
- Create: `eavf_mappo/mapping/occupancy_grid.py`
- Create: `eavf_mappo/planning/astar.py`

- [ ] **Step 1: Implement dataclasses with exact field names**
- [ ] **Step 2: Implement occupancy grid coordinate conversion, ray updates, BEV, and coverage**
- [ ] **Step 3: Implement A* and one-shot BFS distance field**
- [ ] **Step 4: Run targeted tests**

Run: `python -m unittest tests.test_occupancy_grid tests.test_astar -v`
Expected: PASS.

### Task 3: Frontier Tokens and Execution-Aware Voronoi

**Files:**
- Create: `eavf_mappo/exploration/frontier_detector.py`
- Create: `eavf_mappo/exploration/execution_aware_voronoi.py`

- [ ] **Step 1: Implement frontier detection and connected-component clustering**
- [ ] **Step 2: Implement token risk fields with the `comm_aware` information split**
- [ ] **Step 3: Implement normalized execution-aware cost matrix and greedy assignment**
- [ ] **Step 4: Run targeted tests**

Run: `python -m unittest tests.test_frontier_detector tests.test_voronoi -v`
Expected: PASS.

### Task 4: Residual Actor, Communication, Metrics, and Env Skeleton

**Files:**
- Create: `eavf_mappo/marl/actor.py`
- Create: `eavf_mappo/marl/critic.py`
- Create: `eavf_mappo/marl/mappo.py`
- Create: `eavf_mappo/marl/rollout_buffer.py`
- Create: `eavf_mappo/communication/communication_scheduler.py`
- Create: `eavf_mappo/control/local_controller.py`
- Create: `eavf_mappo/envs/multi_robot_exploration_env.py`
- Create: `eavf_mappo/utils/metrics.py`
- Create: `eavf_mappo/utils/seed.py`

- [ ] **Step 1: Implement pointer-style residual actor with action masking and `tau * tanh`**
- [ ] **Step 2: Implement bandwidth-limited top-k scheduler**
- [ ] **Step 3: Implement retention ratio and seed utilities**
- [ ] **Step 4: Implement a lightweight high-level env skeleton with dict API and reward aggregation**
- [ ] **Step 5: Run targeted tests**

Run: `python -m unittest tests.test_residual_actor tests.test_comm_scheduler tests.test_env_step tests.test_reward -v`
Expected: PASS.

### Task 5: Package Scaffolding and Full Verification

**Files:**
- Create: `pyproject.toml`
- Create: `README.md`
- Create: `configs/experiment_config.yaml`
- Create: package `__init__.py` files

- [ ] **Step 1: Add project metadata and configuration schema**
- [ ] **Step 2: Run full suite**

Run: `python -m unittest discover -v`
Expected: PASS with all tests green.
