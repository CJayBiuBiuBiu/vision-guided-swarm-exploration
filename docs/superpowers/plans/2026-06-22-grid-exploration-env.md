# Grid Exploration Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the lightweight evaluation environment so coverage changes with robot motion and sensor observations, making behavior metrics meaningful before PyBullet integration.

**Architecture:** Add a hidden true 2D world map and ray-cast sensor helpers under `eavf_mappo/envs/grid_world.py`. Keep `OccupancyGrid` as the robot's observed map. Modify `MultiRobotExplorationEnv` so reset generates a true map, each low-level step raycasts from true map into observations, and collision/motion uses the true map.

**Tech Stack:** Python 3.10, NumPy, stdlib unittest, existing occupancy grid and frontier modules.

---

### Task 1: Failing Tests

**Files:**
- Create: `tests/test_grid_world.py`
- Modify: `tests/test_env_step.py`
- Modify: `tests/test_evaluation.py`

- [x] **Step 1: Add tests for generated worlds, raycast ranges, and env coverage growth**

Run: `python -m unittest tests.test_grid_world tests.test_env_step tests.test_evaluation -v`
Expected: ERROR/FAIL because `eavf_mappo.envs.grid_world` and true-map coverage behavior are missing.

### Task 2: True Grid World Helpers

**Files:**
- Create: `eavf_mappo/envs/grid_world.py`

- [ ] **Step 1: Implement deterministic true-world generation with occupied borders and map-type obstacles**
- [ ] **Step 2: Implement world-cell clearing for spawn areas**
- [ ] **Step 3: Implement ray casting from true map to `(angle, distance, has_hit)` observations**

Run: `python -m unittest tests.test_grid_world -v`
Expected: PASS.

### Task 3: Environment Integration

**Files:**
- Modify: `eavf_mappo/envs/multi_robot_exploration_env.py`

- [ ] **Step 1: Store `self.true_cells` during reset**
- [ ] **Step 2: Replace direct observed free-disk updates with sensor observations from `raycast_ranges`**
- [ ] **Step 3: Use true map to guard motion and collision**
- [ ] **Step 4: Replan when target frontier changes or path is exhausted**

Run: `python -m unittest tests.test_env_step tests.test_evaluation -v`
Expected: PASS and coverage increases in rollout tests.

### Task 4: Verification and Example Results

**Files:**
- Existing scripts only.

- [ ] **Step 1: Run full tests**

Run: `python -m unittest discover -v`
Expected: PASS.

- [ ] **Step 2: Run smoke evaluation**

Run: `python scripts/evaluate.py --seeds 0 1 --policies nearest_frontier round_robin --max-steps 30 --output-dir results/grid_smoke`
Expected: CSV/PNG files exist and `final_coverage` is greater than initial coverage.
