# Evaluation Reporting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reproducible way to observe EA-VF-MAPPO behavior beyond unit-test pass/fail by running rollouts, writing CSV metrics, and rendering plots.

**Architecture:** Put reusable evaluation code under `eavf_mappo/evaluation`, with pure functions for policies, rollout execution, CSV writing, summary aggregation, and plotting. Keep `scripts/evaluate.py` and `scripts/plot_results.py` as thin command-line wrappers so users can run experiments without touching internals.

**Tech Stack:** Python 3.10, stdlib `csv`/`argparse`, NumPy, Matplotlib, existing lightweight `MultiRobotExplorationEnv`.

---

### Task 1: Failing Evaluation Tests

**Files:**
- Create: `tests/test_evaluation.py`

- [x] **Step 1: Write tests for rollout rows, summary rows, CSV export, and plots**

Run: `python -m unittest tests.test_evaluation -v`
Expected: ERROR because `eavf_mappo.evaluation` does not exist yet.

### Task 2: Evaluation Library

**Files:**
- Create: `eavf_mappo/evaluation/__init__.py`
- Create: `eavf_mappo/evaluation/policies.py`
- Create: `eavf_mappo/evaluation/runner.py`
- Create: `eavf_mappo/evaluation/reporting.py`

- [ ] **Step 1: Implement `choose_actions` for `constant`, `round_robin`, and `nearest_frontier`**
- [ ] **Step 2: Implement `run_episode` and `run_evaluation` returning structured row dictionaries**
- [ ] **Step 3: Implement CSV writers and summary aggregation**
- [ ] **Step 4: Implement plot generation for coverage and reward curves**

Run: `python -m unittest tests.test_evaluation -v`
Expected: PASS.

### Task 3: Command-Line Scripts and Docs

**Files:**
- Create: `scripts/evaluate.py`
- Create: `scripts/plot_results.py`
- Modify: `README.md`

- [ ] **Step 1: Add `scripts/evaluate.py` with options for seeds, policies, steps, and output dir**
- [ ] **Step 2: Add `scripts/plot_results.py` to regenerate plots from an existing step CSV**
- [ ] **Step 3: Document commands and output files in README**

Run:
- `python -m unittest discover -v`
- `python scripts/evaluate.py --seeds 0 1 --policies nearest_frontier round_robin --max-steps 12 --output-dir results/smoke`

Expected:
- Tests pass.
- `results/smoke/eval_steps.csv`, `results/smoke/eval_summary.csv`, `results/smoke/coverage_curve.png`, and `results/smoke/reward_curve.png` exist.
