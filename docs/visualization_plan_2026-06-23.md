# Plan — Graphical feedback for training & evaluation (2026-06-23)

Context: so far the only feedback from training/evaluation runs is CSV
numbers (`train_log.csv`, `eval_steps.csv`/`eval_summary.csv`). This plan
adds two independent, low-risk pieces of visual feedback. Neither touches
the algorithm — both are additive, read-only consumers of existing state.
`matplotlib>=3.5` is already a declared dependency (`pyproject.toml`), no
new dependency needed for plots; the episode renderer needs GIF support,
which matplotlib gets via Pillow (`PillowWriter`) — check it's importable
in this environment before committing to GIF as the only output format
(fall back to a PNG-frame-sequence if Pillow isn't available — see Part 2).

## Part 1 — Training curve plots

**Goal**: turn `train_log.csv` (written by `scripts/train.py`) into a PNG
so training progress (and regressions like the v3-v5 collapse documented in
`docs/handoff_2026-06-23.md`) is visible at a glance instead of read off a
CSV column.

**Where**: add a new function to `eavf_mappo/evaluation/reporting.py`
(it already has the matplotlib-with-warnings-filter setup and a
`_plot_series`-style helper to model this on — don't duplicate that
plumbing, follow the same pattern):

```python
def plot_training_curves(train_log_csv: str | Path, output_path: str | Path) -> None:
    """Render a multi-panel figure from scripts/train.py's train_log.csv.

    Panels (one per subplot, x-axis = iteration):
      1. loss, actor_loss, value_loss (3 lines, log-scale y if range is large)
      2. coverage
      3. entropy, approx_kl (2 lines)
      4. return_mean, return_std (2 lines)
    """
```

Implementation notes:
- Read the CSV with `csv.DictReader` (same as `_read_step_csv` already does
  for the evaluation pipeline — reuse that helper instead of rewriting CSV
  parsing).
- Use `matplotlib.figure.Figure` with `plt.subplots(2, 2, figsize=(12, 8))`
  or similar; reuse the same `matplotlib.use("Agg")` + warnings-filter
  pattern as `_plot_series` so this works headless and doesn't spam
  deprecation warnings.
- `value_loss` and `loss` can swing over several orders of magnitude in a
  bad run (see v3 in the handoff doc, values in the thousands) — use
  `ax.set_yscale("symlog")` for the loss panel so both collapsed-flat and
  spiky runs are readable in the same figure without manual axis tuning.

**Wire it in two places**:
1. `scripts/train.py`: after `write_log(out_dir / "train_log.csv", log_rows)`,
   call `plot_training_curves(out_dir / "train_log.csv", out_dir / "training_curves.png")`
   and print the written path (same style as the existing `print(f"wrote ...")`
   lines already in that script).
2. New standalone CLI `scripts/plot_training_log.py` (mirrors the existing
   `scripts/plot_results.py` for the evaluation pipeline) so an already-written
   `train_log.csv` can be re-plotted without re-running training:
   ```
   python scripts/plot_training_log.py results/sanity_check_v6/train_log.csv \
       --output results/sanity_check_v6/training_curves.png
   ```

**Test**: add `test_plot_training_curves_writes_nonempty_png` to
`tests/test_evaluation.py` (or a new `tests/test_training_plots.py`) —
write a small fake `train_log.csv` (a handful of rows with the columns
`scripts/train.py` already writes), call `plot_training_curves`, assert the
output file exists and `stat().st_size > 0`. Follow the same pattern as the
existing `test_csv_and_plots_are_written` test in `tests/test_evaluation.py`.

## Part 2 — Visual rendering of an episode (see what the robots are doing)

**Goal**: render an episode as an animated GIF — occupancy grid, robot
positions/headings, planned paths, and frontier targets — so exploration
behavior can be watched directly instead of inferred from coverage numbers.
This is also useful for sanity-checking any future training run, the same
way the v6 diagnostic in the handoff doc relied on printing raw actions/masks
— a renderer would have made that diagnosis faster.

**Where**: new module `eavf_mappo/evaluation/visualization.py`.

**Frame renderer** (one function, reused by both single-frame and animation use):

```python
def render_frame(env, ax) -> None:
    """Draw the current env state onto a matplotlib Axes.

    - `env.grid.cells` as an image: 3 colors for FREE / OCCUPIED / UNKNOWN
      (use a fixed ListedColormap so colors are stable across frames).
    - For each robot in `env.robots`: a marker at (x, y) plus a short line
      or arrow showing `theta` (heading).
    - For each robot with a non-empty `robot.path`: draw the path as a
      line (grid cells -> world coords via `env.grid.grid_to_world`).
    - For each robot with `target_frontier_id is not None`: highlight that
      frontier's cluster cells (look it up via `env.current_clusters()`).
    - Title/text annotation with the current step number and coverage ratio.
    """
```

Coordinate handling: `env.grid.cells` is indexed `[row, col]` with `row` ~ y
and `col` ~ x (see `OccupancyGrid.world_to_grid`/`grid_to_world`) — when
calling `ax.imshow`, remember to pass `origin="lower"` and set
`extent=(grid.origin, -grid.origin, grid.origin, -grid.origin)` so the image
aligns with the world coordinates used for robot/path markers; otherwise
robots will not appear in the right place relative to the map (this is the
single most likely place to introduce a coordinate-frame bug — sanity check
by rendering one frame and confirming a robot marker lands inside the free
disk cleared around its spawn point).

**Episode recorder**:

```python
def record_episode(
    env,
    policy_name: str,
    max_steps: int,
    output_path: str | Path,
    fps: int = 6,
    seed: int | None = None,
) -> None:
    """Reset env, step it with eavf_mappo.evaluation.policies.choose_actions
    (or a loaded trained-policy callable, see below), render one frame per
    step, and save as an animated GIF via matplotlib.animation.
    """
```

- Reuse `eavf_mappo.evaluation.policies.choose_actions(env, policy_name, step)`
  for the heuristic baselines (`nearest_frontier`, `round_robin`, `constant`)
  — don't reimplement action selection here.
- Support visualizing a **trained checkpoint** too: add a
  `policy_name="trained"` branch (or a separate `--checkpoint` CLI flag) that
  loads a `ResidualFrontierActor` from a `.pt` file and decides actions via
  `eavf_mappo.marl.rollout.build_decision_inputs` +
  `eavf_mappo.marl.actor.select_action(..., training=False)` +
  `eavf_mappo.marl.action_adapter.decode_action` — this is the same pattern
  already used in the diagnostic snippet in `docs/handoff_2026-06-23.md`
  ("Useful diagnostic snippet"), just driving a renderer instead of a print
  statement. This is the more valuable of the two policy paths since it's
  what will actually answer "is the trained policy exploring or idling."
- Save with `matplotlib.animation.FuncAnimation(...).save(output_path, writer="pillow", fps=fps)`.
  If `PillowWriter`/Pillow isn't available in this environment, fall back to
  saving each frame as `output_path.parent / f"frame_{step:04d}.png"` instead
  of raising — check for this once during implementation and pick whichever
  path actually works here, don't assume.

**CLI script** `scripts/visualize_episode.py` (mirrors `scripts/evaluate.py`'s
argument style):
```
python scripts/visualize_episode.py --policy nearest_frontier --map-type office \
    --map-size 8 --resolution 0.5 --max-steps 60 --seed 0 --output results/viz/episode.gif

python scripts/visualize_episode.py --checkpoint results/sanity_check_v6/checkpoint.pt \
    --map-type office --map-size 8 --resolution 0.5 --frontier-k 8 --max-steps 60 \
    --output results/viz/trained_episode.gif
```

**Test**: `tests/test_cli_scripts.py`-style smoke test — run the script
with a tiny map and few steps (`--max-steps 5`), assert the output file
exists and is non-empty, same pattern as
`test_evaluate_script_runs_from_project_root`. Keep it fast (small map,
few steps) since it runs as a subprocess with a timeout like the other CLI
tests in that file.

## Order of work

1. Part 1 first (training curves) — small, reuses existing plotting code,
   immediately useful for re-checking the v6 result and any future run.
2. Part 2 second (episode rendering) — more new code (coordinate handling
   is the main risk), but most valuable for the trained-checkpoint case
   since it directly answers "is the policy actually exploring."
