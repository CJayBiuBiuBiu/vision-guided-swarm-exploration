from __future__ import annotations

import csv
import math
import warnings
from collections import defaultdict
from pathlib import Path


def summarize_steps(rows: list[dict], coverage_target: float = 0.9) -> list[dict]:
    grouped: dict[tuple[str, int], list[dict]] = defaultdict(list)
    for row in rows:
        grouped[(row["policy"], int(row["seed"]))].append(row)

    summary: list[dict] = []
    for (policy, seed), group in sorted(grouped.items()):
        group = sorted(group, key=lambda row: int(row["step"]))
        final = group[-1]
        time_to_target = math.nan
        for row in group:
            if float(row["coverage"]) >= coverage_target:
                time_to_target = int(row["step"]) + 1
                break
        summary.append(
            {
                "run_id": final["run_id"],
                "policy": policy,
                "seed": seed,
                "steps": len(group),
                "final_coverage": float(final["coverage"]),
                "total_return": float(final["episode_return"]),
                "mean_reward": sum(float(row["mean_reward"]) for row in group) / len(group),
                "time_to_90": time_to_target,
                "collision_total": sum(int(row["collision_count"]) for row in group),
                "overlap_total": sum(int(row["overlap_count"]) for row in group),
                "path_length_total": sum(int(row["path_length"]) for row in group),
            }
        )
    return summary


def write_csv(path: str | Path, rows: list[dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return

    fieldnames = _fieldnames(rows)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: _csv_value(row.get(name, "")) for name in fieldnames})


def plot_step_metrics(
    step_csv: str | Path,
    coverage_png: str | Path,
    reward_png: str | Path,
) -> None:
    rows = _read_step_csv(step_csv)
    coverage_series = _mean_series(rows, metric="coverage")
    reward_series = _mean_series(rows, metric="episode_return")
    _plot_series(coverage_series, "Coverage by Step", "coverage", coverage_png)
    _plot_series(reward_series, "Episode Return by Step", "episode return", reward_png)


def plot_training_curves(train_log_csv: str | Path, output_path: str | Path) -> None:
    rows = _read_step_csv(train_log_csv)
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message=".*deprecated.*", category=Warning)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        xs = _float_column(rows, "iteration")

        loss_ax = axes[0][0]
        _plot_columns(loss_ax, xs, rows, ["loss", "actor_loss", "value_loss"])
        loss_ax.set_title("Losses")
        loss_ax.set_xlabel("iteration")
        loss_ax.set_ylabel("loss")
        loss_ax.set_yscale("symlog")

        coverage_ax = axes[0][1]
        _plot_columns(coverage_ax, xs, rows, ["coverage"])
        coverage_ax.set_title("Coverage")
        coverage_ax.set_xlabel("iteration")
        coverage_ax.set_ylabel("coverage")
        coverage_ax.set_ylim(bottom=0.0)

        policy_ax = axes[1][0]
        _plot_columns(policy_ax, xs, rows, ["entropy", "approx_kl"])
        policy_ax.set_title("Policy Diagnostics")
        policy_ax.set_xlabel("iteration")

        return_ax = axes[1][1]
        _plot_columns(return_ax, xs, rows, ["return_mean", "return_std"])
        return_ax.set_title("Return Statistics")
        return_ax.set_xlabel("iteration")

        for ax in axes.flat:
            ax.grid(True, alpha=0.3)
            if ax.lines:
                ax.legend()
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)


def _fieldnames(rows: list[dict]) -> list[str]:
    names: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in names:
                names.append(key)
    return names


def _csv_value(value):
    if isinstance(value, bool):
        return int(value)
    return value


def _read_step_csv(path: str | Path) -> list[dict]:
    with Path(path).open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _mean_series(rows: list[dict], metric: str) -> dict[str, list[tuple[int, float]]]:
    grouped: dict[tuple[str, int], list[float]] = defaultdict(list)
    for row in rows:
        grouped[(row["policy"], int(row["step"]))].append(float(row[metric]))

    by_policy: dict[str, list[tuple[int, float]]] = defaultdict(list)
    for (policy, step), values in grouped.items():
        by_policy[policy].append((step, sum(values) / len(values)))
    for policy in by_policy:
        by_policy[policy].sort(key=lambda item: item[0])
    return dict(by_policy)


def _plot_columns(ax, xs: list[float], rows: list[dict], columns: list[str]) -> None:
    for column in columns:
        if not any(column in row and row[column] != "" for row in rows):
            continue
        ys = _float_column(rows, column)
        ax.plot(xs, ys, marker="o", linewidth=1.6, markersize=3, label=column)


def _float_column(rows: list[dict], column: str) -> list[float]:
    values: list[float] = []
    for idx, row in enumerate(rows):
        value = row.get(column, "")
        if value == "":
            values.append(float(idx))
        else:
            values.append(float(value))
    return values


def _plot_series(series: dict[str, list[tuple[int, float]]], title: str, ylabel: str, path) -> None:
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message=".*deprecated.*", category=Warning)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for policy, points in sorted(series.items()):
            if not points:
                continue
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            ax.plot(xs, ys, marker="o", linewidth=1.8, label=policy)
        ax.set_title(title)
        ax.set_xlabel("step")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        if series:
            ax.legend()
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
