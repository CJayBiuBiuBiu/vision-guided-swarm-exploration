"""Compare route-aware recovery with its unchanged visual gain baseline."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean, stdev

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from diagnose_route_failures import diagnose
from run_matched_vision_training import csv_rows, write_csv
from run_route_recovery_study import MODES, schedule


METRICS = (
    "coverage_pct", "path_m", "new_area_m2", "new_area_per_m", "stuck_events",
    "stuck_per_10m", "repeated_region", "repeated_region_20s",
    "active_goal_not_retired", "near_collision_events", "robots_crossing", "goals_reached",
)


def derived_rows(formal, rows):
    for row in rows:
        name = f"{row['scene']}_{row['environment_seed']}_{row['route_recovery_mode']}"
        marker = json.loads((formal / f"{name}.complete.json").read_text())
        directory = formal / marker["directory"]
        metrics = csv_rows(directory / "metrics.csv")
        initial, final = metrics[0], metrics[-1]
        resolution = float(final["grid_resolution_m"])
        area = (int(final["coverage_explored_cells"]) - int(initial["coverage_explored_cells"])) * resolution ** 2
        distance = float(final["path_length_total_m"]) - float(initial["path_length_total_m"])
        if area < 0 or distance <= 0:
            raise ValueError(f"Invalid area or distance difference in {directory}")
        events = list(diagnose(directory, row))
        yield dict(**row, initial_coverage_pct=float(initial["coverage_pct"]),
                   new_area_m2=area, path_after_initial_m=distance, new_area_per_m=area / distance,
                   stuck_per_10m=10 * int(row["stuck_events"]) / float(row["path_m"]),
                   **{key: sum(event[key] for event in events) for key in (
                       "repeated_region", "repeated_region_20s", "active_goal_not_retired",
                       "retired_by_report")}), events


def pairs(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[(row["scene"], row["environment_seed"])][row["route_recovery_mode"]] = row
    for (scene, seed), group in groups.items():
        if set(group) != set(MODES):
            raise ValueError(f"Incomplete matched repeat {scene} seed {seed}")
        legacy, guarded = (group[mode] for mode in MODES)
        yield dict(scene=scene, seed=seed,
                   legacy_first_crossing_s=legacy["first_crossing_s"] or None,
                   guarded_first_crossing_s=guarded["first_crossing_s"] or None,
                   **{key + "_delta": round(float(guarded[key]) - float(legacy[key]), 6)
                      for key in METRICS})


def group_summary(rows):
    for mode in MODES:
        group = [row for row in rows if row["route_recovery_mode"] == mode]
        recovery_results = sum(int(row["recovery_results"]) for row in group)
        recovery_successes = sum(int(row["recovery_successes"]) for row in group)
        yield dict(mode=mode, runs=len(group),
                   crossings=sum(int(row["robots_crossing"]) > 0 for row in group),
                   coverage_sd=stdev(float(row["coverage_pct"]) for row in group) if len(group) > 1 else 0.0,
                   runs_90=sum(bool(row["time_to_90_s"]) for row in group),
                   runs_97=sum(bool(row["time_to_97_s"]) for row in group),
                   recovery_results=recovery_results, recovery_successes=recovery_successes,
                   recovery_success_rate=recovery_successes / recovery_results if recovery_results else None,
                   **{key + "_mean": mean(float(row[key]) for row in group) for key in METRICS},
                   **{key + "_total": sum(int(row[key]) for row in group) for key in (
                       "stuck_events", "repeated_region", "repeated_region_20s", "retired_by_report",
                       "active_goal_not_retired", "route_goals_retired", "route_far_goals_retired",
                       "route_target_rejections", "guarded_command_samples", "guarded_escape_samples",
                       "guarded_forward_stops", "guarded_stale_stops")})


def topology_summary(rows):
    for scene in dict.fromkeys(row["scene"] for row in rows):
        subset = [row for row in rows if row["scene"] == scene]
        for result in group_summary(subset):
            yield dict(scene=scene, **result)


def paired_summary(rows):
    return [dict(pairs=len(rows),
                 **{key + "_mean_delta": mean(row[key + "_delta"] for row in rows) for key in METRICS},
                 coverage_improved=sum(row["coverage_pct_delta"] > 0 for row in rows),
                 stuck_reduced=sum(row["stuck_events_delta"] < 0 for row in rows),
                 repeats_reduced=sum(row["repeated_region_delta"] < 0 for row in rows),
                 area_efficiency_improved=sum(row["new_area_per_m_delta"] > 0 for row in rows))]


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.8))
    for i, mode in enumerate(MODES):
        subset = [row for row in rows if row["route_recovery_mode"] == mode]
        x = [j + (i - 0.5) * 0.30 for j in range(len(subset))]
        color = ("#365F82", "#4B8269")[i]
        for ax, key in zip(axes, ("coverage_pct", "repeated_region_20s", "new_area_per_m")):
            ax.bar(x, [float(r[key]) for r in subset], width=0.28, color=color,
                   label=("Legacy recovery", "Route-aware recovery")[i])
    seeds = [row["environment_seed"] for row in rows if row["route_recovery_mode"] == MODES[0]]
    for axis in axes:
        axis.set_xticks(range(len(seeds)))
        axis.set_xticklabels(seeds, rotation=35, ha="right")
        axis.set_xlabel("Matched layout seed")
    axes[0].set_ylabel("Final coverage (%)")
    axes[0].set_ylim(0, 100)
    axes[1].set_ylabel("Repeated stuck reports (20 s)")
    axes[2].set_ylabel("New observed area (m²/m)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    for ext in ("png", "pdf"):
        with (output / f"route_recovery_comparison.{ext}").open("xb") as stream:
            fig.savefig(stream, format=ext, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", choices=("smoke", "formal"), default="formal")
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    manifest = json.loads((args.formal / "experiment_manifest.json").read_text())
    runs, _ = schedule(args.stage)
    if (manifest["runs"] != runs or len(rows) != len(runs)
            or any(any(row[key] != str(value) for key, value in run.items()) for row, run in zip(rows, runs))):
        raise ValueError("Summary does not match the locked schedule")
    args.output.mkdir(parents=True, exist_ok=False)
    derived, events = [], []
    for row, stuck in derived_rows(args.formal, rows):
        derived.append(row)
        events.extend(stuck)
    write_csv(args.output / "derived_metrics.csv", derived)
    if events:
        write_csv(args.output / "stuck_events.csv", events)
    matched = list(pairs(derived))
    write_csv(args.output / "paired_deltas.csv", matched)
    write_csv(args.output / "paired_summary.csv", paired_summary(matched))
    write_csv(args.output / "group_summary.csv", list(group_summary(derived)))
    write_csv(args.output / "topology_summary.csv", list(topology_summary(derived)))
    plot(derived, args.output)


if __name__ == "__main__":
    main()
