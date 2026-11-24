"""Summarize matched speed-limiter trials and physical low-box contacts."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows, write_csv
from run_depth_velocity_study import FORMAL, MODES, plan
from run_policy_goal_retention import read_json_lines
from run_wait_margin_study import ROBOTS


METRICS = ("block_contacts", "coverage_pct", "path_m", "reached", "assignments",
           "near_collision_events", "stuck_events", "recovery_successes")
COUNTS = ("block_contacts", "controller_ticks", "fresh_ticks", "would_reduce_ticks",
          "reduced_ticks", "stopped_ticks", "laser_full_speed_reduce_ticks", "reached", "assignments")


def aggregate(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row["scene"], row["depth_velocity_mode"]].append(row)
    for (scene, mode), group in sorted(groups.items()):
        totals = {key + "_total": sum(int(row[key]) for row in group) for key in COUNTS}
        means = {key + "_mean": mean(float(row[key]) for row in group) for key in METRICS}
        yield dict(scene=scene, mode=mode, runs=len(group),
                   fresh_pct=100 * totals["fresh_ticks_total"] / totals["controller_ticks_total"],
                   reached_pct=100 * totals["reached_total"] / totals["assignments_total"],
                   **means, **totals)


def match(rows):
    groups = defaultdict(dict)
    for row in rows:
        groups[row["scene"], int(row["environment_seed"])][row["depth_velocity_mode"]] = row
    for scene, seed in FORMAL:
        group = groups[scene, seed]
        if set(group) != set(MODES):
            raise ValueError(f"Missing paired condition for {scene} {seed}")
        before, after = (group[mode] for mode in MODES)
        yield dict(scene=scene, seed=seed,
                   observed_contacts=int(before["block_contacts"]),
                   limited_contacts=int(after["block_contacts"]),
                   limited_ticks=int(after["reduced_ticks"]),
                   **{key + "_delta": round(float(after[key]) - float(before[key]), 6) for key in METRICS})


def contact_warnings(formal, rows):
    for row in rows:
        mode = row["depth_velocity_mode"]
        trial = f"{row['scene']}_{row['environment_seed']}_{mode}"
        entry = json.loads((formal / f"{trial}.complete.json").read_text())
        directory = formal / entry["directory"]
        contact_stamps = defaultdict(set)
        for event in read_json_lines(directory / "decisions.jsonl"):
            if event["event"] == "depth_contact":
                for pair in event["pairs"]:
                    names = {item.split("::")[0] for item in pair}
                    robots = names.intersection(ROBOTS)
                    blocks = [name for name in names if name.startswith("low_block_")]
                    if len(robots) == len(blocks) == 1:
                        contact_stamps[next(iter(robots)), blocks[0]].add(event["contact_stamp"])
        commands = {robot: read_json_lines(directory / f"{robot}_depth_velocity.jsonl")
                    for robot in sorted({robot for robot, _ in contact_stamps})}
        warnings = []
        for (robot, block), stamps in sorted(contact_stamps.items()):
            previous = float("-inf")
            for stamp in sorted(stamps):
                if stamp - previous > 0.30:
                    preceding = [tick["stamp_s"] for tick in commands[robot]
                                 if 0 < stamp - tick["stamp_s"] <= 1.0
                                 and tick["before_mps"] * (1.0 - tick["scale"]) > 0.01]
                    warnings.append(dict(scene=row["scene"], seed=row["environment_seed"], mode=mode,
                                         robot=robot, block=block, contact_stamp_s=stamp,
                                         preceding_candidate_ticks=len(preceding),
                                         first_warning_lead_s=round(stamp - min(preceding), 6)
                                         if preceding else None))
                previous = stamp
        if len(warnings) != int(row["block_contacts"]):
            raise ValueError(f"Contact episode audit differs for {trial}")
        yield from warnings


def plot(rows, output):
    plt.rcParams.update({"axes.grid": False, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    fig, axes = plt.subplots(1, 2, figsize=(8.7, 3.6))
    for ax, key, label in zip(axes, ("block_contacts_delta", "coverage_pct_delta"),
                              ("Low-box contact episode change", "Final coverage change (pp)")):
        ax.axhline(0, color="#646464", linewidth=0.8)
        for position, row in enumerate(rows):
            color = "#325F83" if row["scene"] == "low_blocks" else "#98704D"
            ax.bar(position, row[key], width=0.65, color=color)
        ax.set_xticks(range(len(rows)))
        ax.set_xticklabels([str(row["seed"]) + ("\nclear" if row["scene"] == "clear" else "")
                           for row in rows], fontsize=8)
        ax.set_ylabel(label)
        ax.set_xlabel("Matched seed")
    fig.suptitle("Depth velocity limiter relative to observation only", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    for kind in ("png", "pdf"):
        with (output / f"depth_velocity_pairs.{kind}").open("xb") as stream:
            fig.savefig(stream, format=kind, dpi=220)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = csv_rows(args.formal / "summary.csv")
    schedule = json.loads((args.formal / "experiment_manifest.json").read_text())["runs"]
    if (schedule != plan("formal") or len(rows) != len(schedule)
            or any(any(row[key] != str(value) for key, value in run.items())
                   for row, run in zip(rows, schedule))):
        raise ValueError("Summary differs from the complete matched schedule")
    args.output.mkdir(parents=True, exist_ok=True)
    pairs = list(match(rows))
    write_csv(args.output / "scene_metrics.csv", list(aggregate(rows)))
    write_csv(args.output / "paired_deltas.csv", pairs)
    warnings = list(contact_warnings(args.formal, rows))
    if warnings:
        write_csv(args.output / "contact_episode_warnings.csv", warnings)
    plot(pairs, args.output)


if __name__ == "__main__":
    main()
