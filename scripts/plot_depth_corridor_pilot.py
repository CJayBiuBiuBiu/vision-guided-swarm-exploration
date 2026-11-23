"""Plot a logged depth retargeting and the matched pilot contact counts."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np

from run_depth_corridor_study import BLOCK_CENTRES, BLOCK_SIZE, near_test_block


CONDITIONS = ("lidar", "fusion", "depth")
LABELS = ("LiDAR", "Fixed fusion", "Depth guidance")
COLORS = ("#2c6083", "#a25231", "#227058")


def first_arrived_retarget(archive):
    # Prefer the independent pilot. The earlier smoke run is a fallback example.
    for stage in ("pilot", "smoke"):
        for marker in sorted((archive / stage).glob("low_blocks_*_depth.complete.json")):
            run = json.loads(marker.read_text())
            path = archive / stage / run["directory"] / "decisions.jsonl"
            events = [json.loads(line) for line in path.read_text().splitlines()]
            reached = {event["attempt_id"] for event in events
                       if event["event"] == "outcome" and event["outcome"] == "reached"}
            selections = [event for event in events if event["event"] in ("depth_decision", "assigned")]
            for decision, assignment in zip(selections, selections[1:]):
                if decision["event"] != "depth_decision" or assignment["event"] != "assigned":
                    continue
                first = decision["candidates"][0]
                if (not decision["paired"] or first["laser_hit"] is not None
                        or not near_test_block(first["depth_hit"])
                        or decision["selected"] in (None, first["frontier_id"])
                        or assignment["attempt_id"] not in reached):
                    continue
                return dict(stage=stage, seed=run["result"]["seed"], decision=decision,
                            assignment=assignment)
    raise ValueError("No matched depth-only retargeting with a reached outcome in the archive")


def draw_decision(ax, example):
    decision = example["decision"]
    origin = decision["robot_xy"]
    initial = decision["candidates"][0]
    selected = next(candidate for candidate in decision["candidates"]
                    if candidate["frontier_id"] == decision["selected"])
    hit = initial["depth_hit"]["xy"]
    points = (origin, initial["waypoint"], selected["waypoint"], hit)
    limits = [(min(values) - 0.36, max(values) + 0.36) for values in zip(*points)]
    for x, y in BLOCK_CENTRES:
        width, length, _ = BLOCK_SIZE
        ax.add_patch(Rectangle((x - width / 2, y - length / 2), width, length,
                               facecolor="#d6d8dc", edgecolor="#5b616a", alpha=0.85))
    start = np.asarray(origin)
    end = np.asarray(initial["waypoint"])
    delta = end - start
    normal = np.array([-delta[1], delta[0]]) * 0.175 / np.linalg.norm(delta)
    ax.add_patch(Polygon([start + normal, end + normal, end - normal, start - normal],
                         facecolor=COLORS[1], edgecolor="none", alpha=0.14))
    for waypoint, label, style, color in (
            (initial["waypoint"], "Original command", "--", COLORS[1]),
            (selected["waypoint"], "Replacement command", "-", COLORS[2])):
        ax.plot([origin[0], waypoint[0]], [origin[1], waypoint[1]], style,
                marker="o", color=color, label=label, linewidth=2)
    ax.scatter(*hit, s=80, color="#24262b", marker="x", label="Depth hit", zorder=4)
    ax.scatter(*origin, s=56, color=COLORS[0], marker="s", label="Robot pose", zorder=5)
    ax.set(xlim=limits[0], ylim=limits[1], xlabel="World x (m)", ylabel="World y (m)",
           title=f"Logged retargeting · {example['stage']} seed {example['seed']}")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="lower left", fontsize=9)


def draw_results(coverage, contacts, rows):
    indexed = {(row["scene"], row["condition"]): row for row in rows}
    for index, (key, label, color) in enumerate(zip(CONDITIONS, LABELS, COLORS)):
        values = [float(indexed[scene, key]["coverage_pct"]) for scene in ("low_blocks", "clear")]
        offset = (index - 1) * 0.11
        coverage.scatter([offset, 1 + offset], values, s=42, color=color, label=label)
        count = int(indexed["low_blocks", key]["block_contacts"])
        contacts.bar(index, count, width=0.6, color=color)
        contacts.text(index, count + 0.25, str(count), ha="center", va="bottom", fontsize=9)
    coverage.set(xlim=(-0.15, 1.15), ylabel="Final coverage (%)", title="75 s pilot coverage",
                 xticks=[0, 1], xticklabels=["Low boxes (62400)", "Clear (62401)"])
    coverage.legend(frameon=False, loc="best", fontsize=9)
    contacts.set(title="Physical contacts with low boxes", ylabel="Contact episodes",
                 xticks=[0, 1, 2], xticklabels=["LiDAR", "Fusion", "Depth"])
    contacts.set_ylim(0, contacts.get_ylim()[1] * 1.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("output", type=Path, help="new directory for the paired figure and event")
    args = parser.parse_args()
    example = first_arrived_retarget(args.archive)
    with (args.archive / "pilot/pilot_summary.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 6:
        raise ValueError("Expected all six pilot runs")
    args.output.mkdir()
    with (args.output / "retarget_example.json").open("x") as stream:
        json.dump(example, stream, indent=2, allow_nan=False)
        stream.write("\n")
    figure = plt.figure(figsize=(11.5, 6))
    layout = figure.add_gridspec(2, 2, width_ratios=[1.15, 1.0], hspace=0.58, wspace=0.3)
    ax = figure.add_subplot(layout[:, 0])
    coverage = figure.add_subplot(layout[0, 1])
    contacts = figure.add_subplot(layout[1, 1])
    draw_decision(ax, example)
    draw_results(coverage, contacts, rows)
    for axes in (ax, coverage, contacts):
        for edge in ("top", "right"):
            axes.spines[edge].set_visible(False)
    figure.subplots_adjust(left=0.09, right=0.97, top=0.93, bottom=0.17)
    for suffix in ("pdf", "png"):
        figure.savefig(args.output / f"depth_corridor_pilot.{suffix}", dpi=200)
    plt.close(figure)


if __name__ == "__main__":
    main()
