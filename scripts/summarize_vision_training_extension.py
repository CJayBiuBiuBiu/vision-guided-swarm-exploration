"""Compare fixed validation cases across budgets and summarize the held-out test."""

import argparse
from collections import defaultdict
from pathlib import Path
from statistics import mean

from run_matched_vision_training import csv_rows, write_csv
from summarize_matched_vision_training import paired_evaluation


LAYOUTS = {"hu_aligned": "Hu-aligned", "hu_layout_c": "Layout C"}


def validation_tables(pilot, extension, through):
    roots = {8: pilot, 16: extension / "update_16"}
    if through == 32:
        roots[32] = extension / "update_32"
    paired = []
    summaries = []
    cases = None
    for updates, root in roots.items():
        rows = paired_evaluation(csv_rows(root / "evaluation_runs.csv"))
        keys = {(row["training_seed"], row["layout"], row["evaluation_seed"]) for row in rows}
        if cases is None:
            cases = keys
        if len(rows) != 6 or keys != cases:
            raise ValueError("Validation cases differ across budgets")
        paired.extend(dict(updates=updates, **row) for row in rows)
        for layout in LAYOUTS:
            block = [row for row in rows if row["layout"] == layout]
            summaries.append(dict(updates=updates, layout=layout, **coverage_means(block)))
    return paired, summaries


def coverage_means(block):
    lidar = mean(row["lidar_coverage_pct"] for row in block)
    fusion = mean(row["fusion_coverage_pct"] for row in block)
    differences = [row["difference_coverage_pct"] for row in block]
    return dict(pairs=len(block), lidar_coverage_pct=round(lidar, 6),
                fusion_coverage_pct=round(fusion, 6), difference_coverage_pct=round(mean(differences), 6),
                fusion_ahead=sum(value > 0 for value in differences))


def test_tables(extension):
    paired = paired_evaluation(csv_rows(extension / "test_32/evaluation_runs.csv"))
    grouped = defaultdict(list)
    for row in paired:
        grouped[row["layout"], row["training_seed"]].append(row)
    if len(grouped) != 6 or len(paired) != 18 or any(len(group) != 3 for group in grouped.values()):
        raise ValueError("Final test requires three cases for each layout and lineage")
    lineage_means = [dict(layout=layout, training_seed=seed, **coverage_means(rows))
                     for (layout, seed), rows in sorted(grouped.items())]
    summaries = []
    for layout in LAYOUTS:
        rows = [row for row in paired if row["layout"] == layout]
        lineages = [row for row in lineage_means if row["layout"] == layout]
        summaries.append(dict(layout=layout, **coverage_means(rows),
                              training_seed_pairs=len(lineages),
                              fusion_mean_ahead=sum(row["difference_coverage_pct"] > 0
                                                    for row in lineages)))
    return paired, lineage_means, summaries


def plot_comparison(output, validation, test_lineages):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    two_panels = bool(test_lineages)
    fig, axes = plt.subplots(1, 2 if two_panels else 1, figsize=(10.2 if two_panels else 6, 4.3))
    trend = axes[0] if two_panels else axes
    colors = {"hu_aligned": "#25607d", "hu_layout_c": "#a05036"}
    budgets = sorted({row["updates"] for row in validation})
    for layout, label in LAYOUTS.items():
        rows = [row for row in validation if row["layout"] == layout]
        trend.plot([row["updates"] for row in rows],
                   [row["difference_coverage_pct"] for row in rows],
                   color=colors[layout], marker="o", label=label)
    trend.axhline(0, color="#8b9199", linewidth=1)
    trend.set(xlabel="MAPPO updates", ylabel="RGB–LiDAR minus LiDAR (percentage points)",
              title="Fixed 65 s validation cases", xticks=budgets)
    trend.legend(frameon=False)
    if two_panels:
        final = axes[1]
        final.axvline(0, color="#8b9199", linewidth=1)
        labels = []
        for row in test_lineages:
            final.scatter(row["difference_coverage_pct"], len(labels), color=colors[row["layout"]], s=54)
            labels.append(f"{LAYOUTS[row['layout']]} · {row['training_seed']}")
        final.set(title="New 120 s test seeds", xlabel="Mean paired difference (percentage points)",
                  yticks=list(range(len(labels))))
        final.set_yticklabels(labels, fontsize=9)
        final.xaxis.set_major_locator(MaxNLocator(nbins=5))
    for ax in fig.axes:
        for edge in ("top", "right"):
            ax.spines[edge].set_visible(False)
    fig.subplots_adjust(left=0.12 if two_panels else 0.16, right=0.98,
                        top=0.89, bottom=0.18, wspace=0.58)
    for suffix in ("pdf", "png"):
        fig.savefig(output / f"vision_training_budgets.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pilot", type=Path)
    parser.add_argument("extension", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--through", choices=(16, 32), type=int, default=32)
    args = parser.parse_args()
    validation, trend = validation_tables(args.pilot, args.extension, args.through)
    lineages = []
    args.output.mkdir()
    write_csv(args.output / "validation_pairs.csv", validation)
    write_csv(args.output / "validation_summary.csv", trend)
    if args.through == 32:
        test, lineages, summary = test_tables(args.extension)
        write_csv(args.output / "test_pairs.csv", test)
        write_csv(args.output / "test_lineage_means.csv", lineages)
        write_csv(args.output / "test_summary.csv", summary)
    plot_comparison(args.output, trend, lineages)
