"""Plot paired RGB passage pilot runs without grid lines."""

import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from run_matched_vision_training import csv_rows


CONDITIONS = ('lidar', 'fusion', 'passage')
LABELS = ('LiDAR only', 'Fixed fusion', 'Fusion + passage')
COLOURS = ('#2E628C', '#BC7040', '#4F7864', '#7C5A89')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('navigation', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    cases = defaultdict(dict)
    for row in csv_rows(args.navigation):
        cases[row['layout'], row['environment_seed']][row['condition']] = row
    layout_names = {'passage_left_open': 'Left opening', 'passage_right_open': 'Right opening'}
    if {layout for layout, _ in cases} != layout_names.keys():
        raise ValueError('Both mirrored layouts are required')
    if any(group.keys() != set(CONDITIONS) for group in cases.values()):
        raise ValueError('Every plot row requires the three matched conditions')
    horizons = {int(row['horizon']) for group in cases.values() for row in group.values()}
    if len(horizons) != 1:
        raise ValueError('Expected one simulation horizon for the plot')
    horizon = horizons.pop()
    plt.rcParams.update({
        'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
        'axes.grid': False, 'pdf.fonttype': 42,
    })
    figure, axes = plt.subplots(2, 2, figsize=(8.4, 6), sharex='col', sharey='row')
    for axis, title in zip(axes[0], layout_names.values()):
        axis.set_title(title, loc='left', fontweight='bold')
    for index, ((layout, seed), group) in enumerate(sorted(cases.items())):
        col = list(layout_names).index(layout)
        color = COLOURS[index % len(COLOURS)]
        coverage = [float(group[condition]['coverage_pct']) for condition in CONDITIONS]
        axes[0, col].plot(range(3), coverage, marker='o', color=color, label=f'Seed {seed}')
        crossing = [group[condition]['first_crossing_s'] for condition in CONDITIONS]
        plotted = [float(value) if value else horizon for value in crossing]
        axes[1, col].plot(range(3), plotted, color=color, alpha=0.8)
        for x, (value, event) in enumerate(zip(plotted, crossing)):
            marker = 'o' if event else '^'
            axes[1, col].plot(x, value, color=color, marker=marker, linestyle='none')
    for top, bottom in zip(*axes):
        top.legend(frameon=False, loc='lower right', fontsize=8)
        top.set_ylim(0, 102)
        bottom.set_xticks(range(3))
        bottom.set_xticklabels(LABELS, fontsize=9)
        bottom.set_ylim(0, horizon + 5)
    axes[0, 0].set_ylabel('Final coverage (%)')
    axes[1, 0].set_ylabel('Time to first crossing (s)')
    figure.text(0.5, 0.03, f'Triangles indicate no crossing observed by {horizon} s.',
                ha='center', color='#404040', fontsize=9)
    figure.tight_layout(rect=(0, 0.055, 1, 1))
    args.destination.mkdir(parents=True, exist_ok=True)
    for extension in ('pdf', 'png'):
        with (args.destination / f'matched_navigation.{extension}').open('xb') as stream:
            figure.savefig(stream, format=extension, dpi=220)
    plt.close(figure)


if __name__ == '__main__':
    main()
