"""Summarize early MAPPO training and matched frozen-endpoint evaluations."""

import argparse
from collections import defaultdict
from pathlib import Path
from statistics import mean

from run_matched_vision_training import CUES, csv_rows, write_csv


def paired_evaluation(evaluations):
    groups = defaultdict(dict)
    for row in evaluations:
        key = (int(row["seed"]), row["layout"], int(row["environment_seed"]))
        arms = groups[key]
        if row["cue"] in arms:
            raise ValueError(f"Duplicate evaluation arm in {key}")
        arms[row["cue"]] = row
    pairs = []
    for (seed, layout, environment_seed), arms in groups.items():
        if set(arms) != set(CUES):
            raise ValueError("Every evaluation must contain both frozen endpoints")
        lidar, fusion = (arms[cue] for cue in CUES)
        row = dict(training_seed=seed, layout=layout, evaluation_seed=environment_seed)
        for field in ("coverage_pct", "path_m", "goals_reached", "stuck_events",
                      "near_collision_events", "rgb_pair_availability"):
            baseline = float(lidar[field])
            camera = float(fusion[field])
            row.update({f"lidar_{field}": baseline, f"fusion_{field}": camera,
                        f"difference_{field}": round(camera - baseline, 7)})
        pairs.append(row)
    return pairs


def lineage_endpoints(training):
    endpoints = {}
    for row in training:
        lineage = (int(row["seed"]), row["cue"])
        episode = int(row["episode"])
        expected = endpoints.get(lineage, {}).get("training_episodes", 0) + 1
        if episode != expected:
            raise ValueError(f"Training episode sequence is incomplete for {lineage}")
        endpoints[lineage] = dict(training_seed=lineage[0], cue=lineage[1],
                                  training_episodes=episode, updates=int(row["updates"]),
                                  transitions=episode * int(row["episode_transitions"]),
                                  endpoint_sha256=row["endpoint_sha256"])
    return list(endpoints.values())


def plot_comparison(training, pairs, output):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    colors = {"lidar": "#225e7b", "fusion": "#a04b30"}
    names = {"lidar": "LiDAR", "fusion": "RGB–LiDAR"}
    by_episode = defaultdict(list)
    for row in training:
        by_episode[(row["cue"], int(row["episode"]))].append(float(row["mean_reward"]))
    fig, (reward, final) = plt.subplots(1, 2, figsize=(10.6, 4.5))
    for cue in CUES:
        episodes = sorted(episode for name, episode in by_episode if name == cue)
        groups = [by_episode[(cue, episode)] for episode in episodes]
        reward.plot(episodes, [mean(values) for values in groups],
                    color=colors[cue], marker="o", label=names[cue])
        if len(groups[0]) > 1:
            reward.fill_between(episodes, [min(values) for values in groups],
                                [max(values) for values in groups],
                                color=colors[cue], alpha=0.12)
    reward.set(xlabel="Training episode", ylabel="Mean rollout reward",
               title="Training over paired initializations")
    reward.xaxis.set_major_locator(MaxNLocator(integer=True))
    reward.legend(frameon=False)
    final.axvline(0, color="#8b929b", linewidth=1)
    ordered = sorted(pairs, key=lambda row: (row["training_seed"], row["layout"]), reverse=True)
    labels = [f"{row['training_seed']} · {'Hu-aligned' if row['layout'] == 'hu_aligned' else 'Layout C'}"
              for row in ordered]
    differences = [row["difference_coverage_pct"] for row in ordered]
    final.scatter(differences, range(len(ordered)), s=57,
                  color=[colors["lidar"] if row["layout"] == "hu_aligned" else colors["fusion"]
                         for row in ordered])
    extent = max(1, max(abs(difference) for difference in differences)) * 1.12
    final.set(xlim=(-extent, extent), xlabel="RGB–LiDAR minus LiDAR (percentage points)",
              title="Frozen-policy final coverage")
    final.set_yticks(range(len(labels)))
    final.set_yticklabels(labels, fontsize=9)
    for ax in (reward, final):
        for edge in ("top", "right"):
            ax.spines[edge].set_visible(False)
    fig.subplots_adjust(left=0.1, right=0.97, top=0.9, bottom=0.17, wspace=0.45)
    for suffix in ("pdf", "png"):
        fig.savefig(output / f"matched_vision_training.{suffix}", dpi=190)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    training = csv_rows(args.study / "training_runs.csv")
    endpoints = lineage_endpoints(training)
    evaluations = csv_rows(args.study / "evaluation_runs.csv")
    pairs = paired_evaluation(evaluations)
    hashes = {(row["training_seed"], row["cue"]): row["endpoint_sha256"] for row in endpoints}
    if any(hashes[(int(row["seed"]), row["cue"])] != row["checkpoint_sha256"]
           for row in evaluations):
        raise ValueError("An evaluation run used a checkpoint outside its lineage")
    args.output.mkdir()
    write_csv(args.output / "paired_evaluation.csv", pairs)
    write_csv(args.output / "lineage_endpoints.csv", endpoints)
    plot_comparison(training, pairs, args.output)


if __name__ == "__main__":
    main()
