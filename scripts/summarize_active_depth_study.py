"""Summarize the verified active-waypoint pairs and draw a diagnostic figure."""

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path


def write_rows(path, rows, fields):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def paired_deltas(rows):
    grouped = defaultdict(dict)
    for row in rows:
        base = grouped[(row["scene"], int(row["seed"]))]
        if row["condition"] in base:
            raise ValueError("Duplicate condition in a paired block")
        base[row["condition"]] = row
    pairs = []
    for (scene, seed), arms in grouped.items():
        if set(arms) != {"depth", "recheck"}:
            raise ValueError(f"Incomplete paired block at {scene}, {seed}")
        baseline, recheck = arms["depth"], arms["recheck"]
        prior, current = (float(arm["coverage_pct"]) for arm in (baseline, recheck))
        prior_contacts, current_contacts = (int(arm["block_contacts"]) for arm in (baseline, recheck))
        pairs.append(dict(scene=scene, seed=seed, assignment_coverage_pct=prior,
                          recheck_coverage_pct=current, coverage_difference_pp=round(current-prior, 5),
                          assignment_contacts=prior_contacts, recheck_contacts=current_contacts,
                          contacts_difference=current_contacts-prior_contacts,
                          shadow_confirmed=int(baseline["active_confirmed"]),
                          recheck_interruptions=int(recheck["active_interrupted"]),
                          recheck_depth_only_at_box=int(recheck["active_depth_only_at_box"]),
                          recheck_replacements=int(recheck["active_replacements"]),
                          recheck_replacement_arrivals=int(recheck["active_replacement_arrivals"])))
    return pairs


def interventions(study):
    recovered = []
    for marker in sorted(study.glob("*_recheck.complete.json")):
        run = json.loads(marker.read_text())
        events = [json.loads(line) for line in
                  (study / run["directory"] / "decisions.jsonl").read_text().splitlines()]
        new = {event["depth_interruption_id"]: event for event in events
               if event["event"] == "assigned" and "depth_interruption_id" in event}
        outcomes = {event["attempt_id"]: event for event in events if event["event"] == "outcome"}
        for event in events:
            if event["event"] != "depth_active_check" or not event["applied"]:
                continue
            key = event["interruption_id"]
            assignment = new.get(key, {})
            outcome = outcomes.get(assignment.get("attempt_id"), {})
            lag = assignment["elapsed_sim_s"] - event["elapsed_sim_s"] if assignment else None
            same_frontier = assignment.get("selected_frontier_id") == event["frontier_id"]
            if same_frontier and lag < 3.0 - 1e-6:
                raise ValueError("Interrupted frontier was selected during its exclusion window")
            recovered.append(dict(scene=run["result"]["scene"], seed=run["result"]["seed"],
                                  robot=event["robot"], interruption_id=key,
                                  elapsed_sim_s=event["elapsed_sim_s"],
                                  previous_depth_stamp=event["previous_stamp"],
                                  confirming_depth_stamp=event["depth_stamp"],
                                  paired_laser_stamp=event["laser_stamp"],
                                  paired_laser_hit=(event["laser_hit"] is not None
                                                    if event["laser_stamp"] is not None else None),
                                  hit_x=event["depth_hit"]["xy"][0], hit_y=event["depth_hit"]["xy"][1],
                                  interrupted_frontier=event["frontier_id"],
                                  replacement_frontier=assignment.get("selected_frontier_id"),
                                  same_frontier=same_frontier if assignment else None,
                                  assignment_delay_s=lag, outcome=outcome.get("outcome"),
                                  outcome_time_s=outcome.get("elapsed_sim_s")))
    return recovered


def plot(path, pairs):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    rows = list(reversed(pairs))
    labels = [f"{'Low boxes' if row['scene'] == 'low_blocks' else 'Clear'} · {row['seed']}"
              for row in rows]
    fig, (coverage, contacts) = plt.subplots(1, 2, figsize=(10.2, 3.8), sharey=True)
    levels = range(len(rows))
    for axis, key, label in (
            (coverage, "coverage_difference_pp", "Coverage change (percentage points)"),
            (contacts, "contacts_difference", "Change in low-box contact episodes")):
        axis.axvline(0, color="#8b929b", linewidth=1)
        values = [row[key] for row in rows]
        axis.scatter(values, list(levels), s=55,
                     color=["#8b929b" if row["scene"] == "clear" else "#205d79" for row in rows])
        axis.set(xlabel=label, yticks=list(levels), ylim=(-0.5, len(rows)-0.5))
        for edge in ("top", "right"):
            axis.spines[edge].set_visible(False)
        extent = max(0.5, max(abs(value) for value in values)) * 1.15
        axis.set_xlim(-extent, extent)
    coverage.set_yticklabels(labels)
    contacts.xaxis.set_major_locator(MaxNLocator(integer=True))
    fig.suptitle("Active waypoint recheck minus assignment-only depth", fontsize=13)
    fig.subplots_adjust(left=0.21, right=0.97, top=0.83, bottom=0.19, wspace=0.27)
    for suffix in ("pdf", "png"):
        fig.savefig(path / f"active_depth_pairs.{suffix}", dpi=190)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with (args.study / "pilot_summary.csv").open(newline="") as stream:
        summaries = list(csv.DictReader(stream))
    pairs = paired_deltas(summaries)
    recorded = interventions(args.study)
    if len(recorded) != sum(row["recheck_interruptions"] for row in pairs):
        raise ValueError("Intervention trace disagrees with the audited run counts")
    for pair in pairs:
        assignments = [row for row in recorded if (row["scene"], row["seed"]) ==
                       (pair["scene"], pair["seed"]) and row["replacement_frontier"] is not None]
        changed = [row for row in assignments if not row["same_frontier"]]
        pair["recheck_changed_frontier_ids"] = len(changed)
        pair["recheck_changed_frontier_arrivals"] = sum(row["outcome"] == "reached"
                                                        for row in changed)
    args.output.mkdir()
    write_rows(args.output / "paired_deltas.csv", pairs, list(pairs[0]))
    fields = ("scene seed robot interruption_id elapsed_sim_s previous_depth_stamp "
              "confirming_depth_stamp paired_laser_stamp paired_laser_hit hit_x hit_y "
              "interrupted_frontier replacement_frontier same_frontier assignment_delay_s outcome outcome_time_s")
    write_rows(args.output / "interventions.csv", recorded, fields.split())
    plot(args.output, pairs)


if __name__ == "__main__":
    main()
