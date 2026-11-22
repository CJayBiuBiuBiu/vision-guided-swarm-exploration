"""Measure first logged in-window gate decisions after each exposure change."""

import argparse
import csv
import json
from pathlib import Path

from summarize_clearance_study import save_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.study / "experiment_manifest.json").read_text())
    transitions = {boundary // 60: boundary for boundary in range(60, manifest["horizon_s"], 60)}
    results = []
    for layout, seed, _, condition in manifest["schedule"]:
        if condition != "mixed_adaptive":
            continue
        name = f"{layout}_{seed}_{condition}"
        marker = json.loads((args.study / (name + ".complete.json")).read_text())
        attempt = marker["attempt"]
        if Path(attempt).name != attempt or not attempt.startswith(name + "."):
            raise ValueError(f"Invalid completion marker for {name}")
        first = {}
        names = set()
        with (args.study / attempt / "metrics.csv").open(newline="") as stream:
            for row in csv.DictReader(stream):
                diagnostics = json.loads(row["fusion_diagnostics_json"])
                names.update(diagnostics)
                for robot, sample in diagnostics.items():
                    window = sample["mixed_control_window"]
                    quality = sample["camera_quality"]
                    if window not in transitions or sample["mixed_rgb_window"] != window or quality is None:
                        continue
                    should_accept = window % 2 == 0
                    if (quality[2] == "accepted") != should_accept:
                        continue
                    stamp = sample["mixed_control_stamp_s"]
                    key = window, robot
                    if key not in first or stamp < first[key]:
                        first[key] = stamp
        if len(names) != 3:
            raise ValueError(f"Expected per-robot telemetry for all three robots in {name}")
        for window, boundary in transitions.items():
            for robot in sorted(names):
                stamp = first.get((window, robot))
                results.append({
                    "layout": layout, "seed": seed, "robot": robot, "window": window,
                    "lighting": "dark" if window % 2 else "clean", "boundary_stamp_s": boundary,
                    "first_observed_decision_s": stamp,
                    "observed_delay_s": stamp - boundary if stamp is not None else None,
                })
    if not results:
        raise ValueError("No adaptive exposure transitions were available")
    save_csv(args.output, results)


if __name__ == "__main__":
    main()
