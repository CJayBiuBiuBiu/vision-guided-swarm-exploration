"""Link low-level stuck reports to active exploration goals and repeated locations."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re
from statistics import median

from run_matched_vision_training import write_csv


STUCK_REPORT = re.compile(r"^(\d+(?:\.\d+)?)\s+WARN /(robot\d+)_explorer .*stuck detected")
STUCK_POSITION = re.compile(
    r"^(\d+(?:\.\d+)?)\s+WARN /exploration_coordinator .*blacklisting stuck area near "
    r"\(([-\d.]+), ([-\d.]+)\)")


def stuck_reports(directory, expected_count):
    logs = [path for path in directory.glob("ros_logs/*/rosout.log") if not path.parent.is_symlink()]
    if len(logs) != 1:
        raise ValueError(f"Expected one ROS aggregate log in {directory}")
    reports, positions = [], []
    for line in logs[0].read_text(errors="replace").splitlines():
        if report := STUCK_REPORT.search(line):
            reports.append((float(report[1]), report[2]))
        elif position := STUCK_POSITION.search(line):
            positions.append((float(position[1]), (float(position[2]), float(position[3]))))
    if len(reports) != expected_count or len(positions) != expected_count:
        raise ValueError(f"Stuck logs and final metric differ in {directory}")
    remaining = positions.copy()
    for stamp, robot in reports:
        index = min(range(len(remaining)), key=lambda idx: abs(remaining[idx][0] - stamp))
        received, xy = remaining.pop(index)
        if abs(received - stamp) > 0.10:
            raise ValueError(f"Cannot match a stuck position to its report in {directory}")
        yield stamp, robot, xy


def goal_attempts(events):
    attempts = defaultdict(list)
    active = {}
    for event in events:
        kind, robot = event["event"], event.get("robot")
        if kind == "assigned":
            entry = dict(attempt_id=event["attempt_id"], start=event["elapsed_sim_s"],
                         points=[(event["elapsed_sim_s"], event["selected_xy"])])
            active[robot] = entry
            attempts[robot].append(entry)
        elif kind == "passage_continuation_decision" and event["target_xy"] is not None:
            active[robot]["points"].append((event["elapsed_sim_s"], event["target_xy"]))
        elif kind == "outcome":
            entry = active.pop(robot)
            if entry["attempt_id"] != event["attempt_id"]:
                raise ValueError("Attempt closure differs from assignment")
            entry.update(end=event["elapsed_sim_s"], outcome=event["outcome"])
    if active:
        raise ValueError("Unclosed goal attempts")
    return attempts


def diagnose(directory, result):
    events = [json.loads(line) for line in (directory / "decisions.jsonl").read_text().splitlines()]
    attempts = goal_attempts(events)
    offset = median(event["control_stamp"] - event["elapsed_sim_s"] for event in events
                    if "control_stamp" in event and "elapsed_sim_s" in event)
    positions = defaultdict(list)
    for stamp, robot, xy in stuck_reports(directory, int(result["stuck_events"])):
        elapsed = stamp - offset
        active = [entry for entry in attempts[robot]
                  if entry["start"] <= elapsed + 0.05 and entry["end"] >= elapsed - 0.10]
        attempt = active[-1] if active else None
        goal = None if attempt is None else next(point for time, point in reversed(attempt["points"])
                                                if time <= elapsed + 0.05)
        retired = bool(attempt and attempt["outcome"] == "stuck"
                       and -0.10 <= attempt["end"] - elapsed <= 0.75)
        repeat = any(math.dist(xy, previous) <= 0.90 for _, previous in positions[robot])
        repeat_20s = any(math.dist(xy, previous) <= 0.90 and elapsed - time <= 20.0
                         for time, previous in positions[robot])
        positions[robot].append((elapsed, xy))
        yield dict(scene=result["scene"], seed=result["environment_seed"],
                   strategy=result.get("route_recovery_mode", result.get("passage_strategy")),
                   robot=robot, elapsed_s=round(elapsed, 6), stuck_x=xy[0], stuck_y=xy[1],
                   attempt_id=None if attempt is None else attempt["attempt_id"],
                   goal_x=None if goal is None else goal[0], goal_y=None if goal is None else goal[1],
                   goal_distance_m=None if goal is None else round(math.dist(xy, goal), 6),
                   active_goal=attempt is not None, retired_by_report=retired,
                   active_goal_not_retired=attempt is not None and not retired,
                   repeated_region=repeat, repeated_region_20s=repeat_20s)


def read_archive(formal):
    for marker in sorted(formal.glob("*.complete.json")):
        record = json.loads(marker.read_text())
        yield record["result"], formal / record["directory"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    events, summary = [], []
    for result, directory in read_archive(args.formal):
        rows = list(diagnose(directory, result))
        events.extend(rows)
        summary.append(dict(scene=result["scene"], seed=result["environment_seed"],
                            strategy=result.get("route_recovery_mode", result.get("passage_strategy")),
                            stuck_reports=len(rows), **{
                                key: sum(row[key] for row in rows) for key in (
                                    "active_goal", "retired_by_report", "active_goal_not_retired",
                                    "repeated_region", "repeated_region_20s")}))
    args.output.mkdir(parents=True, exist_ok=False)
    write_csv(args.output / "stuck_summary.csv", summary)
    if events:
        write_csv(args.output / "stuck_events.csv", events)


if __name__ == "__main__":
    main()
