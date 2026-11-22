"""Check calibration image identities, labels and held-out run isolation."""

import argparse
import csv
import hashlib
import json
import shutil
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifests", nargs="+", type=Path)
    parser.add_argument("--held-out-run", action="append", required=True)
    parser.add_argument("--isolated-output", type=Path,
                        help="Create a new hashed dataset view excluding shared image content")
    args = parser.parse_args()
    held_out = set(args.held_out_run)
    paths = defaultdict(list)
    contents = defaultdict(set)
    seed_groups = defaultdict(set)
    sources = {}
    source_rows = defaultdict(list)
    runs = set()
    validation_rows = 0
    for manifest in args.manifests:
        manifest = manifest.resolve()
        sources[str(manifest)] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        with manifest.open(newline="") as stream:
            for row in csv.DictReader(stream):
                image = (manifest.parent / row["image_path"]).resolve()
                if manifest.parent not in image.parents:
                    raise ValueError(f"Image escapes its manifest directory: {image}")
                run = row["run_id"]
                if not run:
                    raise ValueError(f"Missing run ID in {manifest}")
                runs.add(run)
                validation = run in held_out
                validation_rows += int(validation)
                labels = tuple(float(row[direction]) for direction in ("left", "centre", "right"))
                if not all(0.0 <= label <= 1.0 for label in labels):
                    raise ValueError(f"Invalid clearance labels for {image}")
                if "camera_stamp_ns" in row:
                    camera = int(row["camera_stamp_ns"])
                    scan = int(row["scan_stamp_ns"])
                    if min(camera, scan) <= 0 or abs(camera - scan) > int(row["sync_tolerance_ns"]):
                        raise ValueError(f"Invalid timestamp pairing for {image}")
                    seed_groups[row["layout"], int(row["experiment_seed"])].add(validation)
                digest = hashlib.sha256(image.read_bytes()).hexdigest()
                if "image_sha256" in row and row["image_sha256"] != digest:
                    raise ValueError(f"Image content differs from its manifest: {image}")
                paths[image].append((run, validation, labels))
                contents[digest].add(validation)
                if args.isolated_output is not None:
                    if "image_sha256" not in row or "camera_stamp_ns" not in row:
                        raise ValueError("An isolated view requires hashed, timestamped captures")
                    source_rows[run].append((
                        {**row, "image_path": image.relative_to(manifest.parent).as_posix()},
                        image, digest))
    if held_out - runs or not runs - held_out:
        raise ValueError("Each requested held-out run and a separate training or calibration run are required")

    duplicate_paths = [entries for entries in paths.values() if len(entries) > 1]
    shared_paths = [entries for entries in duplicate_paths if len({entry[1] for entry in entries}) > 1]
    cross_run_paths = [entries for entries in duplicate_paths if len({entry[0] for entry in entries}) > 1]
    shared_content = sum(len(split) > 1 for split in contents.values())
    shared_seeds = sum(len(split) > 1 for split in seed_groups.values())
    report = {
        "manifest_sha256": sources,
        "rows": sum(map(len, paths.values())),
        "held_out_runs": sorted(held_out),
        "held_out_rows": validation_rows,
        "duplicate_paths": len(duplicate_paths),
        "cross_run_paths": len(cross_run_paths),
        "cross_run_conflicting_label_paths": sum(
            len({entry[2] for entry in entries}) > 1 for entries in cross_run_paths),
        "paths_shared_across_split": len(shared_paths),
        "held_out_rows_at_shared_paths": sum(entry[1] for entries in shared_paths for entry in entries),
        "image_hashes_shared_across_split": shared_content,
        "layout_seed_groups_shared_across_split": shared_seeds,
    }
    if args.isolated_output is not None:
        if duplicate_paths or shared_seeds:
            raise ValueError("The source capture has repeated paths or shared layout and seed groups")
        retained = {run: [row for row in rows if len(contents[row[2]]) == 1]
                    for run, rows in source_rows.items()}
        if any(not rows or Path(run).name != run or run in (".", "..")
               for run, rows in retained.items()):
            raise ValueError("Every run must retain valid capture rows")
        args.isolated_output.mkdir()
        for run, rows in sorted(retained.items()):
            directory = args.isolated_output / run
            directory.mkdir()
            with (directory / "manifest.csv").open("x", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0][0]))
                writer.writeheader()
                for row, image, _ in rows:
                    destination = directory / row["image_path"]
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    with image.open("rb") as source, destination.open("xb") as target:
                        shutil.copyfileobj(source, target)
                    writer.writerow(row)
        report["excluded_rows_by_run"] = {run: len(rows) - len(retained[run])
                                          for run, rows in sorted(source_rows.items())}
        report["retained_rows"] = sum(map(len, retained.values()))
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.isolated_output is None and (duplicate_paths or shared_content or shared_seeds):
        raise SystemExit("Calibration audit detected duplicate paths or across-split reuse")


if __name__ == "__main__":
    main()
