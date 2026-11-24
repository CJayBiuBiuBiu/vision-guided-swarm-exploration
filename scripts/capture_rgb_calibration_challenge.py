"""Collect a second, independent test set after the factorial comparison."""

from pathlib import Path
import sys

import capture_rgb_training_test as capture
from run_clearance_study import digest, preserve_json


if __name__ == "__main__":
    capture.CASES = (("hu_aligned", 66314), ("hu_layout_c", 66315), ("hu_layout_b", 66316))
    if len(sys.argv) == 4:
        destination = Path(sys.argv[3])
        destination.mkdir(parents=True, exist_ok=True)
        preserve_json(destination / "challenge_plan.json", dict(
            cases=[list(case) for case in capture.CASES], wrapper_sha256=digest(Path(__file__))))
    capture.main()
