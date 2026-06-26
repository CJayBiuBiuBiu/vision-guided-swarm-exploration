import csv
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import torch

from eavf_mappo.marl.actor import ResidualFrontierActor


class CheckpointEvaluationTests(unittest.TestCase):
    def test_evaluate_checkpoint_script_writes_step_and_summary_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            checkpoint = tmp_path / "checkpoint.pt"
            actor = ResidualFrontierActor(d_token=10, d_model=16, tau=0.5)
            torch.save(
                {
                    "actor": actor.state_dict(),
                    "args": {
                        "d_model": 16,
                        "tau": 0.5,
                        "frontier_k": 4,
                        "n_robots": 2,
                        "map_size": 6.0,
                        "resolution": 0.5,
                        "sensor_range": 1.2,
                        "sensor_rays": 24,
                    },
                },
                checkpoint,
            )

            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/evaluate_checkpoint.py",
                    "--checkpoint",
                    str(checkpoint),
                    "--seeds",
                    "0",
                    "--map-types",
                    "office",
                    "--max-steps",
                    "4",
                    "--output-dir",
                    str(tmp_path),
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=60,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            steps_path = tmp_path / "checkpoint_eval_steps.csv"
            summary_path = tmp_path / "checkpoint_eval_summary.csv"
            self.assertTrue(steps_path.exists())
            self.assertTrue(summary_path.exists())

            with summary_path.open(newline="", encoding="utf-8") as fh:
                row = next(csv.DictReader(fh))
            for key in [
                "checkpoint",
                "map_type",
                "seed",
                "final_coverage",
                "total_return",
                "collision_total",
                "overlap_total",
                "frontier_action_total",
                "release_action_total",
            ]:
                self.assertIn(key, row)


if __name__ == "__main__":
    unittest.main()
