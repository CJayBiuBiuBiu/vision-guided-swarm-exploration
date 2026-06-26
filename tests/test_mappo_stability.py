import csv
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import torch

from eavf_mappo.marl.mappo import compute_value_loss, normalize_tensor


class MappoStabilityTests(unittest.TestCase):
    def test_normalize_tensor_has_zero_mean_and_unit_std(self):
        normalized, mean, std = normalize_tensor(torch.tensor([1.0, 2.0, 3.0, 4.0]))

        self.assertAlmostEqual(float(normalized.mean()), 0.0, places=6)
        self.assertAlmostEqual(float(normalized.std()), 1.0, places=6)
        self.assertAlmostEqual(float(mean), 2.5, places=6)
        self.assertGreater(float(std), 0.0)

    def test_normalize_tensor_handles_constant_values(self):
        normalized, mean, std = normalize_tensor(torch.tensor([5.0, 5.0, 5.0]))

        self.assertTrue(torch.equal(normalized, torch.zeros_like(normalized)))
        self.assertAlmostEqual(float(mean), 5.0, places=6)
        self.assertGreater(float(std), 0.0)

    def test_value_loss_clip_penalizes_large_critic_step(self):
        values = torch.tensor([10.0])
        old_values = torch.tensor([0.0])
        returns = torch.tensor([10.0])

        unclipped = compute_value_loss(values, old_values, returns, clip_eps=None)
        clipped = compute_value_loss(values, old_values, returns, clip_eps=0.2)

        self.assertEqual(float(unclipped), 0.0)
        self.assertGreater(float(clipped), 90.0)

    def test_train_log_includes_stability_diagnostics(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/train.py",
                    "--iterations",
                    "1",
                    "--rollout-length",
                    "2",
                    "--n-robots",
                    "2",
                    "--map-size",
                    "6",
                    "--sensor-range",
                    "1.2",
                    "--d-model",
                    "16",
                    "--output-dir",
                    tmp,
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=60,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            with (Path(tmp) / "train_log.csv").open(newline="", encoding="utf-8") as fh:
                row = next(csv.DictReader(fh))
            for key in ["return_mean", "return_std", "value_pred_mean", "approx_kl"]:
                self.assertIn(key, row)
                self.assertTrue(torch.isfinite(torch.tensor(float(row[key]))))


if __name__ == "__main__":
    unittest.main()
