import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class DiagnosticsCliTests(unittest.TestCase):
    def test_diagnose_training_script_writes_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/diagnose_training.py",
                    "--iterations",
                    "1",
                    "--rollout-length",
                    "2",
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
            self.assertTrue((Path(tmp) / "diagnostics.txt").exists())
            self.assertIn("value_loss", (Path(tmp) / "diagnostics.txt").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
