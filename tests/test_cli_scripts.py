import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class CliScriptTests(unittest.TestCase):
    def test_evaluate_script_runs_from_project_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/evaluate.py",
                    "--seeds",
                    "0",
                    "--policies",
                    "nearest_frontier",
                    "--max-steps",
                    "2",
                    "--output-dir",
                    tmp,
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=30,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((Path(tmp) / "eval_steps.csv").exists())
            self.assertTrue((Path(tmp) / "eval_summary.csv").exists())
            self.assertTrue((Path(tmp) / "coverage_curve.png").exists())
            self.assertTrue((Path(tmp) / "reward_curve.png").exists())

    def test_plot_training_log_script_runs_from_project_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            train_log = tmp_path / "train_log.csv"
            output = tmp_path / "training_curves.png"
            train_log.write_text(
                "\n".join(
                    [
                        "iteration,loss,actor_loss,value_loss,entropy,coverage,return_mean,return_std,value_pred_mean,approx_kl",
                        "0,1.0,0.1,0.8,1.2,0.15,-1.0,0.5,0.0,0.001",
                        "1,2.0,0.2,1.5,1.0,0.20,-0.6,0.8,0.1,0.002",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/plot_training_log.py",
                    str(train_log),
                    "--output",
                    str(output),
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=30,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 0)

    def test_visualize_episode_script_writes_nonempty_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "episode.gif"
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/visualize_episode.py",
                    "--policy",
                    "nearest_frontier",
                    "--map-type",
                    "office",
                    "--map-size",
                    "4",
                    "--resolution",
                    "0.5",
                    "--sensor-range",
                    "1.0",
                    "--sensor-rays",
                    "12",
                    "--max-steps",
                    "5",
                    "--seed",
                    "0",
                    "--output",
                    str(output),
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=60,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 0)

    def test_visualize_episode_separates_episode_steps_and_frame_stride(self):
        try:
            from PIL import Image
        except Exception:
            self.skipTest("Pillow is required to inspect GIF frames")

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "episode.gif"
            result = subprocess.run(
                [
                    sys.executable,
                    "scripts/visualize_episode.py",
                    "--policy",
                    "nearest_frontier",
                    "--map-type",
                    "open_room",
                    "--map-size",
                    "4",
                    "--resolution",
                    "0.5",
                    "--sensor-range",
                    "1.0",
                    "--sensor-rays",
                    "12",
                    "--episode-steps",
                    "20",
                    "--max-steps",
                    "6",
                    "--frame-stride",
                    "2",
                    "--seed",
                    "0",
                    "--output",
                    str(output),
                ],
                cwd=Path(__file__).resolve().parents[1],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True,
                text=True,
                timeout=60,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(output.exists())
            with Image.open(output) as image:
                self.assertEqual(image.n_frames, 4)


if __name__ == "__main__":
    unittest.main()
