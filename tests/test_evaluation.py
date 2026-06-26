import csv
import tempfile
import unittest
from pathlib import Path

from eavf_mappo.evaluation.reporting import (
    plot_training_curves,
    plot_step_metrics,
    summarize_steps,
    write_csv,
)
from eavf_mappo.evaluation.runner import run_episode, run_evaluation


class EvaluationTests(unittest.TestCase):
    def test_run_episode_returns_step_rows_with_observable_metrics(self):
        rows = run_episode(
            policy_name="nearest_frontier",
            seed=0,
            max_steps=12,
            env_config={
                "env": {
                    "n_robots": 1,
                    "map_size_m": 6.0,
                    "resolution": 0.5,
                    "max_episode_steps": 12,
                    "sensor_range_m": 1.2,
                    "sensor_n_rays": 24,
                    "map_type": "open_room",
                }
            },
        )

        self.assertGreater(len(rows), 0)
        first = rows[0]
        for key in [
            "run_id",
            "seed",
            "policy",
            "step",
            "coverage",
            "mean_reward",
            "episode_return",
            "collision_count",
            "overlap_count",
            "path_length",
            "high_level_count",
            "terminated",
            "truncated",
        ]:
            self.assertIn(key, first)
        self.assertEqual(first["policy"], "nearest_frontier")
        self.assertGreaterEqual(first["coverage"], 0.0)
        self.assertLessEqual(first["coverage"], 1.0)
        self.assertGreater(rows[-1]["coverage"], rows[0]["coverage"])

    def test_run_evaluation_and_summary_group_by_policy_and_seed(self):
        rows = run_evaluation(
            policies=["nearest_frontier", "round_robin"],
            seeds=[0, 1],
            max_steps=4,
            env_config={
                "env": {
                    "n_robots": 2,
                    "map_size_m": 4.0,
                    "resolution": 0.5,
                    "max_episode_steps": 4,
                }
            },
        )

        summary = summarize_steps(rows, coverage_target=0.9)

        self.assertEqual(len(summary), 4)
        self.assertEqual({row["policy"] for row in summary}, {"nearest_frontier", "round_robin"})
        for row in summary:
            self.assertIn("final_coverage", row)
            self.assertIn("total_return", row)
            self.assertIn("time_to_90", row)
            self.assertIn("collision_total", row)
            self.assertIn("overlap_total", row)

    def test_csv_and_plots_are_written(self):
        rows = run_evaluation(
            policies=["nearest_frontier"],
            seeds=[0],
            max_steps=3,
            env_config={
                "env": {
                    "n_robots": 2,
                    "map_size_m": 4.0,
                    "resolution": 0.5,
                    "max_episode_steps": 3,
                }
            },
        )
        summary = summarize_steps(rows)

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            step_csv = out / "eval_steps.csv"
            summary_csv = out / "eval_summary.csv"
            coverage_png = out / "coverage_curve.png"
            reward_png = out / "reward_curve.png"

            write_csv(step_csv, rows)
            write_csv(summary_csv, summary)
            plot_step_metrics(step_csv, coverage_png, reward_png)

            self.assertTrue(step_csv.exists())
            self.assertTrue(summary_csv.exists())
            self.assertGreater(coverage_png.stat().st_size, 0)
            self.assertGreater(reward_png.stat().st_size, 0)

            with step_csv.open(newline="", encoding="utf-8") as fh:
                reader = csv.DictReader(fh)
                self.assertIn("coverage", reader.fieldnames)
                self.assertIn("episode_return", reader.fieldnames)

    def test_plot_training_curves_writes_nonempty_png(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            train_log = out / "train_log.csv"
            train_log.write_text(
                "\n".join(
                    [
                        "iteration,loss,actor_loss,value_loss,entropy,coverage,return_mean,return_std,value_pred_mean,approx_kl",
                        "0,1.0,0.1,0.8,1.2,0.15,-1.0,0.5,0.0,0.001",
                        "1,10.0,0.2,8.0,1.0,0.22,-0.5,0.7,0.1,0.002",
                        "2,1000.0,0.1,900.0,0.8,0.30,0.2,1.1,0.2,0.003",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            output = out / "training_curves.png"

            plot_training_curves(train_log, output)

            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
