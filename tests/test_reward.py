import unittest

import numpy as np

from eavf_mappo.utils.metrics import retention_ratio


class RewardMetricTests(unittest.TestCase):
    def test_retention_ratio_is_clipped_to_unit_interval(self):
        self.assertAlmostEqual(retention_ratio(0.5, 1.0), 0.5)
        self.assertAlmostEqual(retention_ratio(2.0, 1.0), 1.0)

    def test_retention_ratio_accepts_arrays(self):
        actual = retention_ratio(np.array([0.5, 0.25]), np.array([1.0, 0.5]))

        np.testing.assert_allclose(actual, np.array([0.5, 0.5]))


if __name__ == "__main__":
    unittest.main()
