from __future__ import annotations

import numpy as np


def retention_ratio(perf_degraded, perf_full):
    ratio = np.asarray(perf_degraded, dtype=np.float64) / (
        np.asarray(perf_full, dtype=np.float64) + 1.0e-8
    )
    clipped = np.clip(ratio, 0.0, 1.0)
    if clipped.ndim == 0:
        return float(clipped)
    return clipped
