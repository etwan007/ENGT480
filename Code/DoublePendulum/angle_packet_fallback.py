"""Estimator behavior when the ESP32 angle packet is missing."""

import numpy as np
import time


class MissingAnglePacketError(RuntimeError):
    """No usable first-pendulum angle arrived for this control cycle."""


def collect_valid_samples(read_sample, count, *, interval_s=0.02,
                          max_total_misses=5, max_consecutive_misses=1):
    """Collect setup readings; stop when either allowed miss count is exceeded."""
    for name, limit in (("max_total_misses", max_total_misses),
                        ("max_consecutive_misses", max_consecutive_misses)):
        if isinstance(limit, bool) or not isinstance(limit, int) or limit < 0:
            raise ValueError(f"{name} must be a nonnegative integer")
    samples = []
    misses = 0
    consecutive_misses = 0
    while len(samples) < count:
        try:
            sample = read_sample()
        except MissingAnglePacketError as exc:
            misses += 1
            consecutive_misses += 1
            if consecutive_misses > max_consecutive_misses:
                raise RuntimeError(
                    f"{consecutive_misses} consecutive ESP32 angle packets were unusable "
                    f"during setup (allowed {max_consecutive_misses})") from exc
            if misses > max_total_misses:
                raise RuntimeError(f"Too many unusable ESP32 angle packets during setup ({misses})") from exc
        else:
            samples.append(sample)
            consecutive_misses = 0
        if len(samples) < count:
            time.sleep(interval_s)
    return samples, misses


def estimate_with_angle(predicted, measured_angle, cart_m, observation_matrix,
                        estimator_gain, consecutive_misses):
    """Return the next state, angle to report, and consecutive miss count."""
    if measured_angle is None:
        consecutive_misses += 1
        if consecutive_misses >= 2:
            raise RuntimeError("Two consecutive ESP32 angle packets were unusable; stopping motor")
        measured_angle = float(predicted[0, 0])
    else:
        consecutive_misses = 0

    measured = np.array([[measured_angle], [cart_m]])
    estimated = predicted + estimator_gain @ (measured - observation_matrix @ predicted)
    return estimated, float(measured_angle), consecutive_misses
