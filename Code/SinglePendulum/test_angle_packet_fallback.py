# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

CODE_DIRECTORY = Path(__file__).resolve().parents[1]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

import unittest

import numpy as np

from Global.angle_packet_fallback import (MissingAnglePacketError, collect_valid_samples,
                                   estimate_with_angle)


class AnglePacketFallbackTests(unittest.TestCase):
    def setUp(self):
        self.predicted = np.array([[0.12], [0.4], [0.03], [-0.1]])
        self.observation = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.gain = np.array([[0.5, 0], [0, 0], [0, 0.5], [0, 0]])

    def estimate(self, angle, misses):
        return estimate_with_angle(self.predicted, angle, 0.05,
                                   self.observation, self.gain, misses)

    def test_one_missing_packet_uses_predicted_angle_and_cart_correction(self):
        estimated, angle, misses = self.estimate(None, 0)
        self.assertAlmostEqual(estimated[0, 0], 0.12)
        self.assertAlmostEqual(estimated[2, 0], 0.04)
        self.assertEqual(angle, 0.12)
        self.assertEqual(misses, 1)

    def test_second_consecutive_missing_packet_stops(self):
        with self.assertRaisesRegex(RuntimeError, "Two consecutive ESP32 angle packets were unusable"):
            self.estimate(None, 1)

    def test_valid_packet_corrects_estimate_and_resets_miss_count(self):
        estimated, angle, misses = self.estimate(0.2, 1)
        self.assertAlmostEqual(estimated[0, 0], 0.16)
        self.assertAlmostEqual(estimated[2, 0], 0.04)
        self.assertEqual(angle, 0.2)
        self.assertEqual(misses, 0)
        _, _, misses = self.estimate(None, misses)
        self.assertEqual(misses, 1)

    def test_setup_skips_one_bad_packet_and_collects_valid_samples(self):
        packets = iter([MissingAnglePacketError("invalid"), 100, 101])

        def read_sample():
            packet = next(packets)
            if isinstance(packet, Exception):
                raise packet
            return packet

        samples, misses = collect_valid_samples(read_sample, 2, interval_s=0)
        self.assertEqual(samples, [100, 101])
        self.assertEqual(misses, 1)

    def test_setup_stops_on_two_bad_packets_in_a_row(self):
        def read_sample():
            raise MissingAnglePacketError("invalid")

        with self.assertRaisesRegex(RuntimeError, "2 consecutive ESP32 angle packets were unusable"):
            collect_valid_samples(read_sample, 2, interval_s=0)

    def test_setup_configurable_consecutive_limit_and_reset(self):
        packets = iter([None, None, 100, None, None, 101])

        def read_sample():
            value = next(packets)
            if value is None:
                raise MissingAnglePacketError("missing")
            return value

        samples, misses = collect_valid_samples(
            read_sample, 2, interval_s=0, max_consecutive_misses=2)
        self.assertEqual(samples, [100, 101])
        self.assertEqual(misses, 4)

    def test_setup_stops_when_configured_limit_is_exceeded(self):
        for allowed in (0, 3):
            calls = 0

            def read_sample():
                nonlocal calls
                calls += 1
                raise MissingAnglePacketError("missing")

            with self.subTest(allowed=allowed):
                with self.assertRaisesRegex(RuntimeError, f"allowed {allowed}"):
                    collect_valid_samples(read_sample, 1, interval_s=0,
                                          max_consecutive_misses=allowed)
                self.assertEqual(calls, allowed + 1)

    def test_setup_rejects_invalid_miss_limits(self):
        for name in ("max_total_misses", "max_consecutive_misses"):
            for value in (-1, 1.5, True):
                with self.subTest(name=name, value=value):
                    with self.assertRaisesRegex(ValueError, name):
                        collect_valid_samples(lambda: 100, 1, **{name: value})

    def test_setup_stops_when_interspersed_bad_packets_are_too_frequent(self):
        packets = iter([None, 100, None, 101, None, 102])

        def read_sample():
            packet = next(packets)
            if packet is None:
                raise MissingAnglePacketError("invalid")
            return packet

        with self.assertRaisesRegex(RuntimeError, "Too many unusable ESP32"):
            collect_valid_samples(read_sample, 4, interval_s=0, max_total_misses=2)


if __name__ == "__main__":
    unittest.main()
