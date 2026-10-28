"""Hardware-free checks of the rig simulator (actuator, limits, sensing, CSV)."""

# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

TWIN_DIRECTORY = Path(__file__).resolve().parents[1]
CODE_DIRECTORY = TWIN_DIRECTORY.parent
for path in (CODE_DIRECTORY, TWIN_DIRECTORY, CODE_DIRECTORY / "DoublePendulum" / "Improved"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import csv
import math
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from twin_model import TwinParameters
from twin_sim import RigSimulator, SimConfig, ControlOutput, hanging_state, write_run_csv
from Global.run_csv_logger import DOUBLE_COLUMNS


class Constant:
    def __init__(self, v, stop_after=None):
        self.v, self.stop_after, self.seen = v, stop_after, []

    def update(self, obs):
        self.seen.append(obs)
        if self.stop_after is not None and obs.t >= self.stop_after:
            return ControlOutput(0.0, True, "STOPPED", 0.0, "test stop")
        return ControlOutput(self.v, False, "TEST")


class RigSimulatorTests(unittest.TestCase):
    def test_hanging_and_still_stays_put(self):
        r = RigSimulator(TwinParameters(packet_loss=0), SimConfig(duration_s=2)).run(Constant(0.0), hanging_state())
        self.assertLess(np.max(np.abs(r["x"])), 1e-3)
        self.assertLess(np.max(np.abs(np.cos(r["th1"]) + 1)), 1e-6)

    def test_cart_follows_the_quantised_velocity_command(self):
        r = RigSimulator(TwinParameters(packet_loss=0), SimConfig(duration_s=1)).run(Constant(0.2), hanging_state())
        self.assertAlmostEqual(r["v"][-1], r["v_q"][-1], places=3)
        self.assertLess(abs(r["v_q"][-1] - 0.2), 0.01)
        self.assertAlmostEqual(r["x"][-1], 0.2 * r["t"][-1], delta=0.01)

    def test_limit_switch_disables_the_motor(self):
        r = RigSimulator(TwinParameters(packet_loss=0), SimConfig(duration_s=3)).run(Constant(0.5), hanging_state())
        self.assertTrue(r["limit_hit"])
        self.assertIn("limit switch", r["stop_reason"])
        self.assertLess(abs(r["v"][-1]), 0.05)        # coasted to rest after the switch
        self.assertLess(np.max(r["x"]), 0.45 + 0.05)

    def test_controller_stop_disables_the_motor(self):
        c = Constant(0.3, stop_after=0.3)
        r = RigSimulator(TwinParameters(packet_loss=0), SimConfig(duration_s=3)).run(c, hanging_state())
        self.assertTrue(r["stopped_by_controller"])
        self.assertEqual(r["stop_reason"], "test stop")
        self.assertLess(r["t"][-1], 3.0)

    def test_encoder_readings_are_14_bit_and_delayed(self):
        p = TwinParameters(packet_loss=0, encoder_noise_counts=0, sensor_delay_s=0.006)
        c = Constant(0.0)
        state = np.array([0.5, 2.0, 0.4, 2.0, 0.0, 0.0])
        RigSimulator(p, SimConfig(duration_s=0.2)).run(c, state)
        step = 2 * math.pi / 16384
        for obs in c.seen[1:]:
            self.assertAlmostEqual(obs.theta1 / step, round(obs.theta1 / step), places=6)
        # the rate is positive, so delayed readings lag the truth
        r = RigSimulator(p, SimConfig(duration_s=0.2)).run(Constant(0.0), state)
        self.assertTrue(np.all(r["th1_meas"][2:] < r["th1"][2:]))

    def test_lost_frames_reach_the_controller_as_none(self):
        c = Constant(0.0)
        RigSimulator(TwinParameters(packet_loss=0.5), SimConfig(duration_s=1, seed=2)).run(c, hanging_state())
        self.assertTrue(any(o.theta1 is None for o in c.seen))
        self.assertTrue(any(o.theta1 is not None for o in c.seen))

    def test_run_csv_uses_the_double_logger_columns(self):
        r = RigSimulator(TwinParameters(), SimConfig(duration_s=0.5)).run(Constant(0.1), hanging_state())
        with tempfile.TemporaryDirectory() as tmp:
            path = write_run_csv(r, Path(tmp) / "run.csv")
            with path.open(newline="") as f:
                rows = list(csv.reader(f))
        self.assertEqual(tuple(rows[0]), DOUBLE_COLUMNS)
        samples = [row for row in rows if row[0] == "sample"]
        self.assertEqual(len(samples), len(r["t"]))
        self.assertEqual(rows[-1][0], "stop")
        self.assertIn("SIMULATED", rows[-1][-1])


class VisualizerTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "Node.js not installed")
    def test_simulated_csv_opens_in_the_run_visualizer(self):
        r = RigSimulator(TwinParameters(), SimConfig(duration_s=1.0)).run(Constant(0.1), hanging_state())
        with tempfile.TemporaryDirectory() as tmp:
            path = write_run_csv(r, Path(tmp) / "SIMULATED_run.csv")
            out = subprocess.run(["node", str(Path(__file__).with_name("check_visualizer_load.js")), str(path)],
                                 capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr + out.stdout)
        self.assertIn("visualizer loaded", out.stdout)


class LegacyControllerInTwinTests(unittest.TestCase):
    def test_existing_controller_balances_in_the_twin(self):
        from legacy_controller import LegacyController
        r = RigSimulator(TwinParameters(packet_loss=0), SimConfig(duration_s=8, seed=1)).run(
            LegacyController(), [math.radians(2), 0, math.radians(1), 0, 0, 0])
        self.assertEqual(r["stop_reason"], "")
        self.assertLess(np.max(np.abs(np.degrees(r["th1"][-200:]))), 5)


if __name__ == "__main__":
    unittest.main()
