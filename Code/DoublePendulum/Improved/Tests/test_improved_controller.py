"""Hardware-free checks of the improved controller (logic, limits, twin runs)."""

# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

IMPROVED_DIRECTORY = Path(__file__).resolve().parents[1]
CODE_DIRECTORY = IMPROVED_DIRECTORY.parents[1]
for path in (CODE_DIRECTORY, IMPROVED_DIRECTORY, IMPROVED_DIRECTORY.parent, CODE_DIRECTORY / "DigitalTwin"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import io
import json
import math
import tempfile
import types
import unittest
from contextlib import redirect_stdout

import numpy as np

try:
    import serial  # noqa: F401
except ModuleNotFoundError:
    sys.modules["serial"] = types.ModuleType("serial")

from improved_controller import ImprovedController, ControllerConfig, GAINS_PATH, PLAN_PATH, wrap
from twin_model import load_parameters
from twin_sim import RigSimulator, SimConfig, hanging_state


class Obs:
    def __init__(self, t, theta1, theta2, cart_m=0.0, dt=0.01):
        self.t, self.dt, self.cycle = t, dt, int(t / 0.01)
        self.theta1, self.theta2, self.cart_m = theta1, theta2, cart_m
        self.raw1 = self.raw2 = None


def balance_controller(**kw):
    return ImprovedController(config=ControllerConfig(balance_only=True, **kw))


class SafetyTests(unittest.TestCase):
    def test_missing_frames_stop_after_the_allowed_count(self):
        c = balance_controller(max_missing_frames=3)
        out = c.update(Obs(0.0, 0.01, 0.01))
        self.assertFalse(out.stop)
        for k in range(3):
            self.assertFalse(c.update(Obs(0.01 * (k + 1), None, None)).stop)
        out = c.update(Obs(0.04, None, None))
        self.assertTrue(out.stop)
        self.assertIn("missing", out.reason)
        self.assertTrue(c.update(Obs(0.05, 0.0, 0.0)).stop)      # latched

    def test_cart_beyond_estop_limit_stops(self):
        c = balance_controller(estop_limit_m=0.3)
        out = c.update(Obs(0.0, 0.0, 0.0, cart_m=0.31))
        self.assertTrue(out.stop)
        self.assertIn("e-stop", out.reason)

    def test_stalled_loop_stops(self):
        c = balance_controller()
        c.update(Obs(0.0, 0.0, 0.0))
        out = c.update(Obs(0.2, 0.0, 0.0, dt=0.2))
        self.assertTrue(out.stop)
        self.assertIn("stalled", out.reason)

    def test_balance_only_stops_when_the_arms_fall(self):
        c = balance_controller()
        out = None
        modes = set()
        for k in range(200):
            out = c.update(Obs(0.01 * k, math.radians(50), math.radians(50)))
            modes.add(out.mode)
            if out.stop:
                break
        self.assertTrue(out.stop)
        self.assertIn("balance lost", out.reason)
        self.assertIn("STOPPING", modes)          # braked with the motor first
        self.assertAlmostEqual(c.v, 0.0, places=3)

    def test_stop_brakes_the_cart_before_disabling(self):
        c = balance_controller(stop_brake_accel=8.0)
        c.update(Obs(0.0, 0.0, 0.0))
        c.v = 0.8
        out = c._stop("test")
        self.assertFalse(out.stop)
        self.assertEqual(out.mode, "STOPPING")
        speeds = []
        while not out.stop:
            out = c.update(Obs(0.0, None, None))
            speeds.append(abs(out.velocity))
        self.assertTrue(all(b <= a + 1e-12 for a, b in zip(speeds, speeds[1:])))
        self.assertLessEqual(len(speeds), 12)      # 0.8 m/s at 8 m/s^2 -> ~0.1 s
        self.assertEqual(out.reason, "test")

    def test_governor_keeps_the_cart_inside_the_soft_limit(self):
        c = balance_controller(soft_limit_m=0.25, brake_accel=6.0, max_speed=1.0)
        c.v = 0.8
        x = 0.0
        for _ in range(200):
            a = c._governor(20.0, x, 0.01, 20.0)       # controller keeps pushing +x
            c.v = c.v + a * 0.01
            x += c.v * 0.01
        self.assertLessEqual(x, 0.25 + 1e-3)
        self.assertLessEqual(abs(c.v), 1.0 + 1e-9)

    def test_speed_and_accel_limits(self):
        c = balance_controller(max_speed=0.5, max_accel_balance=5.0, soft_limit_m=10)
        out = c._command(1000.0, 0.0, 0.01, 5.0, 0.0)
        self.assertAlmostEqual(out.velocity, 0.05)
        for _ in range(50):
            out = c._command(1000.0, 0.0, 0.01, 5.0, 0.0)
        self.assertAlmostEqual(out.velocity, 0.5)


class EnergyTests(unittest.TestCase):
    def test_energy_references(self):
        c = ImprovedController(config=ControllerConfig(swing_mode="energy"))
        self.assertAlmostEqual(c.energy(math.pi, 0, math.pi, 0), 0.0, places=12)
        self.assertAlmostEqual(c.energy(0, 0, 0, 0), c.upright_energy, places=12)

    def test_energy_law_adds_energy_below_target(self):
        # dE/dt = -a*y for the cart-frame energy; the pump must make -a*y > 0
        c = ImprovedController(config=ControllerConfig(swing_mode="energy", sync_gain=0.0,
                                                       swing_center_kp=0, swing_center_kd=0, kick_s=0))
        c.kick_left = 0.0
        for th1, w1 in ((2.5, 1.0), (2.5, -1.0), (-2.0, 2.0), (3.0, -0.5)):
            E = c.energy(th1, w1, th1, w1)
            a = c._swing(th1, w1, th1, w1, 0.0, E, 0.01)
            y = c.cfg.h1 * math.cos(th1) * w1 + c.cfg.h2 * math.cos(th1) * w1
            self.assertGreater(-a * y, 0)


class DesignTests(unittest.TestCase):
    def test_gain_profiles_are_stable_and_match_the_model(self):
        data = json.loads(GAINS_PATH.read_text())
        for name in ("conservative", "nominal", "catch"):
            g = data["profiles"][name]
            self.assertLess(g["closed_loop_poles_abs"][0], 1.0, name)
            self.assertEqual(len(g["K"][0]), 7)
        # the stored gains are reproducible from the fitted model
        import design_lqr
        fresh = design_lqr.design(load_parameters(), profile="conservative")
        np.testing.assert_allclose(fresh["K"], data["profiles"]["conservative"]["K"], rtol=1e-3, atol=1e-4)


class TwinRunTests(unittest.TestCase):
    def test_balances_from_a_small_tilt_in_the_twin(self):
        c = balance_controller()
        r = RigSimulator(load_parameters(), SimConfig(duration_s=10, seed=4)).run(
            c, [math.radians(4), 0.0, math.radians(-2), 0.0, 0.0, 0.0])
        self.assertEqual(r["stop_reason"], "")
        self.assertLess(np.max(np.abs(np.degrees(wrap(r["th1"][-300])))), 3)
        self.assertLess(np.max(np.abs(r["x"])), 0.2)

    @unittest.skipUnless(PLAN_PATH.exists(), "swingup_plan.json not generated")
    def test_swings_up_and_balances_in_the_twin(self):
        c = ImprovedController()
        r = RigSimulator(load_parameters(), SimConfig(duration_s=25, seed=1)).run(c, hanging_state())
        self.assertFalse(r["limit_hit"])
        self.assertIn("BALANCE", set(r["mode"]))
        tail = r["t"] > r["t"][-1] - 3
        self.assertTrue(np.all(np.abs(wrap(r["th1"][tail])) < 0.2))
        self.assertTrue(np.all(np.abs(wrap(r["th2"][tail])) < 0.2))

    def test_hardware_script_runs_in_sim_mode(self):
        import PythonMaster2_improved as entry
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            entry.main(["--sim", "--balance-only", "--duration", "2", "--out", tmp,
                        "--name", "t", "--no-animation"])
            self.assertTrue((Path(tmp) / "t.csv").exists())
            self.assertTrue((Path(tmp) / "t.png").exists())


if __name__ == "__main__":
    unittest.main()
