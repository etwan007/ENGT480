"""Hardware-free checks of the digital-twin physics and actuator mapping."""

# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

TWIN_DIRECTORY = Path(__file__).resolve().parents[1]
CODE_DIRECTORY = TWIN_DIRECTORY.parent
for path in (CODE_DIRECTORY, TWIN_DIRECTORY, CODE_DIRECTORY / "DoublePendulum",
             CODE_DIRECTORY / "DoublePendulum" / "Improved"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import math
import types
import unittest

import numpy as np

try:
    import serial  # noqa: F401
except ModuleNotFoundError:
    sys.modules["serial"] = types.ModuleType("serial")

from twin_model import (TwinParameters, load_parameters, link_accelerations, link_accelerations_np,
                        linearized_upright, energy, upright_energy, hanging_mode_frequencies_hz,
                        accel_driven_derivatives)
import plan_model
import stepper_mapping
import PythonMaster2_auto


def rk4_links(p, s, a, dt, n):
    for _ in range(n):
        k1 = accel_driven_derivatives(p, s, a)
        k2 = accel_driven_derivatives(p, s + 0.5 * dt * k1, a)
        k3 = accel_driven_derivatives(p, s + 0.5 * dt * k2, a)
        k4 = accel_driven_derivatives(p, s + dt * k3, a)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return s


class TwinModelTests(unittest.TestCase):
    def test_energy_is_conserved_without_friction(self):
        p = TwinParameters(b1=0, c1=0, b2=0, c2=0)
        s = np.array([2.0, 0.0, 2.6, 0.0])
        e0 = energy(p, *s)
        s = rk4_links(p, s, 0.0, 0.0005, 4000)
        self.assertAlmostEqual(energy(p, *s), e0, places=5)

    def test_friction_only_removes_energy(self):
        p = TwinParameters(b1=5e-4, c1=3e-4, b2=1e-4, c2=1e-4)
        s = np.array([2.0, 0.0, 2.6, 0.0])
        e0 = energy(p, *s)
        s = rk4_links(p, s, 0.0, 0.001, 2000)
        self.assertLess(energy(p, *s), e0)

    def test_energy_reference_points(self):
        p = TwinParameters()
        self.assertAlmostEqual(energy(p, math.pi, 0, math.pi, 0), 0.0, places=12)
        self.assertAlmostEqual(energy(p, 0, 0, 0, 0), upright_energy(p), places=12)
        self.assertAlmostEqual(upright_energy(p), 2 * 9.81 * (p.h1 + p.h2), places=12)

    def test_linearization_matches_nonlinear_model(self):
        p = TwinParameters(c1=0, c2=0)
        A, B = linearized_upright(p, include_friction=True)
        eps = 1e-6
        base = np.array(link_accelerations(p, 0, 0, 0, 0, 0))
        for col, args in ((0, (eps, 0, 0, 0, 0)), (1, (0, eps, 0, 0, 0)), (2, (0, 0, eps, 0, 0)),
                          (3, (0, 0, 0, eps, 0))):
            d = (np.array(link_accelerations(p, *args)) - base) / eps
            np.testing.assert_allclose(d, A[[1, 3], col], rtol=1e-4, atol=1e-6)
        d = (np.array(link_accelerations(p, 0, 0, 0, 0, eps)) - base) / eps
        np.testing.assert_allclose(d, B[[1, 3], 0], rtol=1e-4)

    def test_vectorised_and_scalar_accelerations_agree(self):
        p = TwinParameters()
        rng = np.random.default_rng(3)
        for row in rng.normal(size=(20, 5)):
            a = link_accelerations(p, *row)
            b = link_accelerations_np(p, *[np.array([v]) for v in row])
            np.testing.assert_allclose(a, [b[0][0], b[1][0]], rtol=1e-12)

    def test_nominal_hanging_modes_match_the_sysid_logs(self):
        f1, f2 = hanging_mode_frequencies_hz(TwinParameters())
        # Team sysid logs: 0.86-0.90 Hz and 1.79-1.90 Hz.
        self.assertTrue(0.85 < f1 < 0.92)
        self.assertTrue(1.75 < f2 < 1.95)

    def test_fitted_parameters_load_and_stay_physical(self):
        p = load_parameters()
        nominal = TwinParameters()
        for name in ("J1", "J2", "c", "h1", "h2"):
            self.assertLess(abs(getattr(p, name) / getattr(nominal, name) - 1), 0.25, name)
        for name in ("b1", "c1", "b2", "c2"):
            self.assertGreaterEqual(getattr(p, name), 0.0)
        f1, f2 = hanging_mode_frequencies_hz(p)
        self.assertTrue(0.8 < f1 < 0.95 and 1.7 < f2 < 1.95)

    def test_plan_model_velocity_step_matches_accel_impulse(self):
        p = TwinParameters(c1=0, c2=0)
        k = plan_model.constants_from(p)
        s = (0.3, 0.5, -0.2, -0.4, 0.0)
        # coasting at constant velocity: same as the twin with zero cart accel
        out = plan_model.cycle(k, s, 0.2, 0.2, 0.01, 0.004)
        ref = rk4_links(p, np.array(s[:4]), 0.0, 0.0005, 20)
        np.testing.assert_allclose(out[:4], ref, atol=1e-6)
        self.assertAlmostEqual(out[4], 0.002, places=12)
        # a velocity step dv changes the rates by -b dv
        b1, b2 = plan_model.coupling(k, s[0], s[2])
        out0 = plan_model.cycle(k, s, 0.0, 0.0, 1e-9, 0.0)
        out1 = plan_model.cycle(k, s, 0.0, 0.1, 1e-9, 0.0)
        self.assertAlmostEqual(out1[1] - out0[1], -b1 * 0.1, places=6)
        self.assertAlmostEqual(out1[3] - out0[3], -b2 * 0.1, places=6)


class StepperMappingTests(unittest.TestCase):
    def test_same_timer_command_as_the_existing_controller(self):
        for v in (-2.0, -0.7, -0.05, -0.0001, 0.0, 0.0003, 0.01, 0.3, 1.1):
            self.assertEqual(stepper_mapping.timer_command_for_velocity(v),
                             PythonMaster2_auto.timer_command_for_velocity(v))

    def test_velocity_round_trip_and_direction(self):
        for v in (-0.9, -0.2, 0.05, 0.4, 0.8):
            _, cmd = stepper_mapping.timer_command_for_velocity(v)
            back = stepper_mapping.velocity_for_timer_command(cmd)
            self.assertEqual(math.copysign(1, back), math.copysign(1, v))
            # int() truncation of the timer value: at most one timer tick of error
            tick = abs(stepper_mapping.velocity_for_timer_command(cmd + (1 if cmd > 0 else -1)) - back)
            self.assertLessEqual(abs(back - v), tick + 1e-12)

    def test_stopped_command_is_almost_zero_speed(self):
        self.assertLess(abs(stepper_mapping.velocity_for_timer_command(65535)), 2.5e-4)


if __name__ == "__main__":
    unittest.main()
