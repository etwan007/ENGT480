"""The team's current double-pendulum controller, ported to the twin interface.

Mirrors the control loop of Code/DoublePendulum/PythonMaster2_auto.py (the
script PythonMasterControl2.py launches): discrete model + Kalman predictor
from control_matrices2.txt, force = clip(-1.5 Kd x, +/-5), cart VELOCITY
command = the estimator's cart-velocity state, 20-cycle start ramp, +/-5 m/s
clip, the slow "upright1 += cart/60 + v/30" encoder-reference drift, one
allowed missing frame, and the 1.0 rad angle stop. Balance only (it has no
swing-up). The matrices and helper functions are imported from the real
script so a change there is picked up here.
"""

import math
import sys
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DOUBLE_DIR = HERE.parent / "DoublePendulum"
for path in (HERE.parent, DOUBLE_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import serial  # noqa: F401  (present on the Pi)
except ImportError:  # the twin does not need pyserial; give the import a stub
    sys.modules["serial"] = types.ModuleType("serial")

import PythonMaster2_auto as legacy  # noqa: E402

from twin_sim import ControlOutput  # noqa: E402

ENC_SCALE = 2 * math.pi / legacy.ENCODER_COUNTS


class LegacyController:
    name = "PythonMaster2_auto (existing)"

    def __init__(self, matrices_path=DOUBLE_DIR / "control_matrices2.txt"):
        self.m = legacy.load_control_data(matrices_path)
        self.reset()

    def reset(self):
        self.state = None
        self.force = np.zeros((1, 1))
        self.cycle = 0
        self.misses = 0
        self.offset = 0.0          # radians added by the upright1 drift
        self.mode = "BALANCE"
        self.stop_reason = ""

    def update(self, obs):
        m = self.m
        if self.mode == "STOPPED":
            return ControlOutput(0.0, True, "STOPPED", 0.0, self.stop_reason)
        if self.state is None:
            th1 = (obs.theta1 or 0.0) + self.offset
            th2 = (obs.theta2 or 0.0) + self.offset
            self.state = np.array([[th1], [0.0], [th2], [0.0], [obs.cart_m], [0.0]])
        self.cycle += 1
        predicted = m["A"] @ self.state + m["B"] @ self.force
        if obs.theta1 is None or obs.theta2 is None:
            self.misses += 1
            if self.misses > legacy.MAX_RUNNING_CONSECUTIVE_MISSES:
                return self._stop(f"{self.misses} consecutive ESP32 angle packets were unusable")
            th1 = float(predicted[0, 0])
            th2 = float(predicted[2, 0])
        else:
            self.misses = 0
            # theta1 uses the drifting reference; theta2 = theta1 + joint
            th1 = obs.theta1 + self.offset
            th2 = obs.theta2 + self.offset
        if abs(th1) > legacy.MAX_RUNNING_ANGLE_RAD or abs(th2) > legacy.MAX_RUNNING_ANGLE_RAD:
            return self._stop(f"Angle limit exceeded: arm1={math.degrees(th1):+.1f}deg, "
                              f"arm2={math.degrees(th2):+.1f}deg")
        measured = np.array([[th1], [th2], [obs.cart_m]])
        self.state = predicted + m["Ld"] @ (measured - m["C"] @ predicted)
        self.force = np.clip(-m["Kd"] @ self.state * 1.5, -5, 5)
        v = float(self.state[5, 0])
        if self.cycle < 20:
            v *= self.cycle / 20.0
        v = float(np.clip(v, -5, 5))
        # upright1 += cart_m / 60 + cart_velocity / 30  (counts; uses the NEW reply,
        # approximated here by this cycle's position)
        self.offset += (obs.cart_m / 60 + v / 30) * ENC_SCALE
        return ControlOutput(v, False, "BALANCE", float(self.force[0, 0]))

    def _stop(self, reason):
        self.mode = "STOPPED"
        self.stop_reason = reason
        return ControlOutput(0.0, True, "STOPPED", 0.0, reason)
