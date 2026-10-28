"""Nonlinear double pendulum on a stepper-driven cart (Lagrangian model).

Conventions match Code/DoublePendulum/PythonMaster2_auto.py and
PythonMasterMath2.py: theta1 and theta2 are ABSOLUTE link angles measured from
upright (0 = up, +/-pi = hanging); a positive angle tips the link toward +x.
The second encoder measures the joint angle phi = theta2 - theta1.

The real cart is a closed-loop stepper that follows commanded step rates, so
the cart is modelled kinematically: the controller commands a cart velocity,
the motor tracks it with a short lag and an acceleration ceiling, and the
links are driven by the resulting cart acceleration a = x_ddot. With a
kinematic cart the cart mass drops out of the link equations.

Link equations (q = [theta1, theta2]):

    [J1          c cos(d)] [th1_dd]   [ -c sin(d) th2_d^2 + h1 g sin th1 - h1 cos th1 a + tau1 ]
    [c cos(d)    J2      ] [th2_dd] = [  c sin(d) th1_d^2 + h2 g sin th2 - h2 cos th2 a + tau2 ]

with d = th1 - th2, J1 = m1 l1^2 + m2 L1^2 + I1, J2 = m2 l2^2 + I2,
c = m2 L1 l2, h1 = m1 l1 + m2 L1, h2 = m2 l2 (l = centre-of-mass distance,
L1 = pivot-to-pivot length of link 1, I about the centre of mass).
Joint friction: tau_pivot = -b1 w1 - c1 sgn(w1) acts on link 1; the elbow
torque tau_j = -b2 wj - c2 sgn(wj), wj = th2_d - th1_d, acts +tau_j on link 2
and -tau_j on link 1. Coulomb terms use a smooth tanh with a small velocity
scale so fixed-step RK4 stays well behaved.
"""

from dataclasses import dataclass, field, replace, asdict
import json
import math
import sys
from pathlib import Path

import numpy as np

CODE_DIRECTORY = Path(__file__).resolve().parents[1]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

from Global.physical_parameters import (  # noqa: E402  (shared rig values)
    DOUBLE_ARM_1, DOUBLE_ARM_2, GRAVITY_M_S2, ENCODER_COUNTS,
    DISTANCE_PER_6400_STEPS_M,
)

FITTED_PARAMETERS_PATH = Path(__file__).with_name("fitted_parameters.json")
METRES_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
COULOMB_SMOOTHING_RAD_S = 0.02


@dataclass(frozen=True)
class TwinParameters:
    """Everything the twin needs. Defaults = nominal rig values + fitted extras."""
    # Links (nominal values come from Code/Global/physical_parameters.py).
    m1: float = DOUBLE_ARM_1.mass_kg
    l1: float = DOUBLE_ARM_1.center_of_mass_m
    L1: float = DOUBLE_ARM_1.length_m
    I1: float = DOUBLE_ARM_1.inertia_kg_m2
    m2: float = DOUBLE_ARM_2.mass_kg
    l2: float = DOUBLE_ARM_2.center_of_mass_m
    L2: float = DOUBLE_ARM_2.length_m
    I2: float = DOUBLE_ARM_2.inertia_kg_m2
    g: float = GRAVITY_M_S2
    # Joint friction (fitted; see fit_parameters.py).
    b1: float = 2.0e-4      # pivot viscous, N m s/rad
    c1: float = 5.0e-4      # pivot Coulomb, N m
    b2: float = 2.0e-4      # elbow viscous, N m s/rad
    c2: float = 1.0e-4      # elbow Coulomb, N m
    # Cart / stepper.
    motor_tau_s: float = 0.010          # 1/bandwidth of the closed-loop stepper position servo
    motor_accel_max: float = 20.0       # m/s^2 the motor can actually deliver
    motor_speed_max: float = 1.2        # m/s
    rail_limit_m: float = 0.45          # limit switches (|x| from centre)
    # Sensing / timing.
    encoder_counts: int = ENCODER_COUNTS
    encoder_noise_counts: float = 0.4
    sensor_delay_s: float = 0.006       # encoder sample -> Pi (radio + receiver slot + USB)
    actuator_delay_s: float = 0.002     # Pi write -> Arduino timer update
    packet_loss: float = 0.002          # probability a control cycle gets no valid frame
    control_period_s: float = 0.010     # Pi loop (receiver frames arrive at 100 Hz)
    control_jitter_s: float = 0.0008
    # Effective inertial constants identified from data (None = use m, l, I above).
    J1_eff: object = None
    J2_eff: object = None
    c_eff: object = None
    h1_eff: object = None
    h2_eff: object = None

    # Derived inertial constants -------------------------------------------
    @property
    def J1(self):
        if self.J1_eff is not None:
            return self.J1_eff
        return self.m1 * self.l1 ** 2 + self.m2 * self.L1 ** 2 + self.I1

    @property
    def J2(self):
        if self.J2_eff is not None:
            return self.J2_eff
        return self.m2 * self.l2 ** 2 + self.I2

    @property
    def c(self):
        if self.c_eff is not None:
            return self.c_eff
        return self.m2 * self.L1 * self.l2

    @property
    def h1(self):
        if self.h1_eff is not None:
            return self.h1_eff
        return self.m1 * self.l1 + self.m2 * self.L1

    @property
    def h2(self):
        if self.h2_eff is not None:
            return self.h2_eff
        return self.m2 * self.l2

    def inertial(self):
        """The five constants the dynamics depend on (J1, J2, c, h1, h2)."""
        return dict(J1=self.J1, J2=self.J2, c=self.c, h1=self.h1, h2=self.h2)

    def with_updates(self, **changes):
        return replace(self, **changes)

    def to_dict(self):
        return asdict(self)


def load_parameters(path=FITTED_PARAMETERS_PATH, **overrides):
    """Nominal parameters updated by fitted_parameters.json (if present)."""
    values = {}
    path = Path(path)
    if path.exists():
        data = json.loads(path.read_text())
        fields = TwinParameters.__dataclass_fields__
        values = {k: v["value"] if isinstance(v, dict) else v
                  for k, v in data.get("parameters", {}).items() if k in fields}
    values.update(overrides)
    return TwinParameters(**values)


def _smooth_sign(w):
    return math.tanh(w / COULOMB_SMOOTHING_RAD_S)


def joint_torques(p, w1, w2):
    wj = w2 - w1
    tau_pivot = -p.b1 * w1 - p.c1 * _smooth_sign(w1)
    tau_joint = -p.b2 * wj - p.c2 * _smooth_sign(wj)
    return tau_pivot - tau_joint, tau_joint


def link_accelerations(p, th1, w1, th2, w2, a):
    """Angular accelerations of both links for cart acceleration a."""
    d = th1 - th2
    cd, sd = math.cos(d), math.sin(d)
    tau1, tau2 = joint_torques(p, w1, w2)
    r1 = -p.c * sd * w2 * w2 + p.h1 * p.g * math.sin(th1) - p.h1 * math.cos(th1) * a + tau1
    r2 = p.c * sd * w1 * w1 + p.h2 * p.g * math.sin(th2) - p.h2 * math.cos(th2) * a + tau2
    m11, m12, m22 = p.J1, p.c * cd, p.J2
    det = m11 * m22 - m12 * m12
    return (m22 * r1 - m12 * r2) / det, (m11 * r2 - m12 * r1) / det


def motor_acceleration(p, x, v, x_cmd, v_cmd):
    """Closed-loop stepper as a critically damped position servo on the pulse train.

    The UIM5756PM counts the step pulses and corrects its own position, so the
    cart follows the commanded pulse position x_cmd (the same number the Arduino
    reports) with bandwidth 1/motor_tau_s and an acceleration ceiling.
    """
    wn = 1.0 / p.motor_tau_s
    a = wn * wn * (x_cmd - x) + 2.0 * wn * (v_cmd - v)
    return max(-p.motor_accel_max, min(p.motor_accel_max, a))


def derivatives(p, state, x_cmd, v_cmd):
    """state = [th1, w1, th2, w2, x, v]; the cart servo follows (x_cmd, v_cmd)."""
    th1, w1, th2, w2, x, v = state
    a = motor_acceleration(p, x, v, x_cmd, v_cmd)
    al1, al2 = link_accelerations(p, th1, w1, th2, w2, a)
    return np.array([w1, al1, w2, al2, v, a])


def rk4_step(p, state, x_cmd, v_cmd, dt):
    """One RK4 step; the pulse position advances at v_cmd during the step."""
    k1 = derivatives(p, state, x_cmd, v_cmd)
    k2 = derivatives(p, state + 0.5 * dt * k1, x_cmd + 0.5 * dt * v_cmd, v_cmd)
    k3 = derivatives(p, state + 0.5 * dt * k2, x_cmd + 0.5 * dt * v_cmd, v_cmd)
    k4 = derivatives(p, state + dt * k3, x_cmd + dt * v_cmd, v_cmd)
    return state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def free_cart_step(p, state, decel, dt):
    """Motor disabled (stop command or limit switch): cart coasts to rest."""
    th1, w1, th2, w2, x, v = state
    a = -math.copysign(min(abs(v) / dt, decel), v) if v else 0.0

    def f(s):
        al1, al2 = link_accelerations(p, s[0], s[1], s[2], s[3], a)
        return np.array([s[1], al1, s[3], al2, s[5], a])
    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    return state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def accel_driven_derivatives(p, state, a):
    """Same links, but driven directly by a known cart acceleration (for fitting)."""
    th1, w1, th2, w2 = state[:4]
    al1, al2 = link_accelerations(p, th1, w1, th2, w2, a)
    return np.array([w1, al1, w2, al2])


def energy(p, th1, w1, th2, w2):
    """Link mechanical energy in the cart frame; 0 = both links hanging at rest.

    Upright at rest has energy 2 g (h1 + h2) (about 1.61 J nominally).
    """
    kinetic = 0.5 * p.J1 * w1 ** 2 + 0.5 * p.J2 * w2 ** 2 + p.c * math.cos(th1 - th2) * w1 * w2
    potential = p.g * (p.h1 * (math.cos(th1) + 1) + p.h2 * (math.cos(th2) + 1))
    return kinetic + potential


def upright_energy(p):
    return 2 * p.g * (p.h1 + p.h2)


def linearized_upright(p, include_friction=False):
    """Continuous (A, B) about upright for state [th1,w1,th2,w2,x,v], input = cart accel."""
    M = np.array([[p.J1, p.c], [p.c, p.J2]])
    Minv = np.linalg.inv(M)
    G = np.diag([p.h1 * p.g, p.h2 * p.g])
    H = np.array([p.h1, p.h2])
    D = np.zeros((2, 2))
    if include_friction:
        # viscous part only; torque = -D @ [w1, w2]
        D = np.array([[p.b1 + p.b2, -p.b2], [-p.b2, p.b2]])
    A = np.zeros((6, 6))
    B = np.zeros((6, 1))
    Kq = Minv @ G
    Kw = -Minv @ D
    A[0, 1] = 1
    A[2, 3] = 1
    A[4, 5] = 1
    for row, i in ((1, 0), (3, 1)):
        A[row, 0] = Kq[i, 0]
        A[row, 2] = Kq[i, 1]
        A[row, 1] = Kw[i, 0]
        A[row, 3] = Kw[i, 1]
    Bq = -Minv @ H
    B[1, 0] = Bq[0]
    B[3, 0] = Bq[1]
    B[5, 0] = 1.0
    return A, B


def hanging_mode_frequencies_hz(p):
    M = np.array([[p.J1, p.c], [p.c, p.J2]])
    G = np.diag([p.h1 * p.g, p.h2 * p.g])
    ev = np.sort(np.linalg.eigvals(np.linalg.solve(M, G)).real)
    return np.sqrt(ev) / (2 * np.pi)


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def link_points(p, x, th1, th2):
    """Pivot, elbow and tip positions (m) for drawing; y up."""
    pivot = np.array([x, 0.0])
    elbow = pivot + p.L1 * np.array([math.sin(th1), math.cos(th1)])
    tip = elbow + p.L2 * np.array([math.sin(th2), math.cos(th2)])
    return pivot, elbow, tip


def link_accelerations_np(p, th1, w1, th2, w2, a, b1=None, c1=None, b2=None, c2=None):
    """Vectorised link_accelerations (numpy arrays); friction may be overridden."""
    b1 = p.b1 if b1 is None else b1
    c1 = p.c1 if c1 is None else c1
    b2 = p.b2 if b2 is None else b2
    c2 = p.c2 if c2 is None else c2
    d = th1 - th2
    cd, sd = np.cos(d), np.sin(d)
    wj = w2 - w1
    tau_joint = -b2 * wj - c2 * np.tanh(wj / COULOMB_SMOOTHING_RAD_S)
    tau_pivot = -b1 * w1 - c1 * np.tanh(w1 / COULOMB_SMOOTHING_RAD_S)
    r1 = -p.c * sd * w2 * w2 + p.h1 * p.g * np.sin(th1) - p.h1 * np.cos(th1) * a + tau_pivot - tau_joint
    r2 = p.c * sd * w1 * w1 + p.h2 * p.g * np.sin(th2) - p.h2 * np.cos(th2) * a + tau_joint
    m11, m12, m22 = p.J1, p.c * cd, p.J2
    det = m11 * m22 - m12 * m12
    return (m22 * r1 - m12 * r2) / det, (m11 * r2 - m12 * r1) / det
