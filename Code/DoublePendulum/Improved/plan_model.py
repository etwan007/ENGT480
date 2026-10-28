"""Discrete model of one control cycle as the Pi actually actuates it.

The Arduino holds a step RATE, so the controller's command is a cart velocity
that changes in steps. A velocity step dv applied to the cart gives the links
an instantaneous rate change  dw = -b(theta) dv,  b = M(theta)^-1 [h1 cos th1, h2 cos th2]
(the integral of the cart-acceleration term). Between steps the cart
coasts at constant speed and the links move freely (gravity + friction).

One cycle of length Ts with loop latency d (sensor age + serial):
    coast d at the old velocity -> velocity step -> coast Ts-d at the new one.

Works on scalars or NumPy arrays (vectorised over a batch), so the same code
plans trajectories offline (swingup_trajectory.py) and predicts online.
State s = (th1, w1, th2, w2, x). Inertial constants come as a dict with
J1, J2, c, h1, h2, b1, c1, b2, c2, g.
"""

import numpy as np

COULOMB_SMOOTHING_RAD_S = 0.02


def _accel_free(k, th1, w1, th2, w2):
    d = th1 - th2
    cd, sd = np.cos(d), np.sin(d)
    wj = w2 - w1
    tau_j = -k["b2"] * wj - k["c2"] * np.tanh(wj / COULOMB_SMOOTHING_RAD_S)
    tau_p = -k["b1"] * w1 - k["c1"] * np.tanh(w1 / COULOMB_SMOOTHING_RAD_S)
    r1 = -k["c"] * sd * w2 * w2 + k["h1"] * k["g"] * np.sin(th1) + tau_p - tau_j
    r2 = k["c"] * sd * w1 * w1 + k["h2"] * k["g"] * np.sin(th2) + tau_j
    m12 = k["c"] * cd
    det = k["J1"] * k["J2"] - m12 * m12
    return (k["J2"] * r1 - m12 * r2) / det, (k["J1"] * r2 - m12 * r1) / det


def coupling(k, th1, th2):
    """b(theta): link rate change per unit cart velocity step is -b."""
    m12 = k["c"] * np.cos(th1 - th2)
    det = k["J1"] * k["J2"] - m12 * m12
    h1c = k["h1"] * np.cos(th1)
    h2c = k["h2"] * np.cos(th2)
    return (k["J2"] * h1c - m12 * h2c) / det, (k["J1"] * h2c - m12 * h1c) / det


def coast(k, s, v, T, substeps=1):
    th1, w1, th2, w2, x = s
    h = T / substeps
    for _ in range(substeps):
        def f(a, b, c_, d):
            a1, a2 = _accel_free(k, a, b, c_, d)
            return b, a1, d, a2
        k1 = f(th1, w1, th2, w2)
        k2 = f(th1 + 0.5 * h * k1[0], w1 + 0.5 * h * k1[1], th2 + 0.5 * h * k1[2], w2 + 0.5 * h * k1[3])
        k3 = f(th1 + 0.5 * h * k2[0], w1 + 0.5 * h * k2[1], th2 + 0.5 * h * k2[2], w2 + 0.5 * h * k2[3])
        k4 = f(th1 + h * k3[0], w1 + h * k3[1], th2 + h * k3[2], w2 + h * k3[3])
        th1 = th1 + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        w1 = w1 + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        th2 = th2 + h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        w2 = w2 + h / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
    return (th1, w1, th2, w2, x + v * T)


def cycle(k, s, v_old, v_new, Ts, delay):
    """State after one control cycle (see module docstring)."""
    s = coast(k, s, v_old, delay)
    th1, w1, th2, w2, x = s
    b1, b2 = coupling(k, th1, th2)
    dv = v_new - v_old
    s = (th1, w1 - b1 * dv, th2, w2 - b2 * dv, x)
    return coast(k, s, v_new, Ts - delay, substeps=1 if Ts - delay < 0.004 else 2)


def constants_from(params):
    """Accept a TwinParameters-like object or a dict with the needed keys."""
    keys = ("J1", "J2", "c", "h1", "h2", "b1", "c1", "b2", "c2")
    if isinstance(params, dict):
        k = {key: float(params[key]) for key in keys}
        k["g"] = float(params.get("g", 9.81))
        return k
    k = {key: float(getattr(params, key)) for key in keys}
    k["g"] = float(getattr(params, "g", 9.81))
    return k
