"""Design the balance LQR + estimator for the improved double-pendulum controller.

Model (see Code/DigitalTwin/twin_model.py): kinematic cart, because the
closed-loop stepper follows commanded step rates. The controller's output is
a cart VELOCITY held for one cycle (exactly what the Arduino timer does), so
the design uses the change of coordinates

    w~ = w - Bq v          (Bq = -M^-1 H, the link response to cart accel)

in which the link equations are driven smoothly by v and an exact
zero-order-hold discretisation applies. The loop latency d (sensor age +
serial) is modelled as the velocity command landing d seconds into the
interval:

    s[k+1] = Ad s[k] + B1 v[k-1] + B2 v[k],  B1, B2 from the split interval.

Augmented state z = [th1, w~1, th2, w~2, x, v[k-1], xi]  (xi = integral of x)
and input dv = v[k] - v[k-1], so the input weight penalises cart
ACCELERATION (dv / Ts) and the integral state removes the steady cart
offset caused by small encoder-calibration errors.

Gains are written to improved_gains.json (read at runtime with NumPy only).
Run:  python3 design_lqr.py [--profile conservative|nominal] [--params fitted|nominal]
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are

HERE = Path(__file__).resolve().parent
CODE_DIRECTORY = HERE.parents[1]
for path in (CODE_DIRECTORY, CODE_DIRECTORY / "DigitalTwin"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_model import TwinParameters, load_parameters, linearized_upright  # noqa: E402

GAINS_PATH = HERE / "improved_gains.json"

# Bryson-style limits: the state "allowed excursion" (bigger = softer).
PROFILES = {
    # First hardware tests: lower bandwidth, gentle cart, wide tolerance.
    "conservative": dict(theta_deg=4.0, rate_deg_s=80.0, cart_m=0.10, speed_m_s=0.5,
                         integral_m_s=0.15, accel_m_s2=2.0),
    # Aggressive gains used only for the first ~1 s after a swing-up catch.
    "catch": dict(theta_deg=2.0, rate_deg_s=120.0, cart_m=0.20, speed_m_s=0.8,
                  integral_m_s=0.5, accel_m_s2=8.0),
    # Tighter tracking once the conservative profile balances reliably.
    "nominal": dict(theta_deg=2.0, rate_deg_s=60.0, cart_m=0.06, speed_m_s=0.4,
                    integral_m_s=0.10, accel_m_s2=3.0),
}


def velocity_input_model(p):
    """Continuous (Ac, Bc, Bq) for s = [th1, w~1, th2, w~2, x], input v."""
    A, B = linearized_upright(p, include_friction=True)
    Kq = A[np.ix_([1, 3], [0, 2])]
    Kw = A[np.ix_([1, 3], [1, 3])]
    Bq = B[[1, 3], 0]
    Ac = np.zeros((5, 5))
    Bc = np.zeros(5)
    Ac[0, 1] = 1.0
    Bc[0] = Bq[0]
    Ac[2, 3] = 1.0
    Bc[2] = Bq[1]
    for row, i in ((1, 0), (3, 1)):
        Ac[row, [0, 2]] = Kq[i]
        Ac[row, [1, 3]] = Kw[i]
        Bc[row] = Kw[i] @ Bq
    Bc[4] = 1.0
    return Ac, Bc.reshape(5, 1), Bq


def zoh_split(Ac, Bc, Ts, delay):
    """Ad, and B1 (previous command, first `delay` s), B2 (new command, rest)."""
    n = Ac.shape[0]

    def gamma(tau):
        M = np.zeros((n + 1, n + 1))
        M[:n, :n] = Ac
        M[:n, n:] = Bc
        E = expm(M * tau)
        return E[:n, :n], E[:n, n:]
    Ad, _ = gamma(Ts)
    delay = min(max(delay, 0.0), Ts)
    Phi_rest, G_rest = gamma(Ts - delay)
    _, G_delay = gamma(delay)
    B2 = G_rest
    B1 = Phi_rest @ G_delay
    return Ad, B1, B2


def augmented(Ad, B1, B2, Ts):
    """z = [s(5), v_prev, xi]; input dv."""
    Az = np.zeros((7, 7))
    Bz = np.zeros((7, 1))
    Az[:5, :5] = Ad
    Az[:5, 5:6] = B1 + B2
    Bz[:5] = B2
    Az[5, 5] = 1.0
    Bz[5, 0] = 1.0
    Az[6, 4] = Ts
    Az[6, 6] = 1.0
    return Az, Bz


def design(p, Ts=0.010, delay=0.009, profile="conservative"):
    lim = PROFILES[profile]
    Ac, Bc, Bq = velocity_input_model(p)
    Ad, B1, B2 = zoh_split(Ac, Bc, Ts, delay)
    Az, Bz = augmented(Ad, B1, B2, Ts)
    th = math.radians(lim["theta_deg"])
    w = math.radians(lim["rate_deg_s"])
    Q = np.diag([1 / th ** 2, 1 / w ** 2, 1 / th ** 2, 1 / w ** 2, 1 / lim["cart_m"] ** 2,
                 1 / lim["speed_m_s"] ** 2, 1 / lim["integral_m_s"] ** 2])
    R = np.array([[1 / (lim["accel_m_s2"] * Ts) ** 2]])
    P = solve_discrete_are(Az, Bz, Q, R)
    K = np.linalg.solve(R + Bz.T @ P @ Bz, Bz.T @ P @ Az)
    poles = np.linalg.eigvals(Az - Bz @ K)
    # Steady-state Kalman filter for the four link states (x, v known exactly).
    Ap = Ad[:4, :4]
    C = np.array([[1.0, 0, 0, 0], [0, 0, 1.0, 0]])
    enc = 2 * math.pi / 16384
    Rn = np.diag([(1.5 * enc) ** 2, (1.5 * enc) ** 2])          # quantisation + latency jitter
    Qn = np.diag([1e-8, (0.08) ** 2, 1e-8, (0.12) ** 2])        # unmodelled accel per step
    Pn = solve_discrete_are(Ap.T, C.T, Qn, Rn)
    L = Pn @ C.T @ np.linalg.inv(C @ Pn @ C.T + Rn)
    return dict(
        profile=profile, Ts=Ts, delay=delay, limits=lim,
        Ad=Ad.tolist(), B1=B1.tolist(), B2=B2.tolist(), Bq=Bq.tolist(),
        K=K.tolist(), P=P.tolist(), L=L.tolist(),
        closed_loop_poles_abs=sorted(np.abs(poles).tolist(), reverse=True),
        open_loop_unstable_poles_rad_s=sorted([float(e.real) for e in np.linalg.eigvals(Ac) if e.real > 1e-6], reverse=True),
        params={**p.inertial(), "b1": p.b1, "c1": p.c1, "b2": p.b2, "c2": p.c2, "g": p.g},
        state_order=["theta1", "w1_tilde", "theta2", "w2_tilde", "x", "v_prev", "xi"],
        note="u = dv = -K z ; v_cmd = v_prev + dv ; w_tilde = w - Bq * v_applied",
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", choices=sorted(PROFILES), default=None,
                    help="write only this profile (default: all)")
    ap.add_argument("--params", choices=("fitted", "nominal"), default="fitted")
    ap.add_argument("--Ts", type=float, default=0.010)
    ap.add_argument("--delay", type=float, default=0.009)
    ap.add_argument("--out", default=str(GAINS_PATH))
    args = ap.parse_args(argv)
    p = load_parameters() if args.params == "fitted" else TwinParameters()
    profiles = [args.profile] if args.profile else sorted(PROFILES)
    out = {"source": f"design_lqr.py --params {args.params}", "default_profile": "conservative",
           "profiles": {name: design(p, args.Ts, args.delay, name) for name in profiles}}
    Path(args.out).write_text(json.dumps(out, indent=1) + "\n")
    for name, d in out["profiles"].items():
        print(name, "K =", np.round(np.array(d["K"]), 3).tolist())
        print("   |closed-loop poles| max", round(d["closed_loop_poles_abs"][0], 4),
              " open-loop unstable", np.round(d["open_loop_unstable_poles_rad_s"], 2).tolist())
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
