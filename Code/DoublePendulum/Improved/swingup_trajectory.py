"""Offline swing-up planner + time-varying LQR (TVLQR) tracking gains.

Why this exists: in the digital twin, pure energy pumping (the classic
single-pendulum method, kept in improved_controller.py as swing_mode
"energy") raises the energy in ~2 s but the two links tumble chaotically and
practically never pass the upright slowly enough to catch on this rail
(+/-0.28 m soft limit, ~1 m/s). The standard fix for a double pendulum on a
cart (e.g. Graichen et al., Automatica 2007) is a planned feed-forward cart
motion from hanging to upright plus feedback that keeps the links on that
plan. The team's own Pi-only PythonMaster2_swingup.py also planned a 2.5 s
trajectory but tracked it with a fixed balance gain and aborted with ~35 deg
tracking error on every logged attempt. Here the plan and the tracking gains
are computed on the same discrete model (plan_model.cycle) that matches how
the Pi actuates the stepper (velocity steps every cycle, loop latency), with
the twin's fitted parameters.

Decision variables: cart-acceleration knots (piecewise linear in time); the
commanded velocity per cycle is that profile sampled at the cycle midpoint.
Cost: terminal error at upright (both links, rates, cart at rest near 0) +
penalties for rail/speed/accel limits + small effort regularisation.
Several frequencies/phases are tried; the best feasible plan is kept.

TVLQR: z = [th1, w1, th2, w2, x, v_prev], u = dv (velocity change this cycle).
Backward Riccati along the plan from the upright LQR cost-to-go.

Output: swingup_plan.json (read at runtime with NumPy only).
Run:    python3 swingup_trajectory.py [--duration 2.5] [--params fitted|nominal]
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
CODE_DIRECTORY = HERE.parents[1]
for path in (CODE_DIRECTORY, CODE_DIRECTORY / "DigitalTwin", HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_model import TwinParameters, load_parameters  # noqa: E402
from plan_model import cycle, constants_from  # noqa: E402

PLAN_PATH = HERE / "swingup_plan.json"


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Planner:
    def __init__(self, k, duration, Ts=0.01, delay=0.009, knots=None, x_max=0.22, v_max=0.8,
                 a_max=7.0, hold_s=0.3):
        self.k, self.T, self.Ts, self.delay = k, duration, Ts, delay
        self.N = int(round(duration / Ts))
        self.hold = int(round(hold_s / Ts))     # cart must be at rest for the last steps
        knots = knots or int(round(14 * duration))   # ~14 knots per second
        self.M = knots
        self.tk = np.linspace(0, duration, knots)
        self.x_max, self.v_max, self.a_max = x_max, v_max, a_max
        tmid = (np.arange(self.N) + 0.5) * Ts
        # v(t_mid) = integral of the piecewise-linear acceleration: linear map S @ a
        fine = np.linspace(0, duration, 2001)
        basis = np.array([np.interp(fine, self.tk, np.eye(knots)[j]) for j in range(knots)])
        integ = np.cumsum(basis, axis=1) * (fine[1] - fine[0])
        self.S = np.array([np.interp(tmid, fine, integ[j]) for j in range(knots)]).T  # (N, M)
        # Terminal cost = the upright LQR cost-to-go, so the plan ends where the
        # catch is easiest (not just "close to 0 deg" with awkward rates).
        Au, Bu, _ = linearize(k, np.zeros(6), 0.0, Ts, delay)
        w = TRACK_WEIGHTS
        Q = np.diag([1 / math.radians(w["angle_deg"]) ** 2, 1 / math.radians(w["rate_deg_s"]) ** 2,
                     1 / math.radians(w["angle_deg"]) ** 2, 1 / math.radians(w["rate_deg_s"]) ** 2,
                     1 / w["cart_m"] ** 2, 1 / w["speed_m_s"] ** 2])
        R = np.array([[1 / (w["accel_m_s2"] * Ts) ** 2]])
        P = solve_discrete_are(Au, Bu, Q, R)
        self.term_W = np.linalg.cholesky(P).T * (6.0 / math.sqrt(P[0, 0]))

    def velocities(self, A):
        """A: (B, M) knots -> (B, N) commanded velocities."""
        return A @ self.S.T

    def simulate(self, A):
        A = np.atleast_2d(A)
        V = self.velocities(A)
        B = A.shape[0]
        s = (np.full(B, np.pi), np.zeros(B), np.full(B, np.pi), np.zeros(B), np.zeros(B))
        v_old = np.zeros(B)
        X = np.zeros((B, self.N + 1, 5))
        X[:, 0] = np.stack(s, axis=1)
        for i in range(self.N):
            s = cycle(self.k, s, v_old, V[:, i], self.Ts, self.delay)
            v_old = V[:, i]
            X[:, i + 1] = np.stack(s, axis=1)
        return X, V

    def residuals_batch(self, A):
        X, V = self.simulate(A)
        th1, w1, th2, w2, x = (X[:, -1, j] for j in range(5))
        zN = np.stack([wrap(th1), w1, wrap(th2), w2, x, V[:, -1]], axis=1)
        # explicit "arrive at rest, centred" + a little LQR cost-to-go shaping
        term = np.concatenate([zN * np.array([8.0, 2.0, 8.0, 2.0, 6.0, 6.0]),
                               0.3 * zN @ self.term_W.T], axis=1)
        relu = lambda z: np.maximum(z, 0)
        pen_x = 40 * relu(np.abs(X[:, :, 4]) - self.x_max)
        pen_v = 10 * relu(np.abs(V) - self.v_max)
        pen_a = 3 * relu(np.abs(A) - self.a_max)
        rest = 3 * V[:, -self.hold:]
        reg = 0.003 * A
        return np.concatenate([term, pen_x, pen_v, pen_a, rest, reg], axis=1)

    def solve(self, A0, max_nfev=150):
        eps = 1e-4

        def fun(a):
            return self.residuals_batch(a[None])[0]

        def jac(a):
            batch = np.vstack([a, a + eps * np.eye(len(a))])
            R = self.residuals_batch(batch)
            return ((R[1:] - R[0]) / eps).T

        sol = least_squares(fun, A0, jac=jac, method="trf", max_nfev=max_nfev, xtol=1e-10, ftol=1e-10)
        return sol

    def report(self, A):
        X, V = self.simulate(A)
        X, V = X[0], V[0]
        return dict(final_deg=[math.degrees(wrap(X[-1, 0])), math.degrees(wrap(X[-1, 2]))],
                    final_rates=[X[-1, 1], X[-1, 3]], final_x=X[-1, 4], final_v=V[-1],
                    max_x=float(np.abs(X[:, 4]).max()), max_v=float(np.abs(V).max()),
                    max_a=float(np.abs(np.diff(np.concatenate([[0], V]))).max() / self.Ts))


def energy_rollout_guess(pl, k, gain, sync, center=6.0, v_max=0.8, a_max=7.0):
    """Initial guess from the energy-pumping law (the classic swing-up),
    simulated on the plan model; returns acceleration knots."""
    Eu = 2 * k["g"] * (k["h1"] + k["h2"])
    s = (np.pi, 0.0, np.pi, 0.0, 0.0)
    v = 0.0
    V = []
    for i in range(pl.N):
        th1, w1, th2, w2, x = s
        E = (0.5 * k["J1"] * w1 ** 2 + 0.5 * k["J2"] * w2 ** 2 + k["c"] * np.cos(th1 - th2) * w1 * w2
             + k["g"] * (k["h1"] * (np.cos(th1) + 1) + k["h2"] * (np.cos(th2) + 1)))
        y = k["h1"] * np.cos(th1) * w1 + k["h2"] * np.cos(th2) * w2
        a = -gain * (Eu - E) * np.clip(y / 0.15, -1, 1) - center * x - 3.0 * v
        if i < 10:
            a = 3.0 if i < 5 else -3.0
        m12 = k["c"] * np.cos(th1 - th2)
        det = k["J1"] * k["J2"] - m12 * m12
        b1 = (k["J2"] * k["h1"] * np.cos(th1) - m12 * k["h2"] * np.cos(th2)) / det
        b2 = (k["J1"] * k["h2"] * np.cos(th2) - m12 * k["h1"] * np.cos(th1)) / det
        a += sync * (b2 - b1) * (w2 - w1)
        v_new = float(np.clip(v + np.clip(a, -a_max, a_max) * pl.Ts, -v_max, v_max))
        s = cycle(k, s, v, v_new, pl.Ts, pl.delay)
        v = v_new
        V.append(v)
    A, *_ = np.linalg.lstsq(pl.S, np.array(V), rcond=None)
    return A


def linearize(k, z, u, Ts, delay, eps=1e-6):
    def F(zz, uu):
        s = tuple(zz[:5])
        v_new = zz[5] + uu
        s2 = cycle(k, s, zz[5], v_new, Ts, delay)
        return np.array([*s2, v_new])
    f0 = F(z, u)
    A = np.zeros((6, 6))
    for j in range(6):
        dz = np.zeros(6)
        dz[j] = eps
        A[:, j] = (F(z + dz, u) - F(z - dz, u)) / (2 * eps)
    B = ((F(z, u + eps) - F(z, u - eps)) / (2 * eps)).reshape(6, 1)
    return A, B, f0


def tvlqr(k, Z, U, Ts, delay, Q, R, Qf_scale=1.0):
    N = len(U)
    Au, Bu, _ = linearize(k, np.zeros(6), 0.0, Ts, delay)
    Pf = solve_discrete_are(Au, Bu, Q, R) * Qf_scale
    P = Pf
    K = np.zeros((N, 6))
    for i in range(N - 1, -1, -1):
        A, B, _ = linearize(k, Z[i], U[i], Ts, delay)
        S = R + B.T @ P @ B
        Ki = np.linalg.solve(S, B.T @ P @ A)
        P = Q + A.T @ P @ (A - B @ Ki)
        P = 0.5 * (P + P.T)
        K[i] = Ki[0]
    Kf = np.linalg.solve(R + Bu.T @ Pf @ Bu, Bu.T @ Pf @ Au)[0]
    return K, Kf, Pf


TRACK_WEIGHTS = dict(angle_deg=1.5, rate_deg_s=60.0, cart_m=0.2, speed_m_s=0.6, accel_m_s2=15.0)


def tracking_gains(plan, weights=None):
    """(Re)compute the TVLQR gains for a stored plan."""
    w = dict(TRACK_WEIGHTS, **(weights or {}))
    k = {kk: float(v) for kk, v in plan["constants"].items()}
    Ts, delay = plan["Ts"], plan["delay"]
    Z = np.array(plan["z_nominal"])
    U = np.array(plan["dv_nominal"])
    Q = np.diag([1 / math.radians(w["angle_deg"]) ** 2, 1 / math.radians(w["rate_deg_s"]) ** 2,
                 1 / math.radians(w["angle_deg"]) ** 2, 1 / math.radians(w["rate_deg_s"]) ** 2,
                 1 / w["cart_m"] ** 2, 1 / w["speed_m_s"] ** 2])
    R = np.array([[1 / (w["accel_m_s2"] * Ts) ** 2]])
    K, Kf, _ = tvlqr(k, Z, U, Ts, delay, Q, R)
    plan = dict(plan)
    plan["K"] = np.round(K, 5).tolist()
    plan["K_final"] = np.round(Kf, 5).tolist()
    plan["tracking_weights"] = w
    return plan


def build_plan(pl, A, rep, k, args, weights):
    T = pl.T
    Ts = pl.Ts
    X, V = pl.simulate(A)
    X, V = X[0], V[0]
    Z = np.column_stack([X[:-1], np.concatenate([[0.0], V[:-1]])])   # z_k = [s_k, v_{k-1}]
    U = V - Z[:, 5]
    out = {
        "source": f"swingup_trajectory.py --params {args.params}",
        "Ts": Ts, "delay": args.delay, "duration_s": T, "steps": len(U),
        "limits": {"x_max": args.x_max, "v_max": args.v_max, "a_max": args.a_max},
        "report": {kk: (float(vv) if np.isscalar(vv) else [float(q) for q in vv]) for kk, vv in rep.items()},
        "constants": k,
        "state_order": ["theta1", "w1", "theta2", "w2", "x", "v_prev"],
        "z_nominal": np.round(Z, 6).tolist(),
        "z_final": np.round(np.concatenate([X[-1], [V[-1]]]), 6).tolist(),
        "dv_nominal": np.round(U, 6).tolist(),
        "accel_knots": np.asarray(A).tolist(),
        "note": "v_cmd = v_nominal(t) - K[k] @ (z - z_nominal(t)) with angle errors wrapped",
    }
    return tracking_gains(out, weights)


def _solve_job(job):
    T, seed, params, delay, x_max, v_max, a_max = job
    p = load_parameters() if params == "fitted" else TwinParameters()
    k = constants_from(p)
    pl = Planner(k, T, 0.01, delay, x_max=x_max, v_max=v_max, a_max=a_max)
    rng = np.random.default_rng(1000 * seed + int(T * 10))
    if seed % 3 == 2:
        # energy-pumping rollout (the classic swing-up law) as the starting guess
        A0 = energy_rollout_guess(pl, k, gain=rng.uniform(5, 30), sync=rng.uniform(0, 0.05),
                                  v_max=v_max, a_max=a_max)
    else:
        f = rng.uniform(0.7, 1.1)
        ph = rng.uniform(0, 2 * np.pi)
        amp = rng.uniform(2, 5)
        A0 = amp * np.sin(2 * np.pi * f * pl.tk + ph) * np.minimum(1, pl.tk / 0.4)
    sol = pl.solve(A0, max_nfev=250)
    cost = float(np.sum(sol.fun ** 2))
    rep = pl.report(sol.x)
    ok = (max(abs(a) for a in rep["final_deg"]) < 2 and rep["max_x"] <= x_max + 0.01
          and rep["max_v"] <= v_max + 0.02)
    return cost, ok, sol.x.tolist(), rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, nargs="+", default=[3.0, 3.5, 4.0])
    ap.add_argument("--params", choices=("fitted", "nominal"), default="fitted")
    ap.add_argument("--x-max", type=float, default=0.22)
    ap.add_argument("--v-max", type=float, default=0.8)
    ap.add_argument("--a-max", type=float, default=7.0)
    ap.add_argument("--delay", type=float, default=0.009)
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--out", default=str(PLAN_PATH))
    ap.add_argument("--polish", action="store_true",
                    help="continue optimising the stored plan (same duration), then retune")
    ap.add_argument("--retune", action="store_true",
                    help="keep the stored plan, only recompute the TVLQR gains")
    for name, val in TRACK_WEIGHTS.items():
        ap.add_argument(f"--w-{name.replace('_', '-')}", type=float, default=val)
    args = ap.parse_args(argv)
    weights = {name: getattr(args, "w_" + name) for name in TRACK_WEIGHTS}
    if args.polish:
        old = json.loads(Path(args.out).read_text())
        p = load_parameters() if args.params == "fitted" else TwinParameters()
        k = constants_from(p)
        lim = old["limits"]
        args.x_max, args.v_max, args.a_max, args.delay = lim["x_max"], lim["v_max"], lim["a_max"], old["delay"]
        pl = Planner(k, old["duration_s"], old["Ts"], old["delay"], x_max=lim["x_max"],
                     v_max=lim["v_max"], a_max=lim["a_max"])
        Zold = np.array(old["z_nominal"])
        Vold = np.concatenate([Zold[1:, 5], [old["z_final"][5]]])
        A0 = np.array(old["accel_knots"]) if "accel_knots" in old else np.linalg.lstsq(pl.S, Vold, rcond=None)[0]
        sol = pl.solve(A0, max_nfev=1500)
        rep = pl.report(sol.x)
        print("polished:", {kk: np.round(vv, 3) for kk, vv in rep.items()})
        plan = build_plan(pl, sol.x, rep, k, args, weights)
        Path(args.out).write_text(json.dumps(plan) + "\n")
        return 0
    if args.retune:
        plan = tracking_gains(json.loads(Path(args.out).read_text()), weights)
        Path(args.out).write_text(json.dumps(plan) + "\n")
        print("retuned", args.out, weights)
        return 0
    p = load_parameters() if args.params == "fitted" else TwinParameters()
    k = constants_from(p)
    Ts = 0.01
    jobs = [(T, seed, args.params, args.delay, args.x_max, args.v_max, args.a_max)
            for T in args.duration for seed in range(args.seeds)]
    import multiprocessing as mp
    with mp.Pool(max(1, mp.cpu_count())) as pool:
        results = pool.map(_solve_job, jobs)
    best = None
    for (T, seed, *_), (cost, ok, A_sol, rep) in zip(jobs, results):
        print(f"T={T:.1f} seed={seed} cost={cost:.4f} ok={ok} final={np.round(rep['final_deg'], 2)} "
              f"rates={np.round(rep['final_rates'], 2)} max_x={rep['max_x']:.3f} max_v={rep['max_v']:.2f} "
              f"max_a={rep['max_a']:.1f}", flush=True)
        score = cost + (0 if ok else 100)
        if best is None or score < best[0]:
            best = (score, T, np.array(A_sol), Planner(k, T, Ts, args.delay, x_max=args.x_max,
                                                       v_max=args.v_max, a_max=args.a_max), rep)
    _, T, A, pl, rep = best
    # polish the best plan with a longer solve
    sol = pl.solve(A, max_nfev=800)
    if np.sum(sol.fun ** 2) < best[0] or best[0] >= 100:
        A = sol.x
        rep = pl.report(A)
    print(f"polished: cost={np.sum(sol.fun ** 2):.4f} final={np.round(rep['final_deg'], 2)} "
          f"rates={np.round(rep['final_rates'], 2)} x={rep['final_x']:.3f} v={rep['final_v']:.3f}", flush=True)
    out = build_plan(pl, A, rep, k, args, weights)
    Path(args.out).write_text(json.dumps(out) + "\n")
    print("best plan:", T, "s", rep)
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
