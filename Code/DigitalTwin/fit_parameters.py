"""Fit the uncertain twin parameters to real rig data.

Data used (from the `logs` branch, see README):
  * sysid_*/samples.csv  - the team's system-ID runs: free mode decays with the
    cart still, chirp / dwell / multisine / random cart excitation. The cart
    trajectory is known exactly (commanded step count), and both encoder
    angles are logged at 100 Hz.
  * optional: the swing-up run filmed in IMG_0494 for a large-angle check.

Method: many short windows (multiple shooting). Each window starts from the
measured angles and a smoothed rate estimate, is driven by the logged
pulse-position trajectory through the stepper servo model, and is compared
with the measured angles (shifted by the sensor delay). Parameters are found
with scipy.optimize.least_squares using a robust loss; 1-sigma uncertainties
come from the Jacobian, inflated by the spread of per-run fits.

Fitted: b1, c1 (pivot viscous/Coulomb), b2, c2 (elbow); the effective inertial
constants J1, J2, c (coupling) and h2 as scale factors on the nominal values
(h1 stays nominal: from link motion alone only M^-1 G is identifiable, i.e.
everything up to one overall scale); sensor_delay_s and motor_tau_s.

The most recent run is used for the twin's default ("current configuration");
the others are reported, and the run-to-run spread is folded into sigma.

Usage:
    python3 fit_parameters.py RUN_DIR [RUN_DIR ...] [--out fitted_parameters.json]
where RUN_DIR is a sysid_* folder containing samples.csv.
"""

import argparse
import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from twin_model import (  # noqa: E402
    TwinParameters, link_accelerations_np, hanging_mode_frequencies_hz, FITTED_PARAMETERS_PATH,
)

WINDOW_S = 3.0
STALE_AGE_MS = 12.0
DT = 0.002
FREE_NAMES = ("b1", "c1", "b2", "c2", "kJ1", "kJ2", "kc", "kh2", "sensor_delay_s", "motor_tau_s")
X0 = np.array([3e-4, 3e-4, 1e-4, 5e-5, 1.0, 1.0, 1.0, 1.0, 0.006, 0.010])
LOWER = np.array([0, 0, 0, 0, 0.6, 0.6, 0.6, 0.6, 0.0, 0.002])
UPPER = np.array([5e-3, 1e-2, 3e-3, 1e-2, 1.6, 1.6, 1.6, 1.6, 0.03, 0.05])
SCALE = np.array([1e-4, 1e-4, 1e-4, 1e-4, 0.1, 0.1, 0.1, 0.1, 0.005, 0.005])


def apply_theta(p, theta):
    _, _, _, _, kJ1, kJ2, kc, kh2, _, _ = theta
    return replace(p, J1_eff=p.J1 * kJ1, J2_eff=p.J2 * kJ2, c_eff=p.c * kc, h2_eff=p.h2 * kh2,
                   h1_eff=p.h1, b1=theta[0], c1=theta[1], b2=theta[2], c2=theta[3],
                   sensor_delay_s=theta[8], motor_tau_s=theta[9])


def load_sysid(run_dir):
    """Return dict of arrays in model convention (theta from upright, rad)."""
    path = Path(run_dir) / "samples.csv"
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    def col(k):
        return np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    t = col("t_s")
    # sysid convention: psi = +encoder counts from hanging; the balancing
    # controller's model angle is theta = pi - psi (see PythonMaster2_auto.py).
    th1 = math.pi - np.radians(col("psi1_deg"))
    th2 = math.pi - np.radians(col("psi2_deg"))
    # Stale radio samples (the receiver repeats an old angle; seen as flat
    # steps followed by jumps in the Oct-27 arm-2 data) are excluded.
    age1, age2 = col("age1_ms"), col("age2_ms")
    stale = (age1 > STALE_AGE_MS) | (age2 > STALE_AGE_MS) | ~np.isfinite(age1) | ~np.isfinite(age2)
    th1[stale] = np.nan
    th2[stale] = np.nan
    data = dict(t=t, th1=th1, th2=th2, x_cmd=col("x_steps_m"), v_cmd=col("v_cmd_m_s"),
                t_write=col("t_write_s"), test=np.array([r["test"] for r in rows]),
                name=Path(run_dir).name)
    return data


def make_windows(data, tests, window_s=WINDOW_S, stride_s=None):
    stride_s = stride_s or window_s
    t = data["t"]
    wins = []
    for test in tests:
        m = np.flatnonzero(data["test"] == test)
        if len(m) < 50:
            continue
        t0, t1 = t[m[0]], t[m[-1]]
        s = t0 + 0.2
        while s + window_s <= t1:
            i0 = int(np.searchsorted(t, s))
            i1 = int(np.searchsorted(t, s + window_s))
            ok = np.isfinite(data["th1"][i0:i1]) & np.isfinite(data["th2"][i0:i1])
            # need clean data at the start (initial rates) and mostly clean after
            if ok[:12].all() and ok.mean() > 0.9:
                wins.append((test, i0, i1))
            s += stride_s
    return wins


class WindowSet:
    """Pre-sampled inputs for a batch of windows, integrated together (vectorised)."""

    def __init__(self, data, wins):
        self.data = data
        self.wins = wins
        t = data["t"]
        n = int(round(WINDOW_S / DT))
        self.n = n
        W = len(wins)
        self.tgrid = np.zeros((W, n + 1))
        self.xcmd = np.zeros((W, n + 1))
        self.vcmd = np.zeros((W, n + 1))
        self.meas_t, self.meas_th1, self.meas_th2 = [], [], []
        self.init = np.zeros((W, 6))
        # smoothed rates for the initial conditions
        good = np.isfinite(data["th1"]) & np.isfinite(data["th2"])
        fill = lambda a: np.interp(t, t[good], np.unwrap(a[good]))
        th1s = fill(data["th1"])
        th2s = fill(data["th2"])
        self.good = good
        dtm = np.median(np.diff(t))
        w1 = savgol_filter(th1s, 9, 3, deriv=1, delta=dtm)
        w2 = savgol_filter(th2s, 9, 3, deriv=1, delta=dtm)
        tw = data["t_write"]
        for k, (_, i0, i1) in enumerate(wins):
            tg = t[i0] + DT * np.arange(n + 1)
            self.tgrid[k] = tg
            # zero-order hold of the commanded velocity from its write time
            j = np.clip(np.searchsorted(tw, tg, side="right") - 1, 0, len(tw) - 1)
            self.vcmd[k] = data["v_cmd"][j]
            # pulse position: integrate the held velocity from the logged count
            xc = data["x_cmd"][max(i0 - 1, 0)] + np.concatenate(([0], np.cumsum(self.vcmd[k][:-1] * DT)))
            self.xcmd[k] = xc
            self.init[k] = [th1s[i0], w1[i0], th2s[i0], w2[i0], xc[0], self.vcmd[k][0]]
            self.meas_t.append(t[i0:i1] - t[i0])
            self.meas_th1.append(np.where(good[i0:i1], th1s[i0:i1], np.nan))
            self.meas_th2.append(np.where(good[i0:i1], th2s[i0:i1], np.nan))

    def simulate(self, p, b1, c1, b2, c2, tau):
        W, n = self.xcmd.shape[0], self.n
        s = self.init.copy()
        out = np.zeros((W, n + 1, 2))
        out[:, 0, 0] = s[:, 0]
        out[:, 0, 1] = s[:, 2]
        wn = 1.0 / tau

        def f(s, xc, vc):
            a = np.clip(wn * wn * (xc - s[:, 4]) + 2 * wn * (vc - s[:, 5]), -p.motor_accel_max, p.motor_accel_max)
            al1, al2 = link_accelerations_np(p, s[:, 0], s[:, 1], s[:, 2], s[:, 3], a, b1, c1, b2, c2)
            return np.stack([s[:, 1], al1, s[:, 3], al2, s[:, 5], a], axis=1)

        for i in range(n):
            xc, vc = self.xcmd[:, i], self.vcmd[:, i]
            k1 = f(s, xc, vc)
            k2 = f(s + 0.5 * DT * k1, xc + 0.5 * DT * vc, vc)
            k3 = f(s + 0.5 * DT * k2, xc + 0.5 * DT * vc, vc)
            k4 = f(s + DT * k3, xc + DT * vc, vc)
            s = s + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            out[:, i + 1, 0] = s[:, 0]
            out[:, i + 1, 1] = s[:, 2]
        return out

    def residuals(self, p, theta, return_parts=False):
        q = apply_theta(p, theta)
        delay = theta[8]
        sim = self.simulate(q, q.b1, q.c1, q.b2, q.c2, q.motor_tau_s)
        tg = np.arange(self.n + 1) * DT
        res = []
        parts = []
        for k in range(len(self.wins)):
            mt = self.meas_t[k]
            # measurement at t saw the true state at t - delay
            st1 = np.interp(mt - delay, tg, sim[k, :, 0])
            st2 = np.interp(mt - delay, tg, sim[k, :, 1])
            r1 = st1 - self.meas_th1[k]
            r2 = st2 - self.meas_th2[k]
            # skip the first 0.1 s (initial-rate estimate settles)
            keep = (mt > 0.1) & np.isfinite(r1) & np.isfinite(r2)
            res.append(r1[keep])
            res.append(r2[keep])
            if return_parts:
                parts.append((mt, self.meas_th1[k], self.meas_th2[k], st1, st2))
        res = np.concatenate(res)
        return (res, parts) if return_parts else res


def fit(window_sets, p0, x0=X0):
    def fun(z):
        theta = z * SCALE
        return np.concatenate([ws.residuals(p0, theta) for ws in window_sets])
    sol = least_squares(fun, x0 / SCALE, bounds=(LOWER / SCALE, UPPER / SCALE),
                        loss="soft_l1", f_scale=0.03, x_scale=1.0, max_nfev=60)
    theta = sol.x * SCALE
    J = sol.jac
    dof = max(1, len(sol.fun) - len(theta))
    s2 = float(np.sum(sol.fun ** 2) / dof)
    try:
        cov = np.linalg.pinv(J.T @ J) * s2
        sigma = np.sqrt(np.clip(np.diag(cov), 0, None)) * SCALE
    except np.linalg.LinAlgError:
        sigma = np.full(len(theta), np.nan)
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    return theta, sigma, rms, sol


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default=str(FITTED_PARAMETERS_PATH))
    ap.add_argument("--plot", default=str(HERE / "plots" / "fit_sysid.png"))
    args = ap.parse_args(argv)
    p0 = TwinParameters()
    tests = ("mode1_decay_1", "mode1_decay_2", "mode1_decay_3", "mode2_decay_1",
             "mode2_decay_2", "chirp", "dwell_1", "dwell_2", "dwell_3", "dwell_4",
             "multisine", "random", "moves")
    per_run = []
    window_sets = []
    for run in args.runs:
        data = load_sysid(run)
        wins = make_windows(data, tests, stride_s=4.5)
        ws = WindowSet(data, wins)
        window_sets.append(ws)
        theta, sigma, rms, _ = fit([ws], p0)
        per_run.append((data["name"], theta, sigma, rms, len(wins)))
        print(f"{data['name']}: windows={len(wins)} rms={math.degrees(rms):.2f} deg",
              dict(zip(FREE_NAMES, np.round(theta, 6))), flush=True)
    # The most recent run defines the current configuration (the hardware
    # changed between sessions; see README). Its own Jacobian sigma is combined
    # with the run-to-run spread.
    name, theta, sigma, rms, _ = per_run[-1]
    spread = np.std([r[1] for r in per_run], axis=0) if len(per_run) > 1 else np.zeros_like(theta)
    unc = np.sqrt(sigma ** 2 + spread ** 2)
    fitted = apply_theta(p0, theta)
    k_unc = dict(zip(FREE_NAMES, unc))
    params = {
        "b1": {"value": theta[0], "sigma": unc[0], "unit": "N m s/rad", "what": "pivot viscous friction"},
        "c1": {"value": theta[1], "sigma": unc[1], "unit": "N m", "what": "pivot Coulomb friction"},
        "b2": {"value": theta[2], "sigma": unc[2], "unit": "N m s/rad", "what": "elbow viscous friction"},
        "c2": {"value": theta[3], "sigma": unc[3], "unit": "N m", "what": "elbow Coulomb friction"},
        "J1_eff": {"value": fitted.J1, "sigma": p0.J1 * k_unc["kJ1"], "unit": "kg m^2",
                   "what": f"m1 l1^2 + m2 L1^2 + I1 (nominal {p0.J1:.5g})"},
        "J2_eff": {"value": fitted.J2, "sigma": p0.J2 * k_unc["kJ2"], "unit": "kg m^2",
                   "what": f"m2 l2^2 + I2 (nominal {p0.J2:.5g})"},
        "c_eff": {"value": fitted.c, "sigma": p0.c * k_unc["kc"], "unit": "kg m^2",
                  "what": f"m2 L1 l2 coupling (nominal {p0.c:.5g})"},
        "h1_eff": {"value": fitted.h1, "sigma": 0.0, "unit": "kg m",
                   "what": "m1 l1 + m2 L1, held at nominal (sets the overall scale)"},
        "h2_eff": {"value": fitted.h2, "sigma": p0.h2 * k_unc["kh2"], "unit": "kg m",
                   "what": f"m2 l2 (nominal {p0.h2:.5g})"},
        "sensor_delay_s": {"value": theta[8], "sigma": unc[8], "unit": "s",
                           "what": "encoder sample -> Pi (radio+receiver+USB), excludes TX sample age"},
        "motor_tau_s": {"value": theta[9], "sigma": unc[9], "unit": "s",
                        "what": "1/bandwidth of the stepper servo (weakly identifiable)"},
    }
    result = {
        "description": "Twin parameters fitted by fit_parameters.py; nominal values for "
                       "everything else come from Code/Global/physical_parameters.py.",
        "current_configuration_run": name,
        "data": [r[0] for r in per_run],
        "rms_angle_error_deg": math.degrees(rms),
        "hanging_modes_hz_fitted": list(hanging_mode_frequencies_hz(fitted)),
        "hanging_modes_hz_nominal": list(hanging_mode_frequencies_hz(p0)),
        "per_run": [{"run": r[0], "windows": r[4], "rms_deg": math.degrees(r[3]),
                     "hanging_modes_hz": list(hanging_mode_frequencies_hz(apply_theta(p0, r[1]))),
                     **{n: float(v) for n, v in zip(FREE_NAMES, r[1])},
                     "jacobian_sigma": {n: float(v) for n, v in zip(FREE_NAMES, r[2])}} for r in per_run],
        "parameters": {k: {kk: (float(vv) if isinstance(vv, (float, np.floating)) else vv)
                           for kk, vv in v.items()} for k, v in params.items()},
    }
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    plot_validation(window_sets, p0, theta, args.plot)
    # also the earlier run with its own parameters, for the README
    if len(per_run) > 1:
        plot_validation(window_sets[:1], p0, per_run[0][1],
                        str(Path(args.plot).with_name("fit_sysid_earlier_run.png")))
    return 0


def plot_validation(window_sets, p0, theta, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    ws = window_sets[-1]
    _, parts = ws.residuals(p0, theta, return_parts=True)
    pick = {}
    for k, (test, _, _) in enumerate(ws.wins):
        key = test.split("_")[0] if test.startswith(("dwell", "mode1", "mode2")) else test
        pick.setdefault(key, k)
    keys = list(pick)[:6]
    fig, axes = plt.subplots(len(keys), 1, figsize=(10, 2.1 * len(keys)))
    for ax, key in zip(np.atleast_1d(axes), keys):
        mt, m1, m2, s1, s2 = parts[pick[key]]
        ax.plot(mt, np.degrees(m1 - np.pi), color="#999", lw=2.5, label="encoder arm 1")
        ax.plot(mt, np.degrees(s1 - np.pi), color="#1f6fb4", lw=1, label="twin arm 1")
        ax.plot(mt, np.degrees(m2 - np.pi), color="#ccc", lw=2.5, label="encoder arm 2")
        ax.plot(mt, np.degrees(s2 - np.pi), color="#d9480f", lw=1, label="twin arm 2")
        ax.set_ylabel(f"{key}\n(deg from hanging)", fontsize=8)
        ax.grid(alpha=0.3)
    np.atleast_1d(axes)[0].legend(fontsize=7, ncol=4, loc="upper right")
    np.atleast_1d(axes)[0].set_title(f"Twin vs. sysid log {ws.data['name']} "
                                     "(3 s open-loop windows driven by the logged cart motion)")
    np.atleast_1d(axes)[-1].set_xlabel("time in window (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
