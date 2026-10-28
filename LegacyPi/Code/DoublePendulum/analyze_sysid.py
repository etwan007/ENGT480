#!/usr/bin/env python3
"""analyze_sysid.py - fit the ENGT480 rig's dynamics from PendulumSysID.py runs.

    python3 analyze_sysid.py run_logs/sysid_20261016_...        one run
    python3 analyze_sysid.py run_logs/sysid_*                    several runs, plus a combined summary
    python3 analyze_sysid.py RUN --arm1-mass 0.207 --arm1-com 0.087
                              measured values (override RIG in PendulumSysID.py)

Needs Python 3.9+, numpy, scipy and matplotlib (pip install numpy scipy matplotlib).
Writes into each run folder's analysis/ folder:
    report.md               the results in words and tables, with the figures
    results.json            every fitted number, with uncertainties
    twin_rig_settings.json  values for the digital twin (run_twin.py --rig, or type them into Rig settings)
    01_overview.png ... 09_motor.png
With several runs it also writes sysid_summary.md / .json next to the run folders.

What it fits
  Single arm:  psi'' = -w0^2 sin(psi) - k cos(psi) a_cart(t - d) - cv psi' - cc sgn(psi') - cq psi'|psi'|
    w0 and the friction terms come from the free decays (period vs amplitude, energy lost per swing),
    k and d from everything the cart did (simulated in short windows from the measured state).
    For a rigid arm k = w0^2 / g exactly; k*g/w0^2 is reported as the "cart scale" (1.00 = the
    m-per-step value and the arm are as assumed).
  Double arm:  the twin's two-link model with every term divided by arm 1's pivot inertia J1:
    P1 = p1/J1, P2 = p2/J1, J2/J1, H = L1*p2/J1, B1 = b1/J1, B2 = b2/J1, cart scale, delay.
    Masses and lengths you measured (or the PythonMasterMath2 values) set the absolute scale.
"""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy import optimize, signal, special

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

G = 9.81
ENC = 16384
RAD = 2 * math.pi / ENC
CART_CMD_LATENCY_S = 0.0008     # write() to the Arduino acting on it (USB + 4 bytes at 115200); the fit's delay adds to it
DT_SIM = 0.004
DOUBLE_MODAL_MAX_DEG = 25.0     # double: mode envelopes are read only while both arms swing less than this
NOMINAL = {"arm1": {"m": 0.207, "c": 0.087, "I": 0.00123324992, "L": 0.2},
           "arm2": {"m": 0.210, "c": 0.106, "I": 0.00197563667, "L": 0.25}}

# chart styling (reference palette, light)
C_MEAS, C_MODEL, C_THIRD, C_FOURTH = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": "#c9c8c3", "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "legend.frameon": False, "lines.linewidth": 1.4,
})


# ------------------------------------------------------------------ reading a run
def read_table(path, text_cols=()):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r if row]
    out = {}
    for j, name in enumerate(header):
        vals = [row[j] if j < len(row) else "" for row in rows]
        if name in text_cols:
            out[name] = np.array(vals, dtype=object)
            continue
        try:
            out[name] = np.array([float(v) if v != "" else np.nan for v in vals])
        except ValueError:
            out[name] = np.array(vals, dtype=object)
    return out


def wrap_counts(c):
    return (np.asarray(c) + ENC // 2) % ENC - ENC // 2


def robust_std(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if len(x) else float("nan")


def pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if len(x) else float("nan")


class Run:
    def __init__(self, folder):
        self.dir = Path(folder)
        self.name = self.dir.name
        with open(self.dir / "metadata.json", encoding="utf-8") as f:
            self.meta = json.load(f)
        self.arms = int(self.meta["arms"])
        self.mps = float(self.meta["settings"]["M_PER_STEP"])
        cal = self.meta.get("calibration", {})
        self.hang1 = cal.get("hang1_counts")
        self.aligned2 = cal.get("aligned2_counts")
        self.fr = read_table(self.dir / "angle_frames.csv", ("status1", "status2", "status3"))
        self.sm = read_table(self.dir / "samples.csv", ("test", "phase", "flags"))
        self.mc = read_table(self.dir / "motor_commands.csv", ("note",))
        self.sg = read_table(self.dir / "segments.csv", ("test", "kind", "status", "params", "results"))
        self.ev = read_table(self.dir / "events.csv", ("test", "kind", "text"))
        self.zero = self.meta.get("step_zero")
        self.model_used = self.meta.get("model_used") or self.meta.get("summary", {}).get("model_used") or {}
        self._clock()
        self._angles()
        self._cart()
        self._intervals()

    # receiver clock -> Pi clock (lower envelope: the frames that arrived fastest)
    def _clock(self):
        th = self.fr["t_host_s"]
        rx = self.fr["rx_ms"] / 1000.0
        piece = np.concatenate([[0], np.cumsum(np.diff(rx) < -1.0)]).astype(int)
        self.fr_piece = piece
        self.clock = {}
        lat = np.full(len(th), np.nan)
        for p in np.unique(piece):
            m = piece == p
            x, y = rx[m], th[m]
            if m.sum() < 50:
                A = np.array([1.0, float(np.min(y - x))])
            else:
                A = np.polyfit(x - x[0], y, 1)
                for _ in range(5):
                    r = y - np.polyval(A, x - x[0])
                    keep = r <= np.percentile(r, 15)
                    A = np.polyfit(x[keep] - x[0], y[keep], 1)
                r = y - np.polyval(A, x - x[0])
                A[1] += np.percentile(r, 0.5)
            self.clock[int(p)] = (float(A[0]), float(A[1]), float(x[0]))
            lat[m] = y - (A[0] * (x - x[0]) + A[1])
        self.frame_latency_ms = lat * 1000.0
        big = max(self.clock, key=lambda p: (piece == p).sum())
        self.drift_ppm = (self.clock[big][0] - 1.0) * 1e6

    def rx_to_host(self, rx_s, piece):
        out = np.empty(len(rx_s))
        for p, (a, b, x0) in self.clock.items():
            m = piece == p
            out[m] = a * (rx_s[m] - x0) + b
        return out

    def _angles(self):
        fr = self.fr
        rx = fr["rx_ms"]
        self.ch = {}
        for i, (raw_k, st_k, age_k) in enumerate((("raw1", "status1", "age1_ms"), ("raw2", "status2", "age2_ms"))):
            ok = (fr[st_k] == "valid") & np.isfinite(fr[raw_k]) & (fr[raw_k] < ENC)
            if ok.sum() < 10:
                continue
            ts_rx = (rx[ok] - fr[age_k][ok]) / 1000.0
            piece = self.fr_piece[ok]
            raw = fr[raw_k][ok]
            key = np.round(ts_rx * 1000).astype(np.int64) * 8 + piece
            _, first = np.unique(key, return_index=True)
            first = np.sort(first)
            t = self.rx_to_host(ts_rx[first], piece[first])
            order = np.argsort(t)
            self.ch[i] = {"t": t[order], "raw": raw[first][order], "age": fr[age_k][ok][first][order]}
        c1 = self.ch[0]
        c1["psi"] = np.unwrap(wrap_counts(c1["raw"] - self.hang1) * RAD)
        if self.arms == 2 and 1 in self.ch and self.aligned2 is not None:
            c2 = self.ch[1]
            c2["joint"] = np.unwrap(wrap_counts(c2["raw"] - self.aligned2) * RAD)

    def _cart(self):
        mc = self.mc
        ok = np.isfinite(mc["steps"]) & np.isfinite(mc["t_write_s"])
        t = mc["t_write_s"][ok] + CART_CMD_LATENCY_S
        if self.zero is None:
            self.zero = float(mc["steps"][ok][0])
        x = (mc["steps"][ok] - self.zero) * self.mps
        v = np.where(mc["timer_cmd"][ok] == -1, 0.0, mc["v_cmd_m_s"][ok])
        v = np.nan_to_num(v)
        self.c_t, self.c_x, self.c_v = t, x, v
        self.t_motor_on = float(t[0]) if len(t) else float("nan")
        pred = x[:-1] + v[:-1] * np.diff(t)
        self.step_resid = (x[1:] - pred) / self.mps
        self.reply_ms = mc["reply_ms"][ok]

    def cart_x(self, t):
        t = np.asarray(t, float)
        i = np.searchsorted(self.c_t, t, side="right") - 1
        before = i < 0
        i = np.clip(i, 0, len(self.c_t) - 1)
        x = self.c_x[i] + self.c_v[i] * (t - self.c_t[i])
        return np.where(before, self.c_x[0], x)

    def cart_v(self, t, h=DT_SIM):
        return (self.cart_x(np.asarray(t) + h / 2) - self.cart_x(np.asarray(t) - h / 2)) / h

    def _intervals(self):
        """Contiguous runs of (test, phase) from samples.csv."""
        t = self.sm["t_s"]
        seg = self.sm["seg"]
        ph = self.sm["phase"]
        name = self.sm["test"]
        self.intervals = []
        if not len(t):
            return
        start = 0
        for i in range(1, len(t) + 1):
            if i == len(t) or seg[i] != seg[start] or ph[i] != ph[start]:
                self.intervals.append({"seg": int(seg[start]), "test": str(name[start]), "phase": str(ph[start]),
                                       "t0": float(t[start]), "t1": float(t[i - 1])})
                start = i
        self.segments = []
        for j in range(len(self.sg["seg"])):
            try:
                params = json.loads(self.sg["params"][j]) if self.sg["params"][j] else {}
                res = json.loads(self.sg["results"][j]) if self.sg["results"][j] else {}
            except ValueError:
                params, res = {}, {}
            self.segments.append({"seg": int(self.sg["seg"][j]), "test": str(self.sg["test"][j]),
                                  "kind": str(self.sg["kind"][j]), "t0": self.sg["t_start_s"][j],
                                  "t1": self.sg["t_end_s"][j], "status": str(self.sg["status"][j]),
                                  "params": params, "results": res})
        self.kind_of = {s["seg"]: s["kind"] for s in self.segments}

    def decay_intervals(self):
        return [iv for iv in self.intervals
                if iv["phase"] == "decay" and self.kind_of.get(iv["seg"]) in ("pump_decay", "mode_decay")
                and iv["t1"] - iv["t0"] > 2.0]

    def still_window(self, name):
        for s in self.segments:
            if s["test"] == name and np.isfinite(s["t0"]):
                return s["t0"], s["t1"]
        return None


# ------------------------------------------------------------------ timing and sensors
def timing_stats(run):
    sm = run.sm
    cyc = sm["cycle_ms"]
    age1 = run.ch[0]["age"]
    out = {
        "cycle_ms": {"mean": float(np.nanmean(cyc)), "p50": pct(cyc, 50), "p99": pct(cyc, 99), "max": float(np.nanmax(cyc))},
        "arduino_reply_ms": {"mean": float(np.nanmean(run.reply_ms)), "p50": pct(run.reply_ms, 50),
                             "p99": pct(run.reply_ms, 99), "max": float(np.nanmax(run.reply_ms))},
        "frame_transport_extra_ms": {"p50": pct(run.frame_latency_ms, 50), "p99": pct(run.frame_latency_ms, 99),
                                     "max": float(np.nanmax(run.frame_latency_ms))},
        "tx1_age_ms": {"mean": float(np.mean(age1)), "p99": pct(age1, 99), "max": float(np.max(age1))},
        "receiver_clock_drift_ppm": run.drift_ppm,
        "frames": int(len(run.fr["seq"])),
        "frame_gaps": int(np.nansum(run.fr["gap"])),
        "crc_errors": int(np.nanmax(run.fr["crc_errors"])) if len(run.fr["crc_errors"]) else 0,
        "tx1_not_valid_frames": int(np.sum(run.fr["status1"] != "valid")),
    }
    if run.arms == 2:
        out["tx2_not_valid_frames"] = int(np.sum(run.fr["status2"] != "valid"))
        out["tx2_age_ms"] = {"mean": float(np.mean(run.ch[1]["age"])), "p99": pct(run.ch[1]["age"], 99)}
    flags = sm["flags"]
    for f in ("a_clip", "v_clip", "brake", "stale_angle"):
        out[f"cycles_{f}"] = int(sum(f in str(x) for x in flags))
    return out


def sensor_stats(run):
    out = {}
    for name in ("still_off", "still_on"):
        w = run.still_window(name)
        if not w:
            continue
        d = {}
        for i in run.ch:
            c = run.ch[i]
            m = (c["t"] >= w[0] + 0.2) & (c["t"] <= w[1])
            if m.sum() < 20:
                continue
            raw = c["raw"][m]
            dev = wrap_counts(raw - np.round(np.median(raw)))
            dev = dev - np.mean(dev)
            d[f"tx{i + 1}"] = {"n": int(m.sum()), "std_counts": float(np.std(dev)), "std_deg": float(np.std(dev) * 360 / ENC),
                               "peak_to_peak_counts": float(np.ptp(dev)), "distinct_values": int(len(np.unique(raw))),
                               "mean_counts": float(np.median(raw) + np.mean(wrap_counts(raw - np.median(raw))))}
        out[name] = d
    return out


# ------------------------------------------------------------------ single arm: free decays
def ell_half_period_factor(A):
    """Half period x w0 for a frictionless pendulum of amplitude A (rad): 2 K(sin^2(A/2))."""
    return 2.0 * special.ellipk(np.sin(np.asarray(A) / 2.0) ** 2)


def visc_integral(A, n=200):
    """J(A) = integral over -A..A of sqrt(2(cos psi - cos A)) d psi."""
    A = np.atleast_1d(np.asarray(A, float))
    th = np.linspace(-math.pi / 2, math.pi / 2, n)
    out = []
    for a in A:
        psi = a * np.sin(th)
        f = np.sqrt(np.maximum(2 * (np.cos(psi) - math.cos(a)), 0.0)) * a * np.cos(th)
        out.append(np.trapezoid(f, th) if hasattr(np, "trapezoid") else np.trapz(f, th))
    return np.array(out)


def quad_integral(A):
    """Q(A) = integral over -A..A of 2(cos psi - cos A) d psi."""
    A = np.asarray(A, float)
    return 4 * np.sin(A) - 4 * A * np.cos(A)


def find_extrema(t, y, T_half_guess):
    off = np.median(y)
    s = np.sign(y - off)
    s[s == 0] = 1
    zc = np.where(s[1:] != s[:-1])[0]
    # drop crossings that are too close (noise near zero)
    keep = []
    for z in zc:
        if keep and t[z] - t[keep[-1]] < 0.35 * T_half_guess:
            continue
        keep.append(z)
    ext = []
    for a, b in zip(keep[:-1], keep[1:]):
        seg = slice(a + 1, b + 1)
        if b - a < 3:
            continue
        idx = a + 1 + int(np.argmax(np.abs(y[seg] - off)))
        sel = (t >= t[idx] - 0.05) & (t <= t[idx] + 0.05)
        if sel.sum() < 5:
            continue
        tt = t[sel] - t[idx]
        c = np.polyfit(tt, y[sel], 2)
        tp = -c[1] / (2 * c[0]) if c[0] != 0 else 0.0
        if abs(tp) > 0.05:
            tp = 0.0
        ext.append((t[idx] + tp, float(np.polyval(c, tp))))
    return np.array(ext).reshape(-1, 2)


def decay_analysis_single(run, w0_guess):
    rows = []      # per half swing: decay#, t, A_k, A_k1, T_half
    traces = []
    c = run.ch[0]
    for j, iv in enumerate(run.decay_intervals()):
        m = (c["t"] >= iv["t0"] + 0.05) & (c["t"] <= iv["t1"])
        t, y = c["t"][m], c["psi"][m]
        if len(t) < 100:
            continue
        ext = find_extrema(t, y, math.pi / w0_guess)
        if len(ext) < 6:
            continue
        te, ye = ext[:, 0], ext[:, 1]
        offs = (ye[:-2] + 2 * ye[1:-1] + ye[2:]) / 4.0
        off = float(np.median(offs))
        A = np.abs(ye - off)
        traces.append({"test": iv["test"], "t": t, "y": y, "te": te, "ye": ye, "off": off, "A": A, "t0": iv["t0"]})
        for k in range(len(A) - 1):
            rows.append((j, te[k], A[k], A[k + 1], te[k + 1] - te[k]))
    if len(rows) < 8:
        return None
    R = np.array(rows)
    Amid = (R[:, 2] + R[:, 3]) / 2
    Th = R[:, 4]
    Kf = ell_half_period_factor(Amid)
    good = (Amid > math.radians(0.8)) & np.isfinite(Th)
    # w0 from the exact pendulum period, with outlier rejection
    for _ in range(3):
        inv_w0 = np.sum(Kf[good] * Th[good]) / np.sum(Kf[good] ** 2)
        res = Th - Kf * inv_w0
        mad = robust_std(res[good])
        good = good & (np.abs(res) < max(5 * mad, 1e-4))
    w0 = 1.0 / inv_w0
    # energy lost per half swing (per unit pivot inertia): w0^2 (cos A_k+1 - cos A_k)
    dE = w0 ** 2 * (np.cos(R[:, 3]) - np.cos(R[:, 2]))
    Jv = visc_integral(Amid) * w0
    Cc = 2 * Amid
    Qq = w0 ** 2 * quad_integral(Amid)
    use = good & (Amid > math.radians(1.0))
    X = np.column_stack([Jv, Cc, Qq])[use]
    y = dE[use]
    scale = np.max(np.abs(X), axis=0)
    scale[scale == 0] = 1
    coef3, _ = optimize.nnls(X / scale, y)
    coef3 = coef3 / scale
    cv_only = float(np.sum(X[:, 0] * y) / np.sum(X[:, 0] ** 2))
    coef2, _ = optimize.nnls(X[:, :2] / scale[:2], y)
    coef2 = coef2 / scale[:2]
    # bootstrap uncertainty
    rng = np.random.default_rng(0)
    idx_all = np.where(use)[0]
    boots = []
    for _ in range(300):
        b = rng.choice(idx_all, len(idx_all))
        Xb = np.column_stack([Jv, Cc, Qq])[b]
        cb, _ = optimize.nnls(Xb / scale, dE[b])
        iw = np.sum(Kf[b] * Th[b]) / np.sum(Kf[b] ** 2)
        boots.append([1 / iw, *(cb / scale), float(np.sum(Xb[:, 0] * dE[b]) / np.sum(Xb[:, 0] ** 2))])
    boots = np.array(boots)
    sd = np.std(boots, axis=0)

    def rss(pred):
        return float(np.sum((y - pred) ** 2))
    n = len(y)
    fits = {
        "viscous": {"cv": cv_only, "rss": rss(X[:, 0] * cv_only), "k": 1},
        "viscous+dry": {"cv": float(coef2[0]), "cc": float(coef2[1]), "rss": rss(X[:, :2] @ coef2), "k": 2},
        "viscous+dry+air": {"cv": float(coef3[0]), "cc": float(coef3[1]), "cq": float(coef3[2]),
                            "rss": rss(X @ coef3), "k": 3},
    }
    for f in fits.values():
        f["aic"] = n * math.log(max(f["rss"], 1e-30) / n) + 2 * f["k"]
    best = min(fits, key=lambda k: fits[k]["aic"])
    bf = fits[best]
    out = {"w0": float(w0), "w0_sd": float(sd[0]), "f0_hz": float(w0 / (2 * math.pi)), "T0_s": float(2 * math.pi / w0),
           "L_equiv_m": float(G / w0 ** 2), "half_swings": int(use.sum()), "decays": len(traces),
           "amp_range_deg": [float(np.degrees(Amid[use].min())), float(np.degrees(Amid[use].max()))],
           "cv": float(bf.get("cv", 0.0)), "cc": float(bf.get("cc", 0.0)), "cq": float(bf.get("cq", 0.0)),
           "cv_sd": float(sd[1] if best != "viscous" else sd[4]), "cc_sd": float(sd[2]) if "cc" in bf else 0.0,
           "cq_sd": float(sd[3]) if "cq" in bf else 0.0,
           "cv_viscous_only": cv_only, "cv_viscous_only_sd": float(sd[4]),
           "model_fits": fits, "best_by_aic": best,
           "period_resid_ms_rms": float(np.sqrt(np.mean(res[good] ** 2)) * 1000),
           "offset_deg": [float(np.degrees(tr["off"])) for tr in traces]}
    out["_arrays"] = {"Amid": Amid, "Th": Th, "dE": dE, "use": use, "good": good, "traces": traces}
    return out


def sim_single_free(w0, cv, cc, cq, psi0, dpsi0, t_eval, dt=0.001):
    """RK4 from (psi0, dpsi0) at t_eval[0]; returns psi at t_eval."""
    def f(p, v):
        return v, -w0 * w0 * math.sin(p) - cv * v - cc * math.tanh(v / 0.02) - cq * v * abs(v)
    t0, t1 = t_eval[0], t_eval[-1]
    n = int(math.ceil((t1 - t0) / dt)) + 1
    ps = np.empty(n + 1)
    p, v = psi0, dpsi0
    ps[0] = p
    for i in range(n):
        k1 = f(p, v)
        k2 = f(p + dt / 2 * k1[0], v + dt / 2 * k1[1])
        k3 = f(p + dt / 2 * k2[0], v + dt / 2 * k2[1])
        k4 = f(p + dt * k3[0], v + dt * k3[1])
        p += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        v += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        ps[i + 1] = p
    tg = t0 + np.arange(n + 1) * dt
    return np.interp(t_eval, tg, ps)


# ------------------------------------------------------------------ windowed simulation fits
class Windows:
    """Short stretches of the run, each simulated from the measured state at its start."""

    def __init__(self, run, length, t_from, t_to, min_amp_deg=0.0, ranges=None):
        c1 = run.ch[0]
        self.run = run
        self.length = length
        self.N = int(round(length / DT_SIM))
        if ranges is None:
            ranges = [(t_from, t_to)]
        starts = np.concatenate([np.arange(a, b - length, length) for a, b in ranges] or [np.array([])])
        keep = []
        ics = []
        for t0 in starts:
            ic = self._ic(c1["t"], c1["psi"], t0)
            if ic is None:
                continue
            m = (c1["t"] >= t0) & (c1["t"] <= t0 + length)
            tt = c1["t"][m]
            if len(tt) < 0.6 * length * 100 or np.max(np.diff(np.concatenate([[t0], tt, [t0 + length]]))) > 0.06:
                continue
            if np.ptp(c1["psi"][m]) < math.radians(min_amp_deg):
                continue
            if run.arms == 2:
                c2 = run.ch[1]
                ic2 = self._ic(c2["t"], c2["joint"], t0)
                if ic2 is None:
                    continue
                m2 = (c2["t"] >= t0) & (c2["t"] <= t0 + length)
                if m2.sum() < 0.6 * length * 100:
                    continue
                ic = (ic[0], ic[1], ic[0] + ic2[0], ic[1] + ic2[1])
            keep.append(t0)
            ics.append(ic)
        self.t0 = np.array(keep)
        self.ic = np.array(ics)
        self.W = len(self.t0)
        if self.W == 0:
            return
        self.T_half = self.t0[:, None] + np.arange(2 * self.N + 1)[None, :] * (DT_SIM / 2)
        # measured samples inside windows (skip the first 20 ms)
        self.obs = []
        for key, ch_i in (("psi", 0), ("joint", 1)):
            if ch_i not in run.ch or key not in run.ch[ch_i]:
                continue
            c = run.ch[ch_i]
            w = np.searchsorted(self.t0, c["t"], side="right") - 1
            ok = w >= 0
            w = np.clip(w, 0, self.W - 1)
            rel = c["t"] - self.t0[w]
            ok &= (rel > 0.02) & (rel < length - DT_SIM)
            pos = rel[ok] / DT_SIM
            i = np.floor(pos).astype(int)
            self.obs.append({"key": key, "w": w[ok], "i": i, "f": pos - i, "y": c[key][ok], "t": c["t"][ok]})

    @staticmethod
    def _ic(t, y, t0):
        m = (t >= t0 - 0.05) & (t <= t0 + 0.05)
        if m.sum() < 6:
            return None
        c = np.polyfit(t[m] - t0, y[m], 3)
        return (float(c[3]), float(c[2]))

    def interp(self, grid, ob):
        g = grid[ob["w"], ob["i"]]
        g1 = grid[ob["w"], np.minimum(ob["i"] + 1, grid.shape[1] - 1)]
        return g + (g1 - g) * ob["f"]


def sim_windows_single(win, w0, k, d, cv, cc, cq, off):
    run = win.run
    V = run.cart_v(win.T_half - d)
    dt = DT_SIM
    psi = win.ic[:, 0] - off
    u = win.ic[:, 1] + k * np.cos(psi) * V[:, 0]
    out = np.empty((win.W, win.N + 1))
    out[:, 0] = psi
    w2 = w0 * w0

    def f(p, uu, v):
        dp = uu - k * np.cos(p) * v
        du = -w2 * np.sin(p) - cv * dp - cc * np.tanh(dp / 0.02) - cq * dp * np.abs(dp) - k * np.sin(p) * dp * v
        return dp, du
    for n in range(win.N):
        v0, vh, v1 = V[:, 2 * n], V[:, 2 * n + 1], V[:, 2 * n + 2]
        a1, b1 = f(psi, u, v0)
        a2, b2 = f(psi + dt / 2 * a1, u + dt / 2 * b1, vh)
        a3, b3 = f(psi + dt / 2 * a2, u + dt / 2 * b2, vh)
        a4, b4 = f(psi + dt * a3, u + dt * b3, v1)
        psi = psi + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        u = u + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4)
        out[:, n + 1] = psi
    return out + off


def fit_cart_single(run, dec, w0_start, k_start):
    t_from = run.t_motor_on + 0.3
    t_to = float(np.nanmax(run.c_t))
    win = Windows(run, 2.0, t_from, t_to, min_amp_deg=0.5)
    if win.W < 5:
        return None
    ob = win.obs[0]
    cv, cc, cq = (dec["cv"], dec["cc"], dec["cq"]) if dec else (0.05, 0.0, 0.0)

    def resid(p):
        w0, k, d_ms, off_deg = p
        g = sim_windows_single(win, w0, k, d_ms / 1000.0, cv, cc, cq, math.radians(off_deg))
        return win.interp(g, ob) - ob["y"]
    x0 = [w0_start, k_start, 10.0, 0.0]
    lb = [0.5 * w0_start, -60.0, -20.0, -5.0]
    ub = [1.5 * w0_start, 60.0, 100.0, 5.0]
    t_start = time.time()
    res = optimize.least_squares(resid, x0, bounds=(lb, ub), x_scale=[0.1, 0.5, 5.0, 0.2],
                                 loss="soft_l1", f_scale=math.radians(0.5), max_nfev=120)
    r = res.fun
    J = res.jac
    dof = max(len(r) - len(x0), 1)
    try:
        cov = np.linalg.inv(J.T @ J) * (np.sum(r ** 2) / dof)
        sd = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        sd = np.full(len(x0), np.nan)
    w0, k, d_ms, off = res.x
    g = sim_windows_single(win, w0, k, d_ms / 1000.0, cv, cc, cq, math.radians(off))
    pred = win.interp(g, ob)
    out = {"w0": float(w0), "w0_sd": float(sd[0]), "k": float(k), "k_sd": float(sd[1]),
           "delay_ms": float(d_ms), "delay_sd_ms": float(sd[2]), "offset_deg": float(off),
           "cart_scale": float(k * G / w0 ** 2), "cart_scale_sd": float(abs(sd[1] * G / w0 ** 2)),
           "windows": int(win.W), "samples": int(len(r)), "rms_deg": float(np.degrees(np.sqrt(np.mean((pred - ob["y"]) ** 2)))),
           "signal_rms_deg": float(np.degrees(np.std(ob["y"]))), "seconds": round(time.time() - t_start, 1)}
    out["fit_pct"] = float(100 * (1 - np.linalg.norm(pred - ob["y"]) / np.linalg.norm(ob["y"] - np.mean(ob["y"]))))
    out["_win"] = (win, g)
    return out


# ------------------------------------------------------------------ double arm
def nominal_double_ratios():
    a1, a2 = NOMINAL["arm1"], NOMINAL["arm2"]
    J1 = a1["I"] + a1["m"] * a1["c"] ** 2 + a2["m"] * a1["L"] ** 2
    p1 = a1["m"] * a1["c"] + a2["m"] * a1["L"]
    p2 = a2["m"] * a2["c"]
    J2 = a2["I"] + a2["m"] * a2["c"] ** 2
    return {"P1": p1 / J1, "P2": p2 / J1, "J2r": J2 / J1, "H": a1["L"] * p2 / J1, "J1": J1}


def linear_modes(P1, P2, J2r, H, B1=0.0, B2=0.0):
    M = np.array([[1.0, H], [H, J2r]])
    K = G * np.diag([P1, P2])
    Bm = np.array([[B1 + B2, -B2], [-B2, B2]])
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [-np.linalg.solve(M, K), -np.linalg.solve(M, Bm)]])
    ev = np.linalg.eigvals(A)
    ev = ev[np.imag(ev) > 0]
    ev = ev[np.argsort(np.abs(ev))]
    wn = np.abs(ev)
    zeta = -np.real(ev) / wn
    return wn / (2 * math.pi), zeta


def sim_windows_double(win, p, d, dry=(0.0, 0.0)):
    P1, P2, J2r, H, B1, B2, sa = p
    C1, C2 = dry
    run = win.run
    V = run.cart_v(win.T_half - d)
    dt = DT_SIM
    s1, w1, s2, w2 = (win.ic[:, i].copy() for i in range(4))

    def vel(s1, s2, q1, q2, v):
        r1 = q1 - sa * P1 * np.cos(s1) * v
        r2 = q2 - sa * P2 * np.cos(s2) * v
        hc = H * np.cos(s1 - s2)
        det = J2r - hc * hc
        return (J2r * r1 - hc * r2) / det, (r2 - hc * r1) / det

    def f(s1, s2, q1, q2, v):
        d1, d2 = vel(s1, s2, q1, q2, v)
        hs = H * np.sin(s1 - s2) * d1 * d2
        rel = d2 - d1
        t1 = -B1 * d1 + B2 * rel
        t2 = -B2 * rel
        if C1 or C2:
            f1, f2 = C1 * np.tanh(d1 / 0.02), C2 * np.tanh(rel / 0.02)
            t1 = t1 - f1 + f2
            t2 = t2 - f2
        dq1 = -hs - sa * P1 * np.sin(s1) * d1 * v - G * P1 * np.sin(s1) + t1
        dq2 = hs - sa * P2 * np.sin(s2) * d2 * v - G * P2 * np.sin(s2) + t2
        return d1, d2, dq1, dq2
    hc = H * np.cos(s1 - s2)
    q1 = w1 + hc * w2 + sa * P1 * np.cos(s1) * V[:, 0]
    q2 = hc * w1 + J2r * w2 + sa * P2 * np.cos(s2) * V[:, 0]
    o1 = np.empty((win.W, win.N + 1))
    o2 = np.empty((win.W, win.N + 1))
    o1[:, 0], o2[:, 0] = s1, s2
    for n in range(win.N):
        v0, vh, v1 = V[:, 2 * n], V[:, 2 * n + 1], V[:, 2 * n + 2]
        k1 = f(s1, s2, q1, q2, v0)
        k2 = f(s1 + dt / 2 * k1[0], s2 + dt / 2 * k1[1], q1 + dt / 2 * k1[2], q2 + dt / 2 * k1[3], vh)
        k3 = f(s1 + dt / 2 * k2[0], s2 + dt / 2 * k2[1], q1 + dt / 2 * k2[2], q2 + dt / 2 * k2[3], vh)
        k4 = f(s1 + dt * k3[0], s2 + dt * k3[1], q1 + dt * k3[2], q2 + dt * k3[3], v1)
        s1 = s1 + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        s2 = s2 + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        q1 = q1 + dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        q2 = q2 + dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
        o1[:, n + 1], o2[:, n + 1] = s1, s2
    return o1, o2


DOUBLE_NAMES = ["P1", "P2", "J2r", "H", "B1", "B2", "cart_scale", "delay_ms"]


def fit_double(run):
    t_from = run.t_motor_on + 0.3
    t_to = float(np.nanmax(run.c_t))
    win = Windows(run, 1.5, t_from, t_to, min_amp_deg=0.5)
    if win.W < 5:
        return None
    nom = nominal_double_ratios()
    sign = float(np.sign(run.model_used.get("k_sign", 1.0)) or 1.0)
    ob1 = [o for o in win.obs if o["key"] == "psi"][0]
    ob2 = [o for o in win.obs if o["key"] == "joint"][0]
    # start from the nominal arms, scaled to the mode frequencies measured during the run
    fm = run.model_used.get("f_modes")
    P1, P2 = nom["P1"], nom["P2"]
    if fm and len(fm) == 2:
        f_nom, _ = linear_modes(P1, P2, nom["J2r"], nom["H"])
        P1 *= (fm[0] / f_nom[0]) ** 2
        P2 *= (fm[0] / f_nom[0]) ** 2
    x0 = [P1, P2, nom["J2r"], nom["H"], 0.02, 0.01, sign, 10.0]
    lb = [0.2 * x0[0], 0.2 * x0[1], 0.05, 0.0, 0.0, 0.0, -2.0, -20.0]
    ub = [5 * x0[0], 5 * x0[1], 3.0, 1.5, 2.0, 2.0, 2.0, 100.0]

    def resid(p):
        o1, o2 = sim_windows_double(win, p[:7], p[7] / 1000.0)
        r1 = win.interp(o1, ob1) - ob1["y"]
        r2 = (win.interp(o2, ob2) - win.interp(o1, ob2)) - ob2["y"]
        return np.concatenate([r1, r2])
    t_start = time.time()
    res = optimize.least_squares(resid, x0, bounds=(lb, ub), x_scale="jac", loss="soft_l1",
                                 f_scale=math.radians(0.5), max_nfev=150)
    r = res.fun
    dof = max(len(r) - len(x0), 1)
    try:
        cov = np.linalg.inv(res.jac.T @ res.jac) * (np.sum(r ** 2) / dof)
        sd = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        sd = np.full(len(x0), np.nan)
    p = res.x.copy()
    out = {n: float(v) for n, v in zip(DOUBLE_NAMES, p)}
    out.update({n + "_sd": float(s) for n, s in zip(DOUBLE_NAMES, sd)})
    # stage 2: friction from how fast each mode dies away in the free decays (envelopes are not
    # thrown off by small phase errors, unlike long simulated windows)
    dry = (0.0, 0.0)
    md = modal_damping_double(run, *p[:4])
    if md and md.get("B") is not None:
        out["friction_from_windows"] = {"B1": float(p[4]), "B2": float(p[5])}
        p[4], p[5] = md["B"]
        out.update({"B1": float(p[4]), "B2": float(p[5]), "B1_sd": float("nan"), "B2_sd": float("nan"),
                    "modal_decay": strip(md),
                    "friction_from": f"mode envelopes in {md['decays_used']} free decays"})
    o1, o2 = sim_windows_double(win, p[:7], p[7] / 1000.0, dry)
    pred1 = win.interp(o1, ob1)
    pred2 = win.interp(o2, ob2) - win.interp(o1, ob2)
    fmod, zeta = linear_modes(*p[:6])
    out.update({"mode_freq_hz": [float(f) for f in fmod], "mode_damping_ratio": [float(z) for z in zeta],
                "windows": int(win.W), "samples": int(len(r)),
                "rms_deg_arm1": float(np.degrees(np.sqrt(np.mean((pred1 - ob1["y"]) ** 2)))),
                "rms_deg_joint": float(np.degrees(np.sqrt(np.mean((pred2 - ob2["y"]) ** 2)))),
                "fit_pct_arm1": float(100 * (1 - np.linalg.norm(pred1 - ob1["y"]) / np.linalg.norm(ob1["y"] - ob1["y"].mean()))),
                "fit_pct_joint": float(100 * (1 - np.linalg.norm(pred2 - ob2["y"]) / np.linalg.norm(ob2["y"] - ob2["y"].mean()))),
                "seconds": round(time.time() - t_start, 1), "start": dict(zip(DOUBLE_NAMES, map(float, x0)))})
    out["_win"] = (win, o1, o2)
    return out


def modal_damping_double(run, P1, P2, J2r, H):
    """Damping ratio of each mode from the envelope of its modal coordinate in the free decays,
    then the pivot and joint friction (B1, B2) that give those ratios."""
    M = np.array([[1.0, H], [H, J2r]])
    K = G * np.diag([P1, P2])
    w2, Phi = np.linalg.eig(np.linalg.solve(M, K))
    order = np.argsort(w2.real)
    w = np.sqrt(w2.real[order])
    Phi = Phi[:, order].real
    Phi_inv = np.linalg.inv(Phi)
    c1, c2 = run.ch[0], run.ch.get(1)
    if c2 is None:
        return None
    rates = [[], []]
    for iv in run.decay_intervals():
        t = np.arange(iv["t0"] + 0.2, iv["t1"], 0.01)
        if len(t) < 500:
            continue
        p1 = np.interp(t, c1["t"], c1["psi"])
        p2 = p1 + np.interp(t, c2["t"], c2["joint"])
        # larger swings trade energy between the modes, so use only the part below DOUBLE_MODAL_MAX_DEG
        big = np.maximum(np.abs(p1 - p1.mean()), np.abs(p2 - p2.mean()))
        roll = np.array([big[max(0, k - 60):k + 60].max() for k in range(len(big))])
        small = roll < math.radians(DOUBLE_MODAL_MAX_DEG)
        if small.sum() < 600:
            continue
        idx = np.where(small)[0]
        t, p1, p2 = t[idx[0]:idx[-1] + 1], p1[idx[0]:idx[-1] + 1], p2[idx[0]:idx[-1] + 1]
        eta = Phi_inv @ np.vstack([p1 - p1.mean(), p2 - p2.mean()])
        for i in range(2):
            f = w[i] / (2 * math.pi)
            b, a = signal.butter(2, [0.7 * f, min(1.3 * f, 45.0)], btype="band", fs=100)
            y = signal.filtfilt(b, a, eta[i])
            env = np.abs(signal.hilbert(y))
            m = slice(150, len(t) - 150)
            e, tt = env[m], t[m]
            ok = e > math.radians(0.3)
            if ok.sum() < 300:
                continue
            slope, icpt = np.polyfit(tt[ok], np.log(e[ok]), 1)
            resid = np.log(e[ok]) - (slope * tt[ok] + icpt)
            if np.std(resid) > 0.2:
                continue
            rates[i].append((-slope, float(tt[ok][-1] - tt[ok][0]), iv["test"], float(np.exp(icpt + slope * tt[ok][0]))))
    if not rates[0] or not rates[1]:
        return {"B": None, "rates": rates}
    # weight each estimate by how long and how large that mode's swing was
    sig = [float(np.average([r[0] for r in rates[i]], weights=[r[1] * r[3] for r in rates[i]])) for i in range(2)]
    zeta = [sig[i] / w[i] for i in range(2)]
    # 2 sigma_i (phi_i' M phi_i) = B1 phi_i1^2 + B2 (phi_i1 - phi_i2)^2
    A = np.array([[Phi[0, i] ** 2, (Phi[0, i] - Phi[1, i]) ** 2] for i in range(2)])
    rhs = np.array([2 * sig[i] * (Phi[:, i] @ M @ Phi[:, i]) for i in range(2)])
    B, _ = optimize.nnls(A, rhs)
    return {"B": [float(B[0]), float(B[1])], "zeta": zeta, "sigma": sig,
            "decays_used": len({r[2] for rr in rates for r in rr}),
            "rates": [[{"test": r[2], "sigma": r[0], "seconds": r[1], "start_amp_modal": r[3]}
                       for r in rates[i]] for i in range(2)]}


def decay_spectra_double(run):
    """Peak frequencies of each arm during the free decays."""
    out = []
    c1, c2 = run.ch[0], run.ch.get(1)
    for iv in run.decay_intervals():
        m = (c1["t"] >= iv["t0"] + 0.2) & (c1["t"] <= iv["t1"])
        if m.sum() < 300:
            continue
        t = np.arange(c1["t"][m][0], c1["t"][m][-1], 0.01)
        y1 = np.interp(t, c1["t"], c1["psi"])
        res = {"test": iv["test"], "seconds": round(float(t[-1] - t[0]), 2)}
        ys = {"arm1": y1}
        if c2 is not None:
            ys["joint"] = np.interp(t, c2["t"], c2["joint"])
        for k, y in ys.items():
            y = (y - y.mean()) * np.hanning(len(y))
            F = np.abs(np.fft.rfft(y, n=8 * len(y)))
            f = np.fft.rfftfreq(8 * len(y), 0.01)
            sel = f > 0.2
            pk, _ = signal.find_peaks(F[sel], height=0.08 * F[sel].max())
            order = np.argsort(F[sel][pk])[::-1][:3]
            res[k + "_peaks_hz"] = sorted(float(f[sel][pk][i]) for i in order)
        out.append(res)
    return out


# ------------------------------------------------------------------ frequency response from dwells
def dwell_response(run):
    pts = []
    c1 = run.ch[0]
    for s in run.segments:
        p = s["params"]
        if p.get("signal") != "dwell" or "f_hz" not in p:
            continue
        runs = [iv for iv in run.intervals if iv["seg"] == s["seg"] and iv["phase"] == "run"]
        if not runs:
            continue
        iv = max(runs, key=lambda r: r["t1"] - r["t0"])
        t0, t1 = iv["t0"] + p.get("ramp_s", 1.0) + 3.0, iv["t1"] - p.get("ramp_s", 1.0)
        if t1 - t0 < 2.0:
            continue
        f = p["f_hz"]
        w = 2 * math.pi * f
        m = (c1["t"] >= t0) & (c1["t"] <= t1)
        t = c1["t"][m]
        cols = [np.cos(w * t), np.sin(w * t), np.ones_like(t), t - t.mean()]
        fn = run.model_used.get("w0", 7.9) / (2 * math.pi) if run.arms == 1 else None
        freqs = [fn] if fn else list(run.model_used.get("f_modes", []))
        for fz in freqs:
            if fz and abs(fz - f) / f > 0.08:
                cols += [np.cos(2 * math.pi * fz * t), np.sin(2 * math.pi * fz * t)]
        A = np.column_stack(cols)
        cy, *_ = np.linalg.lstsq(A, c1["psi"][m], rcond=None)
        xc = run.cart_x(t)
        cx, *_ = np.linalg.lstsq(A[:, :4], xc, rcond=None)
        Y = complex(cy[0], -cy[1])
        X = complex(cx[0], -cx[1])
        Acc = -w * w * X
        if abs(Acc) < 1e-6:
            continue
        pts.append({"test": s["test"], "f_hz": f, "H": Y / Acc, "amp_deg": float(np.degrees(abs(Y))),
                    "accel_m_s2": float(abs(Acc))})
    return pts


def etfe(run, kinds=("excite",)):
    """Transfer function cart acceleration -> arm 1 angle over the chirp/multisine/random tests."""
    c1 = run.ch[0]
    segs = [s for s in run.segments if s["kind"] in kinds and s["params"].get("signal") in ("chirp", "multisine", "random")]
    Hs = []
    for s in segs:
        if not np.isfinite(s["t0"]):
            continue
        t = np.arange(s["t0"] + 0.5, s["t1"] - 0.5, 0.01)
        if len(t) < 1000:
            continue
        y = np.interp(t, c1["t"], c1["psi"])
        a = np.gradient(np.gradient(run.cart_x(t), 0.01), 0.01)
        f, Pya = signal.csd(a, y, fs=100, nperseg=1024)
        _, Paa = signal.welch(a, fs=100, nperseg=1024)
        _, coh = signal.coherence(a, y, fs=100, nperseg=1024)
        Hs.append((f, Pya / Paa, coh, s["test"]))
    return Hs


# ------------------------------------------------------------------ physical values and twin settings
def rig_values(run, args):
    rig = dict(run.meta.get("rig_measurements") or {})
    over = {"arm1_mass_kg": args.arm1_mass, "arm1_pivot_to_com_m": args.arm1_com, "arm1_pivot_to_joint_m": args.arm1_joint,
            "arm2_mass_kg": args.arm2_mass, "arm2_joint_to_com_m": args.arm2_com}
    for k, v in over.items():
        if v is not None:
            rig[k] = v
    return rig


def physical_single(dec, cart, rig):
    w0 = dec["w0"] if dec else cart["w0"]
    m = rig.get("arm1_mass_kg")
    c = rig.get("arm1_pivot_to_com_m")
    src = "measured"
    if not m or not c:
        m, c, src = NOMINAL["arm1"]["m"], NOMINAL["arm1"]["c"], "not measured, so the PythonMasterMath2.py values"
    J = m * c * G / w0 ** 2
    I = J - m * c * c
    out = {"mass_kg": m, "com_m": c, "mass_com_source": src, "J_pivot_kg_m2": J, "I_com_kg_m2": I,
           "L_equiv_m": G / w0 ** 2}
    if dec:
        out.update({"b_viscous_N_m_s": dec["cv"] * J, "tau_dry_N_m": dec["cc"] * J, "c_air_N_m_s2": dec["cq"] * J,
                    "b_equivalent_viscous_N_m_s": dec["cv_viscous_only"] * J})
    if I <= 0:
        out["warning"] = ("mass x CoM distance is too small for this swing frequency (inertia about the CoM came out "
                          "negative): re-measure the mass and the balance point")
    return out


def twin_settings_single(run, dec, phys, sens):
    L = (run.meta.get("rig_measurements") or {}).get("arm1_pivot_to_tip_m") or NOMINAL["arm1"]["L"]
    s = {"arms": 1, "arm1": {"m": round(phys["mass_kg"], 5), "c": round(phys["com_m"], 5),
                             "I": float(f"{max(phys['I_com_kg_m2'], 1e-7):.6g}"), "L": L}}
    if dec:
        s["arm1"]["b"] = float(f"{phys['b_equivalent_viscous_N_m_s']:.4g}")
    enc = {"hang1": round(run.hang1, 1)}
    so = sens.get("still_off", {}).get("tx1") or sens.get("still_on", {}).get("tx1")
    if so:
        enc["noise"] = round(math.sqrt(max(so["std_counts"] ** 2 - 1 / 12, 0.0)), 3)   # the twin rounds to counts itself
    s["enc"] = enc
    return s


def physical_double(fit, rig):
    J_note = []
    m1 = rig.get("arm1_mass_kg") or NOMINAL["arm1"]["m"]
    m2 = rig.get("arm2_mass_kg") or NOMINAL["arm2"]["m"]
    c2 = rig.get("arm2_joint_to_com_m") or NOMINAL["arm2"]["c"]
    L1m = rig.get("arm1_pivot_to_joint_m")
    if not rig.get("arm2_mass_kg") or not rig.get("arm2_joint_to_com_m"):
        J_note.append("arm 2 mass and CoM not measured: PythonMasterMath2 values set the absolute scale")
    p2 = m2 * c2
    J1 = p2 / fit["P2"]
    p1, J2, h = fit["P1"] * J1, fit["J2r"] * J1, fit["H"] * J1
    L1_fit = h / p2
    L1 = L1m or L1_fit
    c1 = (p1 - m2 * L1) / m1
    I1 = J1 - m1 * c1 * c1 - m2 * L1 * L1
    I2 = J2 - m2 * c2 * c2
    out = {"J1_kg_m2": J1, "p1_kg_m": p1, "p2_kg_m": p2, "J2_kg_m2": J2, "h_kg_m2": h,
           "L1_from_fit_m": L1_fit, "L1_used_m": L1, "arm1": {"m": m1, "c": c1, "I": I1, "L": L1, "b": fit["B1"] * J1},
           "arm2": {"m": m2, "c": c2, "I": I2, "b": fit["B2"] * J1}, "notes": J_note,
           "dry_friction_N_m": [fit.get("C1", 0.0) * J1, fit.get("C2", 0.0) * J1]}
    if fit.get("C1") or fit.get("C2"):
        vo = fit["friction_viscous_only"]
        out["arm1"]["b"], out["arm2"]["b"] = vo["B1"] * J1, vo["B2"] * J1
        out["notes"].append("the pivots also show dry friction (see the table); the twin's b values are the "
                            "viscous-only fit")
    if c1 <= 0 or I1 <= 0 or I2 <= 0:
        out["notes"].append("a centre of mass or inertia came out non-physical: measure the masses, the arm 2 "
                            "balance point and the pivot-to-joint distance, then run the analysis again")
    if L1m and abs(L1_fit / L1m - 1) > 0.1:
        out["notes"].append(f"the fit puts arm 2's joint {L1_fit * 1000:.0f} mm from the pivot; you measured "
                            f"{L1m * 1000:.0f} mm")
    return out


def twin_settings_double(run, phys, sens):
    a1, a2 = phys["arm1"], phys["arm2"]
    L2 = (run.meta.get("rig_measurements") or {}).get("arm2_joint_to_tip_m") or NOMINAL["arm2"]["L"]
    s = {"arms": 2,
         "arm1": {"m": round(a1["m"], 5), "c": round(a1["c"], 5), "I": float(f"{max(a1['I'], 1e-7):.6g}"),
                  "L": round(a1["L"], 5), "b": float(f"{a1['b']:.4g}")},
         "arm2": {"m": round(a2["m"], 5), "c": round(a2["c"], 5), "I": float(f"{max(a2['I'], 1e-7):.6g}"),
                  "L": L2, "b": float(f"{a2['b']:.4g}")},
         "enc": {"hang1": round(run.hang1, 1), "aligned2": round(run.aligned2, 1)}}
    so = sens.get("still_off", {}) or sens.get("still_on", {})
    if so.get("tx1"):
        sd = float(np.mean([so[k]["std_counts"] for k in ("tx1", "tx2") if k in so]))
        s["enc"]["noise"] = round(math.sqrt(max(sd ** 2 - 1 / 12, 0.0)), 3)
    return s


# ------------------------------------------------------------------ figures
def fig_overview(run, path):
    c1 = run.ch[0]
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    if run.arms == 2 and 1 in run.ch:
        c2 = run.ch[1]
        p2 = np.interp(c2["t"], c1["t"], c1["psi"]) + c2["joint"]
        ax[0].plot(c2["t"], np.degrees(p2), color=C_MODEL, lw=0.7, label="arm 2 (absolute)", zorder=2)
    ax[0].plot(c1["t"], np.degrees(c1["psi"]), color=C_MEAS, lw=0.7, label="arm 1", zorder=3)
    if run.arms == 2:
        h, lab = ax[0].get_legend_handles_labels()
        ax[0].legend(h[::-1], lab[::-1], loc="upper right", ncol=2)
    ax[0].set_ylabel("angle from hanging, deg")
    t = np.linspace(run.c_t[0], run.c_t[-1], 6000)
    ax[1].plot(t, run.cart_x(t) * 1000, color=C_THIRD, lw=0.8)
    ax[1].set_ylabel("cart, mm")
    ax[1].set_xlabel("time, s")
    for s in run.segments:
        if not np.isfinite(s["t0"]) or s["kind"] in ("settle",):
            continue
        for a in ax:
            a.axvspan(s["t0"], s["t1"], color="#000000", alpha=0.035, lw=0)
        if s["t1"] - s["t0"] < 2.5:
            continue
        ax[0].text((s["t0"] + s["t1"]) / 2, 1.01, s["test"].replace("_", " "), transform=ax[0].get_xaxis_transform(),
                   ha="center", va="bottom", fontsize=7, color=INK2, rotation=60)
    ax[0].set_title(f"{run.name}", loc="left", pad=46)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_decays(run, dec, path):
    tr = dec["_arrays"]["traces"]
    n = len(tr)
    fig, ax = plt.subplots(n, 1, figsize=(11, 2.1 * n + 0.6), squeeze=False)
    for j, d in enumerate(tr):
        a = ax[j, 0]
        t = d["t"]
        k0 = 0
        sim = sim_single_free(dec["w0"], dec["cv"], dec["cc"], dec["cq"], d["ye"][k0] - d["off"], 0.0,
                              t[t >= d["te"][k0]])
        tt = t[t >= d["te"][k0]]
        a.plot(t - d["t0"], np.degrees(d["y"] - d["off"]), color=C_MEAS, lw=0.8, label="measured")
        a.plot(tt - d["t0"], np.degrees(sim), color=C_MODEL, lw=0.8, ls=(0, (4, 2)), label="model from the first peak")
        err = np.degrees(np.sqrt(np.mean((sim - (d["y"][t >= d["te"][k0]] - d["off"])) ** 2)))
        a.set_title(f"{d['test']}: free decay from {np.degrees(d['A'][0]):.0f} deg   (model error {err:.2f} deg rms)",
                    loc="left", fontsize=9)
        a.set_ylabel("deg")
        if j == 0:
            a.legend(loc="upper right", ncol=2)
    ax[-1, 0].set_xlabel("time since the cart stopped, s")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_period_damping(dec, path):
    arr = dec["_arrays"]
    A, Th, dE, use = arr["Amid"], arr["Th"], arr["dE"], arr["use"]
    w0 = dec["w0"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    a = ax[0]
    a.plot(np.degrees(A[use]), 2 * Th[use], "o", ms=3, color=C_MEAS, alpha=0.6, label="measured half swings x 2")
    Ag = np.linspace(max(A[use].min(), 1e-3), A[use].max(), 200)
    a.plot(np.degrees(Ag), 2 * ell_half_period_factor(Ag) / w0, color=C_MODEL, label=f"pendulum, w0 = {w0:.3f} rad/s")
    a.axhline(2 * math.pi / w0, color=INK2, lw=0.8, ls=":", label=f"small-swing period {2 * math.pi / w0:.4f} s")
    a.set_xlabel("swing amplitude, deg")
    a.set_ylabel("period, s")
    a.set_title("Period vs amplitude", loc="left")
    a.legend(loc="upper left")
    a = ax[1]
    # amplitude lost per full cycle, in degrees, against amplitude
    loss_deg = np.degrees(np.arccos(np.clip(np.cos(A) + dE / w0 ** 2, -1, 1)) - A) * -1
    a.plot(np.degrees(A[use]), 2 * loss_deg[use], "o", ms=3, color=C_MEAS, alpha=0.6, label="measured")
    parts = [("viscous", dec["cv"] * visc_integral(Ag) * w0, C_THIRD),
             ("dry (Coulomb)", dec["cc"] * 2 * Ag, C_FOURTH),
             ("air", dec["cq"] * w0 ** 2 * quad_integral(Ag), "#eda100")]
    tot = sum(p[1] for p in parts)

    def to_deg(dEg):
        return 2 * np.degrees(Ag - np.arccos(np.clip(np.cos(Ag) + dEg / w0 ** 2, -1, 1)))
    a.plot(np.degrees(Ag), to_deg(tot), color=C_MODEL, label="fit: total")
    for name, part, col in parts:
        if np.max(part) > 0:
            a.plot(np.degrees(Ag), to_deg(part), color=col, lw=1.0, ls=(0, (4, 2)), label=name)
    a.set_xlabel("swing amplitude, deg")
    a.set_ylabel("amplitude lost per cycle, deg")
    a.set_title("Friction: swing lost per cycle", loc="left")
    a.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_windows(run, fitres, path, double=False):
    if double:
        win, o1, o2 = fitres["_win"]
    else:
        win, g = fitres["_win"]
    # pick up to 8 windows with the most motion, from different tests
    c1 = run.ch[0]
    scores = []
    for w in range(win.W):
        m = (c1["t"] >= win.t0[w]) & (c1["t"] <= win.t0[w] + win.length)
        scores.append(np.ptp(c1["psi"][m]) if m.any() else 0)
    best = {}
    for w in np.argsort(scores)[::-1]:
        iv = [(s["test"], s["kind"], s["params"].get("signal", s["kind"])) for s in run.segments
              if np.isfinite(s["t0"]) and s["t0"] <= win.t0[w] <= s["t1"]]
        if not iv:
            continue
        tname, kind, group = iv[0]
        if group not in best:
            best[group] = (w, tname)
    order = ["moves", "pump_decay", "mode_decay", "chirp", "dwell", "multisine", "random", "settle"]
    chosen = [best[g] for g in order if g in best][:8]
    rows = (len(chosen) + 1) // 2
    fig, ax = plt.subplots(rows, 2, figsize=(11, 2.0 * rows + 0.6), squeeze=False)
    tg = np.arange(win.N + 1) * DT_SIM
    for j, (w, tname) in enumerate(chosen):
        a = ax[j // 2, j % 2]
        m = (c1["t"] >= win.t0[w]) & (c1["t"] <= win.t0[w] + win.length)
        a.plot(c1["t"][m] - win.t0[w], np.degrees(c1["psi"][m]), color=C_MEAS, lw=1.0, label="arm 1 measured")
        if double:
            a.plot(tg, np.degrees(o1[w]), color=C_MODEL, lw=1.0, ls=(0, (4, 2)), label="arm 1 model")
            c2 = run.ch[1]
            m2 = (c2["t"] >= win.t0[w]) & (c2["t"] <= win.t0[w] + win.length)
            a.plot(c2["t"][m2] - win.t0[w], np.degrees(c2["joint"][m2]), color=C_THIRD, lw=1.0, label="joint measured")
            a.plot(tg, np.degrees(o2[w] - o1[w]), color=C_FOURTH, lw=1.0, ls=(0, (4, 2)), label="joint model")
        else:
            a.plot(tg, np.degrees(g[w]), color=C_MODEL, lw=1.0, ls=(0, (4, 2)), label="model")
        a.set_title(f"{tname} at {win.t0[w]:.1f} s", loc="left", fontsize=9)
        a.set_ylabel("deg")
        if j == 0:
            a.legend(loc="upper right", fontsize=7, ncol=2)
    for j in range(len(chosen), rows * 2):
        ax[j // 2, j % 2].axis("off")
    fig.suptitle("Model vs measurement: each window starts from the measured state and is driven by the logged cart motion",
                 x=0.01, ha="left", fontsize=9, color=INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def wrap_phase(H):
    return (np.degrees(np.angle(H)) + 90.0) % 360.0 - 90.0


def fig_freq_response(run, pts, Hs, model_fn, path):
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fmin, fmax = 0.2, 5.0
    for f, H, coh, name in Hs:
        sel = (f >= fmin) & (f <= fmax) & (coh > 0.8)
        if sel.sum() < 3:
            continue
        ax[0].semilogy(f[sel], np.degrees(np.abs(H[sel])), ".", ms=2.5, color=C_THIRD, alpha=0.6)
        ax[1].plot(f[sel], wrap_phase(H[sel]), ".", ms=2.5, color=C_THIRD, alpha=0.6)
    if Hs:
        ax[0].plot([], [], ".", color=C_THIRD, label="chirp / multisine / random (coherence > 0.8)")
    if pts:
        f = np.array([p["f_hz"] for p in pts])
        H = np.array([p["H"] for p in pts])
        ax[0].semilogy(f, np.degrees(np.abs(H)), "o", ms=6, color=C_MEAS, label="sine dwells")
        ax[1].plot(f, wrap_phase(H), "o", ms=6, color=C_MEAS)
    fg = np.linspace(fmin, fmax, 800)
    Hm = model_fn(fg)
    ax[0].semilogy(fg, np.degrees(np.abs(Hm)), color=C_MODEL, label="fitted model")
    ax[1].plot(fg, wrap_phase(Hm), ".", ms=1.2, color=C_MODEL)
    ax[0].set_ylabel("arm 1 deg per m/s^2")
    ax[1].set_ylabel("phase, deg")
    ax[1].set_xlabel("frequency, Hz")
    ax[0].set_title("Cart acceleration to arm 1 angle", loc="left")
    ax[0].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_timing(run, path):
    fig, ax = plt.subplots(2, 2, figsize=(11, 5.5))
    data = [(run.sm["cycle_ms"], "control cycle period, ms"), (run.reply_ms, "Arduino write-to-reply, ms"),
            (run.frame_latency_ms, "angle frame transport beyond the fastest, ms"), (run.ch[0]["age"], "TX1 reading age at the receiver, ms")]
    for a, (d, lab) in zip(ax.flat, data):
        d = np.asarray(d, float)
        d = d[np.isfinite(d)]
        if len(d):
            hi = np.percentile(d, 99.9)
            a.hist(np.clip(d, None, hi), bins=80, color=C_MEAS, alpha=0.85, rwidth=0.9)
            a.set_yscale("log")
            a.set_title(f"{lab}   median {np.median(d):.2f}, 99% {np.percentile(d, 99):.2f}, max {d.max():.2f}", loc="left", fontsize=8.5)
        a.set_ylabel("count")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_noise(run, path):
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    for a, name in zip(ax, ("still_off", "still_on")):
        w = run.still_window(name)
        if not w:
            a.axis("off")
            continue
        for i, col in ((0, C_MEAS), (1, C_MODEL)):
            if i not in run.ch or (run.arms == 1 and i == 1):
                continue
            c = run.ch[i]
            m = (c["t"] >= w[0] + 0.2) & (c["t"] <= w[1])
            if m.sum() < 20:
                continue
            raw = c["raw"][m]
            dev = wrap_counts(raw - np.round(np.median(raw)))
            vals, cnt = np.unique(dev, return_counts=True)
            off = -0.18 if i == 0 else 0.18
            a.bar(vals + (off if run.arms == 2 else 0), cnt, width=0.34 if run.arms == 2 else 0.6, color=col,
                  label=f"TX{i + 1}: std {np.std(dev):.2f} counts")
        a.set_title(f"{name.replace('_', ' ')}: encoder counts around the median", loc="left")
        a.set_xlabel("counts (1 count = 0.022 deg)")
        a.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_motor(run, path):
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    r = run.step_resid
    r = r[np.isfinite(r)]
    ax[0].hist(np.clip(r, -6, 6), bins=np.arange(-6.5, 7, 0.25), color=C_MEAS, rwidth=0.9)
    ax[0].set_yscale("log")
    ax[0].set_title(f"Reported step count minus commanded (per command): std {np.std(r):.2f} steps", loc="left", fontsize=9)
    ax[0].set_xlabel("steps")
    v = run.c_v
    a = np.diff(v) / np.maximum(np.diff(run.c_t), 1e-3)
    ax[1].plot(v[1:], a, ".", ms=1.5, color=C_MEAS, alpha=0.4)
    ax[1].set_xlabel("cart speed commanded, m/s")
    ax[1].set_ylabel("acceleration commanded, m/s^2")
    ax[1].set_title("Speeds and accelerations the run covered", loc="left", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ report
def fmt(v, sd=None, nd=4):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "-"
    s = f"{v:.{nd}g}"
    if sd is not None and np.isfinite(sd):
        s += f" ± {sd:.2g}"
    return s


def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if not k.startswith("_")}
    if isinstance(o, (list, tuple)):
        return [strip(v) for v in o]
    if isinstance(o, complex):
        return {"re": o.real, "im": o.imag}
    if isinstance(o, (np.floating, np.integer)):
        o = o.item()
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, np.ndarray):
        return o.tolist()
    return o


def analyze(folder, args):
    run = Run(folder)
    out_dir = run.dir / "analysis"
    out_dir.mkdir(exist_ok=True)
    print(f"\n=== {run.name}  ({'double' if run.arms == 2 else 'single'}, {run.meta.get('plan')} plan, seed {run.meta.get('seed')})")
    res = {"run": run.name, "arms": run.arms, "plan": run.meta.get("plan"), "seed": run.meta.get("seed"),
           "started": run.meta.get("started_local"), "stop_reason": run.meta.get("summary", {}).get("stop_reason")}
    res["timing"] = timing_stats(run)
    res["sensors"] = sensor_stats(run)
    res["motor"] = {"step_resid_std": float(np.std(run.step_resid)), "step_resid_max_abs": float(np.max(np.abs(run.step_resid))),
                    "max_speed_m_s": float(np.max(np.abs(run.c_v))),
                    "max_accel_m_s2": float(np.max(np.abs(np.diff(run.c_v) / np.maximum(np.diff(run.c_t), 1e-3))))}
    if run.meta.get("end_check"):
        res["motor"]["end_check"] = run.meta["end_check"]
    rig = rig_values(run, args)
    res["rig_measurements_used"] = rig
    fig_overview(run, out_dir / "01_overview.png")
    figs = ["01_overview.png"]
    w0_guess = float(run.model_used.get("w0", 7.9)) if run.arms == 1 else 2 * math.pi * float((run.model_used.get("f_modes") or [1.0])[0])
    if run.arms == 1:
        dec = decay_analysis_single(run, w0_guess)
        if dec:
            print(f"decays: w0 = {dec['w0']:.4f} rad/s (period {dec['T0_s']:.4f} s), viscous {dec['cv']:.4g} 1/s, "
                  f"dry {dec['cc']:.4g} rad/s^2, air {dec['cq']:.4g} 1/rad")
            res["decay"] = dec
            fig_decays(run, dec, out_dir / "02_decays.png")
            fig_period_damping(dec, out_dir / "03_period_friction.png")
            figs += ["02_decays.png", "03_period_friction.png"]
        else:
            print("decays: not enough free swinging to analyse")
        k0 = float(run.model_used.get("k", 6.4)) if run.model_used.get("k") else (dec["w0"] if dec else w0_guess) ** 2 / G
        cart = fit_cart_single(run, dec, dec["w0"] if dec else w0_guess, k0)
        if cart:
            print(f"cart fit: k = {cart['k']:.4f} rad/m, cart scale {cart['cart_scale']:.4f}, delay {cart['delay_ms']:.1f} ms, "
                  f"w0 {cart['w0']:.4f}, error {cart['rms_deg']:.2f} deg rms over {cart['windows']} windows "
                  f"({cart['seconds']} s)")
            res["cart_fit"] = cart
            fig_windows(run, cart, out_dir / "04_model_windows.png")
            figs.append("04_model_windows.png")
        w0 = dec["w0"] if dec else (cart["w0"] if cart else w0_guess)
        k = cart["k"] if cart else w0 ** 2 / G
        d = (cart["delay_ms"] / 1000.0) if cart else 0.0
        cv = dec["cv_viscous_only"] if dec else 0.05

        def model_fn(f):
            w = 2 * np.pi * f
            return -k * np.exp(-1j * w * d) / (w0 ** 2 - w ** 2 + 1j * cv * w)
        phys = physical_single(dec, cart, rig) if (dec or cart) else None
        res["physical"] = phys
        if phys:
            res["twin_rig_settings"] = twin_settings_single(run, dec, phys, res["sensors"])
    else:
        dfit = fit_double(run)
        res["decay_spectra"] = decay_spectra_double(run)
        if dfit:
            print(f"double fit: modes {dfit['mode_freq_hz'][0]:.3f} / {dfit['mode_freq_hz'][1]:.3f} Hz, cart scale "
                  f"{dfit['cart_scale']:.3f}, delay {dfit['delay_ms']:.1f} ms, error {dfit['rms_deg_arm1']:.2f} / "
                  f"{dfit['rms_deg_joint']:.2f} deg ({dfit['seconds']} s)")
            res["double_fit"] = dfit
            fig_windows(run, dfit, out_dir / "04_model_windows.png", double=True)
            figs.append("04_model_windows.png")
            phys = physical_double(dfit, rig)
            res["physical"] = phys
            res["twin_rig_settings"] = twin_settings_double(run, phys, res["sensors"])
            P1, P2, J2r, H, B1, B2, sa = (dfit[n] for n in DOUBLE_NAMES[:7])
            dd = dfit["delay_ms"] / 1000.0

            def model_fn(f):
                out = []
                M = np.array([[1, H], [H, J2r]])
                K = G * np.diag([P1, P2])
                Bm = np.array([[B1 + B2, -B2], [-B2, B2]])
                Pa = sa * np.array([P1, P2])
                for fi in f:
                    w = 2 * np.pi * fi
                    out.append(np.linalg.solve(-w * w * M + 1j * w * Bm + K, -Pa)[0] * np.exp(-1j * w * dd))
                return np.array(out)
        else:
            model_fn = None
    pts = dwell_response(run)
    Hs = etfe(run)
    res["dwell_points"] = [{"test": p["test"], "f_hz": p["f_hz"], "gain_deg_per_m_s2": float(np.degrees(abs(p["H"]))),
                            "phase_deg": float(np.degrees(np.angle(p["H"]))), "amp_deg": p["amp_deg"]} for p in pts]
    if model_fn is not None and (pts or Hs):
        fig_freq_response(run, pts, Hs, model_fn, out_dir / "05_frequency_response.png")
        figs.append("05_frequency_response.png")
    fig_timing(run, out_dir / "06_timing.png")
    fig_noise(run, out_dir / "07_sensor_noise.png")
    fig_motor(run, out_dir / "08_motor.png")
    figs += ["06_timing.png", "07_sensor_noise.png", "08_motor.png"]
    clean = strip(res)
    with open(out_dir / "results.json", "w", encoding="utf-8") as f:
        json.dump(clean, f, indent=1)
    if "twin_rig_settings" in res:
        with open(out_dir / "twin_rig_settings.json", "w", encoding="utf-8") as f:
            json.dump(res["twin_rig_settings"], f, indent=1)
    write_report(run, res, figs, out_dir / "report.md")
    print(f"wrote {out_dir}")
    return clean


def write_report(run, res, figs, path):
    L = []
    t = res["timing"]
    L.append(f"# System identification: {run.name}\n")
    L.append(f"{'Double' if run.arms == 2 else 'Single'} pendulum, {res['plan']} plan, seed {res['seed']}, started {res['started']}. "
             f"Stopped: {res['stop_reason']}.\n")
    if run.meta.get("run_note"):
        L.append(f"Note: {run.meta['run_note']}\n")
    L.append("![overview](01_overview.png)\n")
    if run.arms == 1:
        dec, cart, ph = res.get("decay"), res.get("cart_fit"), res.get("physical")
        L.append("## Arm\n")
        L.append("| Quantity | Value | How it was found |\n| --- | --- | --- |")
        if dec:
            L.append(f"| Natural frequency w0 | {fmt(dec['w0'], dec['w0_sd'], 6)} rad/s ({dec['f0_hz']:.4f} Hz) | "
                     f"{dec['half_swings']} half swings in {dec['decays']} free decays, {dec['amp_range_deg'][0]:.0f}-{dec['amp_range_deg'][1]:.0f} deg, exact pendulum period |")
            L.append(f"| Small-swing period | {dec['T0_s']:.5f} s | 2 pi / w0 |")
            L.append(f"| Equivalent simple-pendulum length | {dec['L_equiv_m'] * 1000:.2f} mm | g / w0^2 = J_pivot / (m c) |")
            nd = "not detected (a model without it fits as well)"
            L.append(f"| Friction model | {dec['best_by_aic']} | the simplest of viscous / +dry / +air that fits the energy lost per swing (AIC) |")
            L.append(f"| Viscous friction cv | {fmt(dec['cv'], dec['cv_sd'])} 1/s | |")
            L.append(f"| Dry (Coulomb) friction cc | {fmt(dec['cc'], dec['cc_sd']) + ' rad/s^2' if dec['cc'] > 0 else nd} | |")
            L.append(f"| Air drag cq | {fmt(dec['cq'], dec['cq_sd']) + ' 1/rad' if dec['cq'] > 0 else nd} | |")
            if dec["best_by_aic"] != "viscous":
                L.append(f"| Viscous-only equivalent | {fmt(dec['cv_viscous_only'], dec['cv_viscous_only_sd'])} 1/s | for models with one friction term |")
        if cart:
            L.append(f"| Cart coupling k | {fmt(cart['k'], cart['k_sd'])} rad/m | arm angular acceleration per cart acceleration; windowed simulation of {cart['windows']} x 2 s, {cart['rms_deg']:.2f} deg rms error ({cart['fit_pct']:.1f}% fit) |")
            L.append(f"| Cart scale k g / w0^2 | {cart['cart_scale']:.4f} ± {cart['cart_scale_sd']:.4f} | 1.0000 = M_PER_STEP right and the arm rigid |")
            L.append(f"| Delay, step count to arm | {fmt(cart['delay_ms'], cart['delay_sd_ms'], 3)} ms | same fit (after the angle readings' own ages) |")
            L.append(f"| w0 from the cart fit | {fmt(cart['w0'], cart['w0_sd'], 6)} rad/s | should match the decays |")
        L.append("")
        if dec:
            L.append("Model: psi'' = -w0^2 sin psi - k cos psi * a_cart(t - delay) - cv psi' - cc sign(psi') - cq psi'|psi'|, "
                     "psi from hanging.\n")
        if ph:
            L.append("## Physical values\n")
            L.append(f"Mass and centre of mass: {ph['mass_kg']} kg, {ph['com_m']} m ({ph['mass_com_source']}).\n")
            L.append("| Quantity | Value |\n| --- | --- |")
            L.append(f"| Inertia about the pivot J | {ph['J_pivot_kg_m2']:.6g} kg m^2 |")
            L.append(f"| Inertia about the CoM I | {ph['I_com_kg_m2']:.6g} kg m^2 |")
            if "b_viscous_N_m_s" in ph:
                L.append(f"| Viscous friction b | {ph['b_viscous_N_m_s']:.4g} N m s/rad |")
                if ph["tau_dry_N_m"] > 0:
                    L.append(f"| Dry friction torque | {ph['tau_dry_N_m']:.4g} N m |")
                if ph["c_air_N_m_s2"] > 0:
                    L.append(f"| Air drag | {ph['c_air_N_m_s2']:.4g} N m s^2/rad^2 |")
                if ph["tau_dry_N_m"] > 0 or ph["c_air_N_m_s2"] > 0:
                    L.append(f"| Viscous-only equivalent b | {ph['b_equivalent_viscous_N_m_s']:.4g} N m s/rad |")
            if ph.get("warning"):
                L.append(f"\n**{ph['warning']}**")
            L.append("")
        for fg in ("02_decays.png", "03_period_friction.png", "04_model_windows.png"):
            if fg in figs:
                L.append(f"![{fg}]({fg})\n")
    else:
        df, ph = res.get("double_fit"), res.get("physical")
        L.append("## Arms\n")
        if df:
            L.append(f"Fitted by simulating {df['windows']} windows of 1.5 s from the measured state, driven by the logged cart motion: "
                     f"{df['rms_deg_arm1']:.2f} deg rms error on arm 1 ({df['fit_pct_arm1']:.1f}% fit), "
                     f"{df['rms_deg_joint']:.2f} deg on the joint ({df['fit_pct_joint']:.1f}% fit).\n")
            L.append("| Parameter | Value | Meaning |\n| --- | --- | --- |")
            meaning = {"P1": "(m1 c1 + m2 L1) / J1, 1/m", "P2": "m2 c2 / J1, 1/m", "J2r": "J2 / J1 (J2 about the joint)",
                       "H": "L1 m2 c2 / J1", "B1": "pivot friction / J1, 1/s", "B2": "joint friction / J1, 1/s",
                       "cart_scale": "1.00 = M_PER_STEP right", "delay_ms": "step count to arms, ms"}
            for n in DOUBLE_NAMES:
                L.append(f"| {n} | {fmt(df[n], df[n + '_sd'])} | {meaning[n]} |")
            L.append(f"| Mode 1 | {df['mode_freq_hz'][0]:.4f} Hz, damping ratio {df['mode_damping_ratio'][0]:.4f} | linearised |")
            L.append(f"| Mode 2 | {df['mode_freq_hz'][1]:.4f} Hz, damping ratio {df['mode_damping_ratio'][1]:.4f} | linearised |")
            L.append("")
            if res.get("decay_spectra"):
                L.append("Peak frequencies during the free decays: " + "; ".join(
                    f"{d['test']}: arm 1 {', '.join(f'{x:.3f}' for x in d.get('arm1_peaks_hz', []))} Hz, joint "
                    f"{', '.join(f'{x:.3f}' for x in d.get('joint_peaks_hz', []))} Hz" for d in res["decay_spectra"]) + "\n")
        if ph:
            L.append("## Physical values\n")
            L.append("| Quantity | Arm 1 | Arm 2 |\n| --- | --- | --- |")
            a1, a2 = ph["arm1"], ph["arm2"]
            L.append(f"| Mass, kg | {a1['m']:.4g} | {a2['m']:.4g} |")
            L.append(f"| CoM from pivot / joint, m | {a1['c']:.4g} | {a2['c']:.4g} |")
            L.append(f"| Inertia about the CoM, kg m^2 | {a1['I']:.4g} | {a2['I']:.4g} |")
            L.append(f"| Friction b, N m s/rad | {a1['b']:.3g} | {a2['b']:.3g} |")
            L.append(f"| Pivot to joint, m | {a1['L']:.4g} (fit says {ph['L1_from_fit_m']:.4g}) | |")
            for n in ph["notes"]:
                L.append(f"\n**{n}**")
            L.append("")
        if "04_model_windows.png" in figs:
            L.append("![windows](04_model_windows.png)\n")
    if "05_frequency_response.png" in figs:
        L.append("## Frequency response\n")
        L.append("![frequency response](05_frequency_response.png)\n")
    if res.get("twin_rig_settings"):
        L.append("## Digital twin settings\n")
        L.append("Command line: `python run_twin.py <folder> <script> --rig \"$(cat twin_rig_settings.json)\"` "
                 "(or paste the JSON in quotes). In the browser, type these into Rig settings:\n")
        ts = res["twin_rig_settings"]
        labels = {"m": "Mass, kg", "c": "CoM from pivot, m", "I": "Inertia about CoM, kg·m²", "L": "Length, m",
                  "b": "Pivot friction, N·m·s"}
        for arm in ("arm1", "arm2"):
            if arm in ts:
                L.append(f"- {'Arm 1' if arm == 'arm1' else 'Arm 2'}: " + ", ".join(f"{labels[k]} = {v}" for k, v in ts[arm].items()))
        if "enc" in ts:
            L.append(f"- Encoders: hanging counts {ts['enc'].get('hang1')}"
                     + (f", arm 2 aligned counts {ts['enc']['aligned2']}" if "aligned2" in ts["enc"] else "")
                     + (f", noise {ts['enc']['noise']} counts" if "noise" in ts["enc"] else ""))
        L.append("\n```json\n" + json.dumps(ts, indent=1) + "\n```\n")
        if run.arms == 1 and res.get("decay") and res["decay"]["best_by_aic"] != "viscous":
            L.append("The twin has viscous pivot friction only, so it gets the viscous-only equivalent. The dry and air "
                     "terms above are the better description of the real pivot.\n")
    L.append("## Timing and sensors\n")
    L.append("| Quantity | Median | 99% | Max |\n| --- | --- | --- | --- |")
    L.append(f"| Control cycle, ms | {t['cycle_ms']['p50']:.2f} | {t['cycle_ms']['p99']:.2f} | {t['cycle_ms']['max']:.2f} |")
    L.append(f"| Arduino write-to-reply, ms | {t['arduino_reply_ms']['p50']:.2f} | {t['arduino_reply_ms']['p99']:.2f} | {t['arduino_reply_ms']['max']:.2f} |")
    L.append(f"| Angle frame transport beyond the fastest, ms | {t['frame_transport_extra_ms']['p50']:.2f} | {t['frame_transport_extra_ms']['p99']:.2f} | {t['frame_transport_extra_ms']['max']:.2f} |")
    L.append(f"| TX1 reading age at the receiver, ms | {np.percentile(run.ch[0]['age'], 50):.1f} | {t['tx1_age_ms']['p99']:.1f} | {t['tx1_age_ms']['max']:.0f} |")
    L.append("")
    L.append(f"Frames {t['frames']}, missing {t['frame_gaps']}, CRC errors {t['crc_errors']}, TX1 not valid in "
             f"{t['tx1_not_valid_frames']}" + (f", TX2 not valid in {t['tx2_not_valid_frames']}" if run.arms == 2 else "")
             + f". Receiver clock runs {t['receiver_clock_drift_ppm']:+.0f} ppm against the Pi. Cycles that hit the "
             f"acceleration limit {t['cycles_a_clip']}, speed limit {t['cycles_v_clip']}, braking zone {t['cycles_brake']}, "
             f"stale angle {t['cycles_stale_angle']}.\n")
    for name, d in res["sensors"].items():
        if d:
            L.append(f"- {name.replace('_', ' ')}: " + "; ".join(
                f"{k.upper()} noise {v['std_counts']:.3f} counts ({v['std_deg'] * 1000:.1f} mdeg), {v['distinct_values']} distinct values, "
                f"mean {v['mean_counts']:.2f}" for k, v in d.items()))
    m = res["motor"]
    L.append(f"\nMotor: step count vs commands {m['step_resid_std']:.2f} steps std (max {m['step_resid_max_abs']:.1f}); "
             f"fastest {m['max_speed_m_s']:.2f} m/s, hardest {m['max_accel_m_s2']:.1f} m/s^2."
             + (f" End check: ruler {m['end_check'].get('measured_mm')} mm vs step count {m['end_check'].get('step_count_mm')} mm."
                if m.get("end_check") else " No end check entered (a ruler measurement there shows missed steps)."))
    L.append("")
    for fg in ("06_timing.png", "07_sensor_noise.png", "08_motor.png"):
        L.append(f"![{fg}]({fg})\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def summarize(results, path_dir):
    keys_single = [("decay", "w0"), ("decay", "cv"), ("decay", "cc"), ("decay", "cq"), ("decay", "cv_viscous_only"),
                   ("cart_fit", "k"), ("cart_fit", "cart_scale"), ("cart_fit", "delay_ms")]
    keys_double = [("double_fit", n) for n in DOUBLE_NAMES]
    out = {"runs": [r["run"] for r in results], "values": {}}
    lines = ["# Summary across runs\n"]
    for arms, keys in ((1, keys_single), (2, keys_double)):
        rs = [r for r in results if r["arms"] == arms]
        if not rs:
            continue
        lines += [f"## {'Single arm' if arms == 1 else 'Double arm'}: {len(rs)} run{'s' if len(rs) > 1 else ''}\n",
                  "Runs: " + ", ".join(r["run"] for r in rs) + "\n",
                  "| Value | Mean | Std | Runs |", "| --- | --- | --- | --- |"]
        for sec, k in keys:
            vals = [r[sec][k] for r in rs if sec in r and k in r[sec]]
            if not vals:
                continue
            out["values"][f"{sec}.{k}"] = {"mean": float(np.mean(vals)), "std": float(np.std(vals)), "n": len(vals), "all": vals}
            lines.append(f"| {sec}.{k} | {np.mean(vals):.5g} | {np.std(vals):.2g} | {len(vals)} |")
        tw = [r["twin_rig_settings"] for r in rs if "twin_rig_settings" in r]
        if tw:
            merged = json.loads(json.dumps(tw[0]))
            for arm in ("arm1", "arm2"):
                if arm in merged:
                    for k in merged[arm]:
                        merged[arm][k] = float(f"{np.median([t[arm][k] for t in tw]):.6g}")
            out[f"twin_rig_settings_{arms}arm"] = merged
            lines.append(f"\nMedian twin settings:\n\n```json\n{json.dumps(merged, indent=1)}\n```\n")
    with open(path_dir / "sysid_summary.json", "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    with open(path_dir / "sysid_summary.md", "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\nwrote {path_dir / 'sysid_summary.md'}")


def main():
    ap = argparse.ArgumentParser(description="Fit pendulum dynamics from PendulumSysID.py run folders.")
    ap.add_argument("runs", nargs="+", help="run folders (sysid_...)")
    ap.add_argument("--arm1-mass", type=float, help="kg, everything that swings with arm 1")
    ap.add_argument("--arm1-com", type=float, help="m, pivot to arm 1's centre of mass")
    ap.add_argument("--arm1-joint", type=float, help="m, pivot to arm 2's joint (double)")
    ap.add_argument("--arm2-mass", type=float, help="kg (double)")
    ap.add_argument("--arm2-com", type=float, help="m, joint to arm 2's centre of mass (double)")
    args = ap.parse_args()
    folders = [Path(p) for p in args.runs if (Path(p) / "metadata.json").exists()]
    if not folders:
        sys.exit("No run folders found (each needs metadata.json).")
    results = []
    for f in folders:
        try:
            results.append(analyze(f, args))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"!! {f}: {exc}")
    if len(results) > 1:
        parents = {f.resolve().parent for f in folders}
        summarize(results, parents.pop() if len(parents) == 1 else Path.cwd())


if __name__ == "__main__":
    main()
