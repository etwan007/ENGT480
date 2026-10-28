"""Monte Carlo validation of the controllers in the digital twin.

Every run draws its own twin: inertial constants within +/-2 sigma of the fit
(and the run-to-run difference seen between the 10/16 and 10/27 sysid days),
friction x0.5..x2, stepper servo bandwidth, sensor delay, frame loss, loop
jitter, encoder calibration offsets and the starting state.

Scenarios
  swingup     improved controller from hanging (planned swing-up + TVLQR)
  energy      improved controller with swing_mode="energy" (for comparison)
  balance     improved (conservative, nominal) vs the existing PythonMaster2_auto
              from a hand-held upright start, with an arm "tap" at t = 5 s
  latency     balance + swing-up success vs extra loop latency

    python3 monte_carlo.py [--runs 100] [--scenarios swingup balance latency energy]
Writes plots/monte_carlo_*.png and monte_carlo_results.json.
"""

import argparse
import json
import math
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for path in (HERE.parent, HERE, HERE.parent / "DoublePendulum" / "Improved"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_model import load_parameters, wrap_angle  # noqa: E402
from twin_sim import RigSimulator, SimConfig, hanging_state  # noqa: E402

RESULTS_PATH = HERE / "monte_carlo_results.json"


# Two uncertainty levels:
#   "fresh": the twin was re-identified the same day (sysid run + fit_parameters.py);
#            inertial constants within ~2x their single-run Jacobian sigma.
#   "stale": the spread actually seen between the 10/16 and 10/27 sysid runs
#            (arm 2 changed by 5-15 %), i.e. using an old fit.
LEVELS = {
    "fresh": dict(J1=0.01, J2=0.015, c=0.01, h2=0.015, fric=(0.7, 1.4), c2_alt=False, tau=(0.002, 0.015)),
    "stale": dict(J1=0.04, J2=0.06, c=0.04, h2=0.06, fric=(0.5, 2.5), c2_alt=True, tau=(0.002, 0.025)),
}


def random_params(rng, base, extra_delay=0.0, level="stale"):
    L = LEVELS[level]
    k = lambda s: float(rng.normal(1.0, s))
    f = lambda: float(rng.uniform(*L["fric"]))
    c2 = base.c2
    if L["c2_alt"] and rng.random() < 0.5:
        c2 = 0.0034                      # the 10/16 elbow had ~3.4 mN m Coulomb friction
    return base.with_updates(
        J1_eff=base.J1 * k(L["J1"]), J2_eff=base.J2 * k(L["J2"]), c_eff=base.c * k(L["c"]),
        h1_eff=base.h1, h2_eff=base.h2 * k(L["h2"]),
        b1=base.b1 * f() + 1e-5, c1=base.c1 * f(), b2=base.b2 * f() + 1e-5,
        c2=c2 * float(rng.uniform(0.7, 1.4)),
        motor_tau_s=float(rng.uniform(*L["tau"])),
        sensor_delay_s=float(rng.uniform(0.002, 0.008)) + extra_delay,
        packet_loss=float(rng.uniform(0.0, 0.01)),
        control_jitter_s=float(rng.uniform(0.0005, 0.002)),
        encoder_noise_counts=float(rng.uniform(0.2, 1.0)),
    )


def make_controller(kind, profile="conservative"):
    if kind == "legacy":
        from legacy_controller import LegacyController
        return LegacyController()
    from improved_controller import ImprovedController, ControllerConfig
    if kind == "improved_balance":
        cfg = ControllerConfig(balance_only=True)
    elif kind == "improved_energy":
        cfg = ControllerConfig(swing_mode="energy", max_attempts=1000, swing_timeout_s=1000)
    else:
        cfg = ControllerConfig(max_attempts=1000)
    return ImprovedController(config=cfg, profile=profile)


def one_run(job):
    scenario, kind, profile, seed, duration, extra_delay = job
    rng = np.random.default_rng(seed)
    level = "stale" if scenario.endswith("stale") or scenario == "balance" else "fresh"
    p = random_params(rng, load_parameters(), extra_delay, level)
    off1 = math.radians(rng.normal(0, 0.4))
    offj = math.radians(rng.normal(0, 0.4))
    cfg = SimConfig(duration_s=duration, seed=seed, offset1_rad=off1, offset_joint_rad=offj)
    ctrl = make_controller(kind, profile)
    if scenario in ("balance", "latency_balance"):
        x0 = np.array([rng.normal(0, math.radians(1.5)), rng.normal(0, 0.1),
                       rng.normal(0, math.radians(1.5)), rng.normal(0, 0.1), 0.0, 0.0])
    else:
        a = math.radians(rng.uniform(0, 3))
        x0 = hanging_state(x=rng.uniform(-0.01, 0.01), th1=math.pi + a, th2=math.pi - a)
    sim = RigSimulator(p, cfg)
    tap_t = 5.0 if scenario == "balance" else None
    tap = float(rng.choice([-1, 1]) * rng.uniform(0.3, 0.8))   # rad/s added to arm 2 (a light tap)

    class Tap:
        def __init__(self, inner):
            self.inner = inner
            self.done = False

        def update(self, obs):
            return self.inner.update(obs)

    r = sim.run(ctrl, x0) if tap_t is None else run_with_tap(sim, ctrl, x0, tap_t, tap)
    t = r["t"]
    m = r["mode"]
    th1 = wrap_angle(r["th1"])
    th2 = wrap_angle(r["th2"])
    upright = (np.abs(th1) < 0.35) & (np.abs(th2) < 0.35)
    tail = t > t[-1] - 5.0
    success = bool(len(t) and t[-1] > duration - 0.5 and upright[tail].all() and not r["limit_hit"]
                   and not r["stopped_by_controller"])
    # time to balance: start of the final continuous upright stretch
    ttb = None
    if success:
        bad = np.flatnonzero(~upright)
        ttb = float(t[bad[-1] + 1]) if len(bad) else 0.0
    settled = tail & upright
    return {
        "scenario": scenario, "kind": kind, "profile": profile, "seed": seed, "extra_delay": extra_delay,
        "success": success, "time_to_balance_s": ttb,
        "peak_cart_m": float(np.max(np.abs(r["x"]))) if len(t) else 0.0,
        "limit_hit": bool(r["limit_hit"]), "stop_reason": r["stop_reason"],
        "attempts": int(getattr(ctrl, "attempts", 0)),
        "catches_held": int(getattr(ctrl, "catches", 0)),
        "level": level,
        "rms_deg_tail": [float(np.degrees(np.sqrt(np.mean(th1[settled] ** 2)))) if settled.any() else None,
                         float(np.degrees(np.sqrt(np.mean(th2[settled] ** 2)))) if settled.any() else None],
        "cart_std_tail": float(np.std(r["x"][settled])) if settled.any() else None,
    }


def run_with_tap(sim, ctrl, x0, tap_t, tap):
    """Run in two pieces with an instantaneous arm-2 rate kick at tap_t."""
    from twin_sim import RigSimulator  # noqa: F401
    import twin_sim
    original = twin_sim.rk4_step
    state = {"done": False, "t": 0.0}

    def stepped(p, s, x_cmd, v, dt):
        state["t"] += dt
        out = original(p, s, x_cmd, v, dt)
        if not state["done"] and state["t"] >= tap_t:
            out = out.copy()
            out[3] += tap
            state["done"] = True
        return out
    twin_sim.rk4_step = stepped
    try:
        return sim.run(ctrl, x0)
    finally:
        twin_sim.rk4_step = original


def summarize(rows):
    out = {}
    keyf = lambda r: (r["scenario"], r["kind"], r["profile"], r["extra_delay"])
    for key in sorted(set(keyf(r) for r in rows), key=str):
        rs = [r for r in rows if keyf(r) == key]
        succ = [r for r in rs if r["success"]]
        ttb = [r["time_to_balance_s"] for r in succ if r["time_to_balance_s"] is not None]
        n = len(rs)
        p = len(succ) / n
        out["|".join(map(str, key))] = {
            "runs": n, "success_rate": p,
            "success_ci95": [max(0.0, p - 1.96 * math.sqrt(p * (1 - p) / n)),
                             min(1.0, p + 1.96 * math.sqrt(p * (1 - p) / n))],
            "time_to_balance_s_median": float(np.median(ttb)) if ttb else None,
            "time_to_balance_s_p90": float(np.percentile(ttb, 90)) if ttb else None,
            "peak_cart_m_median": float(np.median([r["peak_cart_m"] for r in rs])),
            "peak_cart_m_max": float(np.max([r["peak_cart_m"] for r in rs])),
            "limit_switch_hits": int(sum(r["limit_hit"] for r in rs)),
            "attempts_median": float(np.median([r["attempts"] for r in rs])),
            "attempts_total": int(sum(r["attempts"] for r in rs)),
            "per_attempt_success": (len(succ) / max(1, sum(r["attempts"] for r in rs))
                                    if rs and rs[0]["kind"].startswith("improved") and
                                    not rs[0]["kind"].endswith("balance") else None),
            "tail_rms_deg_median": [float(np.median([r["rms_deg_tail"][i] for r in succ])) if succ else None
                                    for i in (0, 1)],
            "stop_reasons": sorted({r["stop_reason"].split(" (")[0][:60] for r in rs if r["stop_reason"]}),
        }
    return out


def plots(rows, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pdir = HERE / "plots"
    pdir.mkdir(exist_ok=True)
    sw = [r for r in rows if r["scenario"] == "swingup"]
    if sw:
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
        ttb = [r["time_to_balance_s"] for r in sw if r["success"]]
        ax[0].hist(ttb, bins=20, color="#1f6fb4")
        ax[0].set_xlabel("time to balance from hanging (s)")
        ax[0].set_ylabel("runs")
        ax[0].set_title(f"Swing-up within 60 s: {np.mean([r['success'] for r in sw]):.0%} of {len(sw)} (fresh fit)")
        ax[1].hist([r["peak_cart_m"] for r in sw], bins=20, color="#d9480f")
        ax[1].set_xlabel("peak |cart| (m)")
        ax[1].set_title("Cart travel (limit switch ~0.45 m)")
        fig.tight_layout()
        fig.savefig(pdir / "monte_carlo_swingup.png", dpi=100)
        plt.close(fig)
    lat = [r for r in rows if r["scenario"].startswith("latency")]
    if lat:
        fig, ax = plt.subplots(figsize=(6.5, 3.6))
        for kind, prof, label, col in (("improved_balance", "conservative", "improved balance (conservative)", "#1f6fb4"),
                                       ("legacy", "conservative", "existing PythonMaster2_auto", "#999"),
                                       ("improved", "conservative", "improved swing-up + balance", "#2a9d8f")):
            rs = [r for r in lat if r["kind"] == kind]
            if not rs:
                continue
            ds = sorted(set(r["extra_delay"] for r in rs))
            ax.plot([1000 * d for d in ds], [np.mean([r["success"] for r in rs if r["extra_delay"] == d]) for d in ds],
                    "-o", label=label, color=col)
        ax.set_xlabel("extra loop latency added in the twin (ms)  [fresh-fit parameter spread]")
        ax.set_ylabel("success rate")
        ax.set_ylim(-0.03, 1.03)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(pdir / "monte_carlo_latency.png", dpi=100)
        plt.close(fig)
    bal = [r for r in rows if r["scenario"] == "balance"]
    if bal:
        fig, ax = plt.subplots(figsize=(6.5, 3.4))
        labels, rates, peaks = [], [], []
        for kind, prof in (("legacy", "conservative"), ("improved_balance", "conservative"),
                           ("improved_balance", "nominal")):
            rs = [r for r in bal if r["kind"] == kind and r["profile"] == prof]
            if rs:
                labels.append("existing" if kind == "legacy" else f"improved\n({prof})")
                rates.append(np.mean([r["success"] for r in rs]))
                peaks.append(np.median([r["peak_cart_m"] for r in rs]))
        ax.bar(labels, rates, color=["#999", "#1f6fb4", "#2a9d8f"][:len(labels)])
        for i, (rt, pk) in enumerate(zip(rates, peaks)):
            ax.text(i, rt + 0.02, f"{rt:.0%}\npeak cart {pk:.2f} m", ha="center", fontsize=8)
        ax.set_ylim(0, 1.25)
        ax.set_ylabel("balance kept for 25 s")
        ax.set_title("Hand-held start, tap on arm 2 at 5 s, old-fit (stale) parameter spread,\n"
                     "0.4 deg calibration errors - SIMULATED", fontsize=9)
        fig.tight_layout()
        fig.savefig(pdir / "monte_carlo_balance.png", dpi=100)
        plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--scenarios", nargs="+",
                    default=["swingup", "swingup_stale", "balance", "latency", "energy"])
    ap.add_argument("--workers", type=int, default=max(1, mp.cpu_count()))
    ap.add_argument("--plots-only", action="store_true", help="redraw plots from monte_carlo_results.json")
    args = ap.parse_args(argv)
    if args.plots_only:
        data = json.loads(RESULTS_PATH.read_text())
        plots(data["runs"], data["summary"])
        return 0
    jobs = []
    n = args.runs
    if "swingup" in args.scenarios:
        jobs += [("swingup", "improved", "conservative", 1000 + i, 60.0, 0.0) for i in range(n)]
    if "swingup_stale" in args.scenarios:
        jobs += [("swingup_stale", "improved", "conservative", 6000 + i, 60.0, 0.0) for i in range(max(10, n // 2))]
    if "energy" in args.scenarios:
        jobs += [("energy", "improved_energy", "conservative", 2000 + i, 60.0, 0.0) for i in range(max(10, n // 4))]
    if "balance" in args.scenarios:
        for kind, prof in (("legacy", "conservative"), ("improved_balance", "conservative"),
                           ("improved_balance", "nominal")):
            jobs += [("balance", kind, prof, 3000 + i, 25.0, 0.0) for i in range(n)]
    if "latency" in args.scenarios:
        m = max(10, n // 4)
        for d in (0.0, 0.005, 0.010, 0.015, 0.020, 0.030):
            for kind in ("legacy", "improved_balance"):
                jobs += [("latency_balance", kind, "conservative", 4000 + i, 15.0, d) for i in range(m)]
            jobs += [("latency_swingup", "improved", "conservative", 5000 + i, 40.0, d) for i in range(m)]
    t0 = time.time()
    with mp.Pool(args.workers) as pool:
        rows = []
        for k, row in enumerate(pool.imap_unordered(one_run, jobs, chunksize=2)):
            rows.append(row)
            if (k + 1) % 25 == 0:
                print(f"{k + 1}/{len(jobs)} runs ({time.time() - t0:.0f} s)", flush=True)
    summary = summarize(rows)
    old = json.loads(RESULTS_PATH.read_text()) if RESULTS_PATH.exists() else {}
    old_rows = [r for r in old.get("runs", []) if r["scenario"] not in {j[0] for j in jobs}]
    all_rows = old_rows + rows
    summary = summarize(all_rows)
    RESULTS_PATH.write_text(json.dumps({"summary": summary, "runs": all_rows}, indent=1) + "\n")
    plots(all_rows, summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
