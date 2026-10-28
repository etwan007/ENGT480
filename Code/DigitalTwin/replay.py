"""Run one simulated rig session and save plots, an animation and a CSV.

    python3 replay.py                                   # improved controller, swing-up from hanging
    python3 replay.py --controller legacy --start upright --duration 15
    python3 replay.py --controller improved --start upright --kick 0.3

Outputs (default folder Code/DigitalTwin/run_logs/, git-ignored; the committed
examples in sim_runs/ were made with --out sim_runs):
    <name>.png   angles, cart, command, energy, controller mode
    <name>.gif   animation of the twin (small; --no-animation to skip)
    <name>.csv   DOUBLE_COLUMNS format: open it in Code/RunReview/balancing_visualizer.html
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for path in (HERE.parent, HERE, HERE.parent / "DoublePendulum" / "Improved"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_sim import RigSimulator, SimConfig, hanging_state, write_run_csv  # noqa: E402
from twin_model import load_parameters, wrap_angle, link_points  # noqa: E402

MODE_COLOURS = {"WAIT": "#bbbbbb", "SWING": "#f2c14e", "BALANCE": "#7bc47f",
                "RECOVER": "#e07b67", "STOPPED": "#555555"}


def summarize(r, dt_ok=5.0):
    m = r["mode"]
    t = r["t"]
    bal = m == "BALANCE"
    balanced_at = None
    n_hold = int(round(2.0 / np.median(np.diff(t)))) if len(t) > 2 else 0
    for i in range(len(m)):
        if bal[i] and bal[i:i + n_hold].all() and len(bal[i:i + n_hold]) == n_hold:
            balanced_at = float(t[i])
            break
    up_end = bool(len(t) and np.all(np.abs(wrap_angle(r["th1"][-int(1 / 0.01):])) < 0.2)
                  and np.all(np.abs(wrap_angle(r["th2"][-int(1 / 0.01):])) < 0.2))
    return {
        "duration_s": float(t[-1]) if len(t) else 0.0,
        "upright_at_end": up_end,
        "time_to_balance_s": balanced_at,
        "peak_cart_travel_m": float(np.max(np.abs(r["x"]))) if len(t) else 0.0,
        "peak_cart_speed_m_s": float(np.max(np.abs(r["v"]))) if len(t) else 0.0,
        "limit_switch_hit": bool(r["limit_hit"]),
        "stop_reason": r["stop_reason"],
        "mode_changes": [(round(float(t[i]), 2), str(m[i])) for i in range(len(m))
                         if i == 0 or m[i] != m[i - 1]][:30],
    }


def plot_run(r, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = r["t"]
    fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    ax[0].plot(t, np.degrees(wrap_angle(r["th1"])), label="arm 1 (true)", color="#1f6fb4")
    ax[0].plot(t, np.degrees(wrap_angle(r["th2"])), label="arm 2 (true)", color="#d9480f")
    ax[0].set_ylabel("angle from upright (deg)")
    ax[0].set_ylim(-185, 185)
    ax[1].plot(t, r["x"], label="cart x", color="#333")
    ax[1].axhline(r["params"].rail_limit_m, color="r", lw=0.8, ls="--")
    ax[1].axhline(-r["params"].rail_limit_m, color="r", lw=0.8, ls="--", label="limit switch")
    ax[1].set_ylabel("cart (m)")
    ax[2].plot(t, r["v_req"], label="velocity command", color="#6a51a3")
    ax[2].plot(t, r["v"], label="cart velocity", color="#999", lw=0.8)
    ax[2].set_ylabel("m/s")
    ax[3].plot(t, r["energy"], color="#2a9d8f", label="link energy")
    ax[3].axhline(r["upright_energy"], color="k", lw=0.8, ls=":", label="upright energy")
    ax[3].set_ylabel("J")
    ax[3].set_xlabel("time (s)")
    m = r["mode"]
    start = 0
    for i in range(1, len(m) + 1):
        if i == len(m) or m[i] != m[start]:
            for a in ax:
                a.axvspan(t[start], t[min(i, len(t) - 1)], color=MODE_COLOURS.get(m[start], "#fff"),
                          alpha=0.18, lw=0)
            start = i
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=7)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.4) for c in MODE_COLOURS.values()]
    ax[0].legend(handles + ax[0].get_lines(), list(MODE_COLOURS) + ["arm 1", "arm 2"],
                 loc="upper right", fontsize=7, ncol=4)
    ax[0].set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def animate_run(r, path, fps=20, max_s=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    p = r["params"]
    t = r["t"]
    step = max(1, int(round(1 / fps / np.median(np.diff(t)))))
    idx = np.arange(0, len(t), step)
    if max_s is not None:
        idx = idx[t[idx] <= max_s]
    fig, ax = plt.subplots(figsize=(4.8, 3.0), dpi=70)
    lim = p.rail_limit_m + 0.05
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-0.5, 0.5)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot([-p.rail_limit_m, p.rail_limit_m], [0, 0], color="#888", lw=3)
    cart, = ax.plot([], [], "s", color="#444", ms=10)
    arms, = ax.plot([], [], "-o", color="#1f6fb4", lw=3, ms=4)
    label = ax.text(-lim + 0.01, 0.43, "", fontsize=8)

    def draw(k):
        i = idx[k]
        pv, el, tip = link_points(p, r["x"][i], r["th1"][i], r["th2"][i])
        cart.set_data([pv[0]], [pv[1]])
        arms.set_data([pv[0], el[0], tip[0]], [pv[1], el[1], tip[1]])
        label.set_text(f"t={t[i]:5.2f}s  {r['mode'][i]}  (SIMULATED)")
        return cart, arms, label

    anim = FuncAnimation(fig, draw, frames=len(idx), blit=True)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def simulate_and_report(controller, duration_s=20.0, seed=0, start="hanging", out_dir=None,
                        name=None, animate=True, params=None, kick=0.0, controller_name=None,
                        gif_seconds=10.0):
    params = params if params is not None else load_parameters()
    if start == "hanging":
        x0 = hanging_state()
    else:
        x0 = np.array([math.radians(1.5) + kick, 0.0, math.radians(-1.0), 0.0, 0.0, 0.0])
    sim = RigSimulator(params, SimConfig(duration_s=duration_s, seed=seed))
    r = sim.run(controller, x0)
    summary = summarize(r)
    out = {"summary": summary, "result": r}
    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        cname = controller_name or type(controller).__name__
        name = name or f"SIMULATED_{cname}_{start}_seed{seed}"
        plot_run(r, out_dir / f"{name}.png", f"Digital twin: {cname}, start {start} (SIMULATED)")
        write_run_csv(r, out_dir / f"{name}.csv", cname)
        if animate:
            animate_run(r, out_dir / f"{name}.gif", max_s=gif_seconds)
        (out_dir / f"{name}.json").write_text(json.dumps(summary, indent=2) + "\n")
        summary["files"] = [str(out_dir / f"{name}.{ext}") for ext in ("png", "csv", "gif", "json")]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", choices=("improved", "legacy"), default="improved")
    ap.add_argument("--start", choices=("hanging", "upright"), default=None)
    ap.add_argument("--profile", default="conservative")
    ap.add_argument("--swing-mode", default="trajectory", choices=("trajectory", "energy"))
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--kick", type=float, default=0.0, help="extra initial arm-1 angle (rad), upright start")
    ap.add_argument("--out", default=str(HERE / "run_logs"))
    ap.add_argument("--name", default=None)
    ap.add_argument("--no-animation", action="store_true")
    ap.add_argument("--gif-seconds", type=float, default=10.0, help="length of the animation")
    args = ap.parse_args(argv)
    if args.controller == "legacy":
        from legacy_controller import LegacyController
        controller = LegacyController()
        start = args.start or "upright"
    else:
        from improved_controller import ImprovedController, ControllerConfig
        start = args.start or "hanging"
        cfg = ControllerConfig(balance_only=(start == "upright"), swing_mode=args.swing_mode)
        controller = ImprovedController(config=cfg, profile=args.profile)
    out = simulate_and_report(controller, args.duration, args.seed, start, args.out, args.name,
                              not args.no_animation, kick=args.kick, controller_name=args.controller,
                              gif_seconds=args.gif_seconds)
    print(json.dumps(out["summary"], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
