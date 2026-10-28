"""Validate the fitted twin on data it was NOT fitted to: the filmed swing-up run.

The run filmed in Media/IMG_0494_double_pendulum.mp4 is the Pi log
swingup_double_20261027_185639_236068.csv (matched by cross-correlating the
cart motion, see video_analysis/). Each swing-up attempt is large-angle,
fast and nonlinear, unlike the small-angle system-ID data used for fitting.

For every attempt the twin starts from the logged state and is driven open
loop by the logged cart commands (exactly like the fit). The plot overlays
twin, encoder log and the independent video tracking.

    python3 validate_twin.py SWINGUP_LOG.csv [--video video_analysis/video_track.csv]
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fit_parameters import WindowSet, DT  # noqa: E402
import fit_parameters  # noqa: E402
from twin_model import load_parameters, TwinParameters  # noqa: E402


def load_run_log(path):
    csv.field_size_limit(10 ** 9)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    samples = [r for r in rows if r["record_type"] == "sample"]
    events = [(r["record_type"], r["elapsed_s"], r["message"]) for r in rows
              if r["record_type"] in ("swing", "abort", "catch", "fault")]
    num = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in samples])
    t = num("elapsed_s")
    data = dict(t=t, th1=num("measured_angle_1_rad"), th2=num("measured_angle_2_rad"),
                x_cmd=num("cart_m"), v_cmd=num("cart_velocity_command_m_s"), t_write=t + 0.0005,
                test=np.array(["run"] * len(t)), name=Path(path).name)
    for key in ("th1", "th2"):
        bad = ~np.isfinite(data[key])
        data[key][bad] = np.nan
    return data, events


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log")
    ap.add_argument("--video", default=str(HERE / "video_analysis" / "video_track.csv"))
    ap.add_argument("--out", default=str(HERE / "plots" / "validation_swingup.png"))
    ap.add_argument("--window", type=float, default=2.6)
    args = ap.parse_args(argv)
    data, events = load_run_log(args.log)
    starts = [float(e[1]) for e in events if e[0] == "swing"]
    fit_parameters.WINDOW_S = args.window
    wins = []
    for s in starts:
        i0 = int(np.searchsorted(data["t"], s + 0.05))
        i1 = int(np.searchsorted(data["t"], s + 0.05 + args.window))
        wins.append(("attempt", i0, i1))
    ws = WindowSet(data, wins)
    results = {}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    video = None
    if Path(args.video).exists():
        video = np.genfromtxt(args.video, delimiter=",", names=True)
    fig, axes = plt.subplots(len(wins), 2, figsize=(12, 2.9 * len(wins)), squeeze=False)
    for label, p in (("fitted", load_parameters()), ("nominal", TwinParameters())):
        theta = [p.b1, p.c1, p.b2, p.c2, 1, 1, 1, 1, p.sensor_delay_s, p.motor_tau_s]
        base = p.with_updates(J1_eff=None, J2_eff=None, c_eff=None, h1_eff=None, h2_eff=None) \
            if label == "fitted" else p
        theta[4:8] = [p.J1 / base.J1, p.J2 / base.J2, p.c / base.c, p.h2 / base.h2]
        _, parts = ws.residuals(base, np.array(theta), return_parts=True)
        errs = []
        for k, (mt, m1, m2, s1, s2) in enumerate(parts):
            for horizon in (0.5, 1.0, 1.5):
                m = (mt <= horizon) & np.isfinite(m1) & np.isfinite(m2)
                errs.append((k, horizon, float(np.degrees(np.sqrt(np.mean((s1[m] - m1[m]) ** 2)))),
                             float(np.degrees(np.sqrt(np.mean((s2[m] - m2[m]) ** 2))))))
            if label == "fitted" or label == "nominal":
                t0 = data["t"][wins[k][1]]
                for j, (meas, sim) in enumerate(((m1, s1), (m2, s2))):
                    ax = axes[k, j]
                    if label == "fitted":
                        ax.plot(mt, np.degrees(meas), color="#999", lw=3, label="encoder (Pi log)")
                        if video is not None:
                            tv = video["t_log_s"] - t0
                            sel = (tv >= 0) & (tv <= mt[-1])
                            col = "theta1_rad" if j == 0 else "theta2_rad"
                            vv = np.unwrap(video[col][sel])
                            if sel.any():
                                vv += 2 * np.pi * np.round((np.interp(tv[sel][0], mt, meas) - vv[0]) / (2 * np.pi))
                                ax.plot(tv[sel], np.degrees(vv), ".", ms=2, color="#2a9d8f", label="video")
                    ax.plot(mt, np.degrees(sim), lw=1.2, ls="-" if label == "fitted" else "--",
                            color="#1f6fb4" if label == "fitted" else "#d9480f", label=f"twin ({label})")
                    ax.set_ylabel(f"attempt {k + 1}: arm {j + 1} (deg)", fontsize=8)
                    ax.grid(alpha=0.3)
        results[label] = errs
    axes[0, 0].legend(fontsize=7)
    axes[-1, 0].set_xlabel("time since attempt start (s)")
    axes[-1, 1].set_xlabel("time since attempt start (s)")
    fig.suptitle("Twin vs. real swing-up attempts (open loop, driven by the logged cart commands)")
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=95)
    summary = {}
    for label, errs in results.items():
        e = np.array([x[1:] for x in errs])
        summary[label] = {f"{h:.1f}s": {"arm1_rms_deg": float(np.mean(e[e[:, 0] == h, 1])),
                                         "arm2_rms_deg": float(np.mean(e[e[:, 0] == h, 2]))}
                          for h in (0.5, 1.0, 1.5)}
    out = {"log": data["name"], "attempt_starts_s": starts, "open_loop_error_vs_horizon": summary}
    Path(args.out).with_suffix(".json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
