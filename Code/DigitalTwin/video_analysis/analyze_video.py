"""Stage 3 of the video analysis: scale, synchronise, check and summarise.

* Synchronises the tracked cart motion with a Pi run log (same rig session)
  by cross-correlation, which gives both the time offset and the true
  horizontal pixel scale (the log's cart position comes from step counts).
* Compares the video angles with the encoder angles from that log.
* Checks the link-length ratio from the image alone.
* Estimates hanging natural frequency, cart travel/speed/acceleration.
* Writes video_track.csv, video_summary.json and the PNG plots in this folder.

Usage:
    python3 analyze_video.py STAB.npz TRACK_RAW.npz [--log RUN.csv] [--out DIR]
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from track_video import PIVOT_Y, ray_scores, N_BINS  # noqa: E402

sys.path.insert(0, str(HERE.parents[1]))
from Global.physical_parameters import DOUBLE_ARM_1, DOUBLE_ARM_2  # noqa: E402

TRACK_PX_PER_M = 478.0   # nominal scale used by track_video.py for its ray lengths
PIVOT_X_HANGING = 327.0  # pivot x of the centred, hanging cart in the reference frame


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_log(path):
    import csv
    csv.field_size_limit(10 ** 9)
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["record_type"] == "sample"]
    num = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    return dict(t=num("elapsed_s"), cart=num("cart_m"), th1=num("measured_angle_1_rad"),
                joint=num("joint_angle_2_rad"), vcmd=num("cart_velocity_command_m_s"),
                acmd=num("force_command_n"))


def synchronise(tv, xv_px, log):
    """Return (offset_s, px_per_m, correlation): log_time = video_time + offset."""
    tg = np.arange(0, log["t"][-1], 1 / 120)
    xg = np.interp(tg, log["t"], log["cart"])
    best = (-1.0, 0.0, 1.0)
    xv = xv_px - np.mean(xv_px)
    for off in np.arange(-10, log["t"][-1] - 10, 1 / 120):
        xl = np.interp(tv + off, tg, xg, left=np.nan, right=np.nan)
        m = np.isfinite(xl)
        if m.sum() < 0.8 * len(tv):
            continue
        a = xv[m] - xv[m].mean()
        b = xl[m] - xl[m].mean()
        c = float(a @ b / np.sqrt((a @ a) * (b @ b) + 1e-12))
        if c > best[0]:
            px_per_m = float((a @ b) / (b @ b))   # least-squares pixel scale
            best = (c, float(off), px_per_m)
    return best[1], best[2], best[0]


def segment_cover(dark, origin, angle, lengths, step=1.0):
    """For each candidate length: sum of (darkness - 0.5) along the segment.

    The sum grows while the segment stays on the dark link and falls once it
    runs past the link end, so its maximum marks the visible end of the link.
    """
    s = np.arange(0, lengths.max() + step, step)
    xs = origin[0] + s * np.sin(angle)
    ys = origin[1] - s * np.cos(angle)
    v = cv2.remap(dark, xs[None].astype(np.float32), ys[None].astype(np.float32),
                  cv2.INTER_LINEAR)[0]
    c = np.cumsum(v - 0.5) * step
    return np.interp(lengths, s, c)


def link_length_fit(darks, r, th1_true, joint_true, frames_sel):
    """Fit link-1 (pivot->elbow) and link-2 (elbow->tip) lengths in pixels using
    the ENCODER angles, on frames where that link is within 35 deg of
    horizontal (the horizontal pixel scale is known from the log sync)."""
    L1c = np.arange(60, 130, 1.0)
    L2c = np.arange(60, 160, 1.0)
    th2_true = th1_true + joint_true
    near_h = lambda a: np.abs(np.abs(wrap(a)) - np.pi / 2) < np.radians(35)
    l1s, l2s = [], []
    for n in frames_sel:
        d = darks[n]
        p = (r["cart_px"][n], PIVOT_Y)
        cov1 = segment_cover(d, p, th1_true[n], L1c)
        l1 = L1c[np.argmax(cov1)]
        if near_h(th1_true[n]):
            l1s.append(l1)
        if near_h(th2_true[n]) and np.abs(wrap(joint_true[n])) > np.radians(30):
            e = (p[0] + l1 * np.sin(th1_true[n]), p[1] - l1 * np.cos(th1_true[n]))
            cov2 = segment_cover(d, e, th2_true[n], L2c)
            l2s.append(L2c[np.argmax(cov2)])
    return np.array(l1s), np.array(l2s)


def tip_length(dark, origin, angle, max_px=160):
    s = np.arange(0, max_px, 0.5)
    xs = origin[0] + s * np.sin(angle)
    ys = origin[1] - s * np.cos(angle)
    v = cv2.remap(dark, xs[None].astype(np.float32), ys[None].astype(np.float32), cv2.INTER_LINEAR)[0]
    # The tip is where darkness collapses after the second link (> 0.6 L1+L2).
    start = int(0.6 * max_px / 0.5)
    after = np.flatnonzero(v[start:] < 0.3)
    return float(s[start + after[0]]) if len(after) else float("nan")


def dominant_frequency(t, y):
    y = y - np.mean(y)
    dt = np.median(np.diff(t))
    n = 1 << int(np.ceil(np.log2(len(y) * 8)))
    spec = np.abs(np.fft.rfft(y * np.hanning(len(y)), n))
    f = np.fft.rfftfreq(n, dt)
    band = (f > 0.4) & (f < 4)
    return float(f[band][np.argmax(spec[band])])


def savgol_derivs(t, x, window=15, order=3):
    from scipy.signal import savgol_filter
    dt = np.median(np.diff(t))
    v = savgol_filter(x, window, order, deriv=1, delta=dt)
    a = savgol_filter(x, window, order, deriv=2, delta=dt)
    return v, a


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("stab")
    ap.add_argument("track")
    ap.add_argument("--log")
    ap.add_argument("--out", default=str(HERE))
    args = ap.parse_args(argv)
    out = Path(args.out)
    stab = np.load(args.stab)
    r = dict(np.load(args.track))
    tv = r["t"]
    fps = float(r["fps"])
    summary = {"video_frames_tracked": int(len(tv)), "video_fps": fps,
               "tracked_from_s": float(tv[0]), "tracked_to_s": float(tv[-1])}

    log = read_log(args.log) if args.log else None
    if log is not None:
        offset, px_per_m, corr = synchronise(tv, r["cart_px"], log)
        summary.update(log_file=Path(args.log).name, log_time_offset_s=offset,
                       px_per_m_from_log=px_per_m, sync_correlation=corr)
    else:
        offset, px_per_m = 0.0, TRACK_PX_PER_M
    cart_m = (r["cart_px"] - PIVOT_X_HANGING) / px_per_m
    th1, th2 = r["th1"], r["th2"]
    joint = wrap(th2 - th1)

    # ---- geometry check (image only) --------------------------------------
    darks = None
    hanging = (np.abs(wrap(th1 - np.pi)) < np.radians(4)) & (np.abs(joint) < np.radians(4))
    warped = stab["warped"]
    bg = r["bg"].astype(np.float32)

    class _Darks:  # computed on demand; the full float stack does not fit in RAM
        def __getitem__(self, n):
            return np.clip(bg - warped[r["frame"][n]].astype(np.float32), 0, 120) / 120.0

    darks = _Darks()
    tips = [tip_length(darks[n], (r["cart_px"][n], PIVOT_Y), th1[n], 250) for n in np.flatnonzero(hanging)[::5]]
    total_px = float(np.nanmedian(tips))
    summary["hanging_pivot_to_tip_px"] = total_px
    summary["hanging_pivot_to_tip_m_if_vertical_scale_equals_horizontal"] = total_px / px_per_m

    # ---- comparison with encoder log --------------------------------------
    if log is not None:
        tl = tv + offset
        m = np.isfinite(log["joint"])
        th1_log = np.interp(tl, log["t"], np.unwrap(log["th1"]))
        joint_log = np.interp(tl, log["t"][m], np.unwrap(log["joint"][m]))
        cart_log = np.interp(tl, log["t"], log["cart"])
        e1 = np.degrees(wrap(th1 - th1_log))
        ej = np.degrees(wrap(joint - joint_log))
        ec = (cart_m - cart_log) * 1000
        summary["video_vs_encoder"] = {
            "theta1_bias_deg": float(np.median(e1)), "theta1_std_deg": float(np.std(e1)),
            "joint_bias_deg": float(np.median(ej)), "joint_std_deg": float(np.std(ej)),
            "joint_abs_p95_deg": float(np.percentile(np.abs(ej - np.median(ej)), 95)),
            "cart_rms_mm": float(np.sqrt(np.mean(ec ** 2))),
            "cart_abs_p95_mm": float(np.percentile(np.abs(ec), 95)),
        }

        b1 = np.radians(summary["video_vs_encoder"]["theta1_bias_deg"])
        bj = np.radians(summary["video_vs_encoder"]["joint_bias_deg"])
        sel = np.flatnonzero((r["score1"] > 0.85) & (np.abs(e1 - np.degrees(b1)) < 6))[::2]
        l1s, l2s = link_length_fit(darks, r, th1_log + b1, joint_log + bj, sel)
        if len(l1s) > 10 and len(l2s) > 10:
            summary["link_length_fit"] = {
                "frames_link1": int(len(l1s)), "frames_link2": int(len(l2s)),
                "link1_pivot_to_elbow_m": [float(np.percentile(l1s, q) / px_per_m) for q in (25, 50, 75)],
                "link2_elbow_to_tip_m": [float(np.percentile(l2s, q) / px_per_m) for q in (25, 50, 75)],
                "ratio_L2_over_L1_video": float(np.median(l2s) / np.median(l1s)),
                "ratio_L2_over_L1_nominal": DOUBLE_ARM_2.length_m / DOUBLE_ARM_1.length_m,
                "note": "visible ends include end caps/hubs; [25,50,75] percentiles",
            }

    # ---- motion statistics --------------------------------------------------
    v, a = savgol_derivs(tv, cart_m)
    summary["cart_travel_m"] = [float(cart_m.min()), float(cart_m.max())]
    summary["cart_peak_speed_m_s"] = float(np.percentile(np.abs(v), 99.5))
    summary["cart_peak_accel_m_s2"] = float(np.percentile(np.abs(a), 99.5))
    # hanging free-oscillation windows (cart quiet): natural frequency of link 1
    freqs = []
    for t0, t1 in ((8.0, 13.2), (20.0, 24.3)):
        w = (tv > t0) & (tv < t1)
        freqs.append(dominant_frequency(tv[w], np.unwrap(th1[w])))
    summary["hanging_theta1_freq_hz"] = freqs

    # ---- outputs -----------------------------------------------------------
    header = ("t_video_s,t_log_s,cart_m,theta1_rad,theta2_rad,joint_rad,"
              "cart_speed_m_s,cart_accel_m_s2,score_link1,score_link2,score_cart")
    table = np.column_stack([tv, tv + offset, cart_m, wrap(th1), wrap(th2), joint, v, a,
                             r["score1"], r["score2"], r["cart_score"]])
    np.savetxt(out / "video_track.csv", table, delimiter=",", header=header, comments="",
               fmt="%.5f")
    (out / "video_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    if log is not None:
        tl = log["t"] - offset
        ax[0].plot(tl, log["cart"], color="#999", lw=2.5, label="Pi log (step count)")
        ax[1].plot(tl, np.degrees(np.mod(log["th1"], 2 * np.pi)), ".", color="#999", ms=3, label="encoder")
        ax[2].plot(tl, np.degrees(wrap(log["joint"])), ".", color="#999", ms=3, label="encoder")
    ax[0].plot(tv, cart_m, color="#1f6fb4", lw=1.2, label="video")
    ax[1].plot(tv, np.degrees(np.mod(th1, 2 * np.pi)), ".", color="#1f6fb4", ms=2, label="video")
    ax[2].plot(tv, np.degrees(joint), ".", color="#1f6fb4", ms=2, label="video")
    ax[0].set_ylabel("cart x (m)")
    ax[1].set_ylabel("theta1 (deg, 0/360=up, 180=hanging)")
    ax[2].set_ylabel("joint theta2-theta1 (deg)")
    ax[2].set_xlabel("video time (s)")
    for a_ in ax:
        a_.grid(alpha=0.3)
        a_.legend(loc="upper left", fontsize=8)
    ax[0].set_xlim(tv[0], tv[-1])
    ax[0].set_title("IMG_0494: video tracking vs. Pi log " + (Path(args.log).name if args.log else ""))
    fig.tight_layout()
    fig.savefig(out / "video_vs_log.png", dpi=110)
    plt.close(fig)

    # overlay montage
    L1 = DOUBLE_ARM_1.length_m * px_per_m
    L2 = DOUBLE_ARM_2.length_m * px_per_m
    tiles = []
    for tt in (9.0, 13.6, 14.2, 14.6, 15.0, 15.4, 15.9, 25.2, 25.6, 26.0, 26.6, 27.2):
        n = int(np.argmin(np.abs(tv - tt)))
        f = cv2.cvtColor(stab["warped"][r["frame"][n]], cv2.COLOR_GRAY2BGR)
        p = np.array([r["cart_px"][n], PIVOT_Y])
        e = p + L1 * np.array([np.sin(th1[n]), -np.cos(th1[n])])
        q = e + L2 * np.array([np.sin(th2[n]), -np.cos(th2[n])])
        cv2.line(f, tuple(p.astype(int)), tuple(e.astype(int)), (40, 40, 230), 2)
        cv2.line(f, tuple(e.astype(int)), tuple(q.astype(int)), (40, 200, 40), 2)
        cv2.circle(f, tuple(p.astype(int)), 4, (230, 40, 230), -1)
        cv2.putText(f, f"t={tt:.1f}s", (95, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 60, 0), 2)
        tiles.append(f[125:415, 80:620])
    grid = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, 12, 4)])
    cv2.imwrite(str(out / "video_overlay_frames.jpg"), cv2.resize(grid, None, fx=0.7, fy=0.7),
                [cv2.IMWRITE_JPEG_QUALITY, 80])

    fig, ax = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
    ax[0].plot(tv, v)
    ax[0].set_ylabel("cart speed (m/s)")
    ax[1].plot(tv, a)
    if log is not None:
        ax[1].plot(log["t"] - offset, log["acmd"], color="#999", lw=0.8, label="commanded accel (log)")
        ax[1].legend(fontsize=8)
    ax[1].set_ylabel("cart accel (m/s^2)")
    ax[1].set_xlabel("video time (s)")
    ax[1].set_xlim(tv[0], tv[-1])
    ax[0].set_title("Cart from video (Savitzky-Golay, 0.25 s window: peaks are smoothed)")
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "video_cart_kinematics.png", dpi=100)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
