"""Stage 2 of the video analysis: track the cart and both link angles.

Input is the stabilised stack written by stabilize.py (every frame warped onto
one reference frame, so the rail is fixed in image coordinates).

Method
  1. Background = per-pixel 95th-percentile brightness of the registered
     frames (the black links occupy any pixel only part of the time).
     "Darkness" = background - frame highlights the links and hides static
     dark objects such as monitors.
  2. Cart: normalised cross-correlation of a carriage template along the rail
     row (the rail is horizontal in the reference frame).
  3. Link 1: for every frame, score 360 candidate angles by the mean darkness
     along a ray of length L1 from the pivot; link 2 likewise from the
     elbow. A Viterbi pass over angle bins enforces temporal continuity
     (links cannot jump more than ~0.5 rad between 60 fps frames).
  4. Sub-bin refinement by a parabola through the best bin's neighbours.

Angles are reported in the controller's model convention (0 = upright,
+/-pi = hanging, positive tips the link toward +x / image right). Cart position
is reported in metres relative to the mean hanging position, using a pixel
scale from the known link lengths (Code/Global/physical_parameters.py).

Usage:
    python3 track_video.py STAB.npz OUT_DIR [--start 5.6]
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

CODE_DIRECTORY = Path(__file__).resolve().parents[2]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

from Global.physical_parameters import DOUBLE_ARM_1, DOUBLE_ARM_2  # noqa: E402

# Reference-frame geometry (half-resolution pixels), measured on the
# registered reference frame (t = 15.75 s); see README for the annotated grid.
PIVOT_Y = 262.0
CART_TEMPLATE_BOX = (300, 248, 357, 292)   # x0, y0, x1, y1 at t = 9 s (hanging, centred)
CART_TEMPLATE_TIME_S = 9.0
RAIL_X_RANGE = (70, 570)
N_BINS = 360
FOLD_EXCLUDE_DEG = 20


def bilinear(img, xs, ys):
    return cv2.remap(img, xs.astype(np.float32), ys.astype(np.float32),
                     interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def ray_scores(dark, origin, length_px, inner=0.18, outer=0.92, n_along=36, half_width=3):
    """Mean darkness along N_BINS rays from origin; returns shape (N_BINS,)."""
    ang = np.arange(N_BINS) * (2 * np.pi / N_BINS)
    s = np.linspace(inner, outer, n_along) * length_px
    off = np.arange(-half_width, half_width + 1, 1.5)
    dx, dy = np.sin(ang), -np.cos(ang)            # model angle 0 = up (image -y)
    px, py = -dy, dx                               # perpendicular
    xs = origin[0] + dx[:, None, None] * s[None, :, None] + px[:, None, None] * off[None, None, :]
    ys = origin[1] + dy[:, None, None] * s[None, :, None] + py[:, None, None] * off[None, None, :]
    shape = xs.shape
    vals = bilinear(dark, xs.reshape(shape[0], -1), ys.reshape(shape[0], -1)).reshape(shape)
    return vals.max(axis=2).mean(axis=1)


def viterbi(scores, max_jump_bins=30, jump_penalty=0.02, circular=True):
    """Best bin path through scores (T, N); bins are angles (circular) or pixels."""
    T, N = scores.shape
    d = np.arange(N)
    diff = np.abs(d[:, None] - d[None, :])
    if circular:
        diff = np.minimum(diff, N - diff)
    trans = np.where(diff <= max_jump_bins, -jump_penalty * diff / max_jump_bins, -np.inf)
    acc = scores[0].copy()
    back = np.zeros((T, N), np.int32)
    for t in range(1, T):
        cand = acc[None, :] + trans            # [to, from]
        back[t] = np.argmax(cand, axis=1)
        acc = cand[np.arange(N), back[t]] + scores[t]
    path = np.zeros(T, int)
    path[-1] = int(np.argmax(acc))
    for t in range(T - 1, 0, -1):
        path[t - 1] = back[t, path[t]]
    return path


def refine(scores, path):
    N = scores.shape[1]
    out = np.zeros(len(path))
    for t, k in enumerate(path):
        y0, y1, y2 = scores[t, (k - 1) % N], scores[t, k], scores[t, (k + 1) % N]
        den = y0 - 2 * y1 + y2
        delta = 0.5 * (y0 - y2) / den if den < 0 else 0.0
        out[t] = (k + np.clip(delta, -0.5, 0.5)) * 2 * np.pi / N
    return out


def track(stab_path, start_s, px_per_m_guess=478.0):
    data = np.load(stab_path)
    frames = data["warped"]
    fps = float(data["fps"])
    inliers = data["inliers"]
    first = int(round(start_s * fps))
    idx = np.arange(first, len(frames))
    stack = frames[idx].astype(np.float32)
    valid = stack > 0
    bg = np.percentile(stack[::3], 95, axis=0).astype(np.float32)
    # Cart template (hanging, centred).
    x0, y0, x1, y1 = CART_TEMPLATE_BOX
    tmpl = frames[int(round(CART_TEMPLATE_TIME_S * fps))][y0:y1, x0:x1].astype(np.float32)
    band = (y0 - 4, y1 + 4)
    strip_x0 = RAIL_X_RANGE[0] - 40
    ncc_rows = []
    L1px = DOUBLE_ARM_1.length_m * px_per_m_guess
    L2px = DOUBLE_ARM_2.length_m * px_per_m_guess
    s1 = np.zeros((len(idx), N_BINS))
    darks = []
    for n, frame in enumerate(stack):
        strip = frame[band[0]:band[1], RAIL_X_RANGE[0] - 40:RAIL_X_RANGE[1] + 40]
        res = cv2.matchTemplate(strip, tmpl, cv2.TM_CCOEFF_NORMED)
        ncc_rows.append(res.max(axis=0))
        dark = np.clip(bg - frame, 0, 120) / 120.0
        dark[~valid[n]] = 0
        darks.append(dark.astype(np.float16))
    # Pivot x relative to template centre: hub centre measured at 327 px vs template centre 328.5.
    pivot_offset = 327.0 - (x0 + x1) / 2
    # Cart: Viterbi over x so an occluded/blurred carriage cannot jump to the
    # rail end blocks; the cart moves < ~12 px (2.5 cm) between frames.
    ncc = np.clip(np.stack(ncc_rows), 0, 1)
    path = viterbi(ncc, max_jump_bins=14, jump_penalty=0.05, circular=False)
    cart_q = ncc[np.arange(len(idx)), path]
    cart_k = path.astype(float)
    for n, k in enumerate(path):
        row = ncc[n]
        if 0 < k < len(row) - 1:
            den = row[k - 1] - 2 * row[k] + row[k + 1]
            if den < 0:
                cart_k[n] += np.clip(0.5 * (row[k - 1] - row[k + 1]) / den, -0.5, 0.5)
    t = idx / fps
    cart_px = strip_x0 + cart_k + (x1 - x0) / 2
    cart_px_f = cart_px + pivot_offset
    for n in range(len(idx)):
        s1[n] = ray_scores(darks[n].astype(np.float32), (cart_px_f[n], PIVOT_Y), L1px)
    p1 = viterbi(s1)
    th1 = refine(s1, p1)
    s2 = np.zeros_like(s1)
    for n in range(len(idx)):
        ex = cart_px_f[n] + L1px * np.sin(th1[n])
        ey = PIVOT_Y - L1px * np.cos(th1[n])
        s2[n] = ray_scores(darks[n].astype(np.float32), (ex, ey), L2px, inner=0.22, outer=0.95)
        # Link 2 cannot lie back along link 1: suppress +/-FOLD_EXCLUDE_DEG
        # around the reversed link-1 direction (otherwise the ray finds link 1).
        back = int(round((th1[n] + np.pi) / (2 * np.pi) * N_BINS)) % N_BINS
        k = np.arange(-FOLD_EXCLUDE_DEG, FOLD_EXCLUDE_DEG + 1) * N_BINS // 360
        s2[n, (back + k) % N_BINS] = 0.0
    p2 = viterbi(s2)
    th2 = refine(s2, p2)
    return dict(t=t, frame=idx, cart_px=cart_px_f,
                cart_score=cart_q, th1=th1, th2=th2,
                score1=s1[np.arange(len(idx)), p1], score2=s2[np.arange(len(idx)), p2],
                inliers=inliers[idx], fps=fps, bg=bg, darks=darks, s1=s1, s2=s2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stab")
    parser.add_argument("out_dir")
    parser.add_argument("--start", type=float, default=5.6)
    parser.add_argument("--px-per-m", type=float, default=478.0)
    args = parser.parse_args(argv)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    r = track(args.stab, args.start, args.px_per_m)
    np.savez_compressed(out / "track_raw.npz", **{k: v for k, v in r.items() if k not in ("darks",)})
    print(json.dumps({"frames": len(r["t"]), "median_score1": float(np.median(r["score1"])),
                      "median_score2": float(np.median(r["score2"]))}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
