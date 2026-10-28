"""Stage 1 of the video analysis: register every frame to one reference frame.

The phone was handheld, so each frame is mapped onto a reference frame with a
RANSAC homography fitted to ORB features on the near plane (desk edge, rail
extrusion and cabinet front). The moving cart and links are rejected as
outliers. Output: a compressed .npz with the warped half-resolution grey
frames, per-frame homographies and an inlier-quality score.

Usage:
    python3 stabilize.py VIDEO.mp4 OUT.npz [--ref-time 15.75] [--scale 0.5]
"""

import argparse
import sys

import cv2
import numpy as np


def read_frames(path, scale):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if scale != 1.0:
            grey = cv2.resize(grey, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        frames.append(grey)
    cap.release()
    return np.stack(frames), fps


def near_plane_mask(shape):
    """Desk front, rail and cabinet: the lower ~55 % of a wide-view frame."""
    h, w = shape
    mask = np.zeros(shape, np.uint8)
    mask[int(0.42 * h):, :] = 255
    return mask


def register(frames, ref_index, n_features=3000):
    orb = cv2.ORB_create(n_features, fastThreshold=10)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    mask = near_plane_mask(frames[0].shape)
    ref = frames[ref_index]
    kp_r, des_r = orb.detectAndCompute(ref, mask)
    hs = np.full((len(frames), 3, 3), np.nan)
    inliers = np.zeros(len(frames), int)
    previous_h = np.eye(3)
    for i, frame in enumerate(frames):
        # Detect over the whole frame: in the close-up opening the near plane
        # sits elsewhere in the image, and RANSAC rejects far-room features.
        kp, des = orb.detectAndCompute(frame, None)
        if des is None or len(kp) < 20:
            continue
        pairs = matcher.knnMatch(des, des_r, k=2)
        good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.75 * n.distance]
        if len(good) < 15:
            continue
        src = np.float32([kp[m.queryIdx].pt for m in good])
        dst = np.float32([kp_r[m.trainIdx].pt for m in good])
        h, inl = cv2.findHomography(src, dst, cv2.RANSAC, 3.0, maxIters=4000)
        if h is None:
            continue
        hs[i] = h
        inliers[i] = int(inl.sum())
        previous_h = h
    return hs, inliers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video")
    parser.add_argument("out")
    parser.add_argument("--ref-time", type=float, default=15.75)
    parser.add_argument("--scale", type=float, default=0.5)
    args = parser.parse_args(argv)
    frames, fps = read_frames(args.video, args.scale)
    ref_index = int(round(args.ref_time * fps))
    print(f"{len(frames)} frames at {fps:.3f} fps, reference frame {ref_index}", flush=True)
    hs, inliers = register(frames, ref_index)
    warped = np.zeros_like(frames)
    shape = frames[0].shape[::-1]
    for i in range(len(frames)):
        if np.isfinite(hs[i]).all():
            warped[i] = cv2.warpPerspective(frames[i], hs[i], shape, flags=cv2.INTER_LINEAR,
                                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    np.savez_compressed(args.out, warped=warped, H=hs, inliers=inliers, fps=fps,
                        ref_index=ref_index, scale=args.scale)
    print("inliers per second (median):",
          [int(np.median(inliers[int(s * fps):int((s + 1) * fps)])) for s in range(int(len(frames) / fps))])
    return 0


if __name__ == "__main__":
    sys.exit(main())
