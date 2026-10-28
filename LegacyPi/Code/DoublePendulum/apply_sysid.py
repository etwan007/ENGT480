#!/usr/bin/env python3
"""apply_sysid.py - copy the arm ratios analyze_sysid.py fitted into the double-pendulum scripts.

    python3 apply_sysid.py run_logs/sysid_<...>_double_<...>            newest analysis in that run
    python3 apply_sysid.py run_logs/sysid_A run_logs/sysid_B            average of several runs

Sets SYSID_RATIOS = {...} in PythonMaster2_swingup.py and PythonMaster2_auto_improved.py (if present
in this folder). The old file is kept as <name>.bak. Then re-plan the swing-up:
    python3 PythonMaster2_swingup.py --plan
"""
import json
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
TARGETS = ["PythonMaster2_swingup.py", "PythonMaster2_auto_improved.py"]
KEYS = ["P1", "P2", "J2r", "H", "B1", "B2"]


def ratios_from(folder):
    path = Path(folder)
    if path.is_dir():
        path = path / "analysis" / "results.json"
    if not path.exists():
        sys.exit(f"{path} not found: run  python3 analyze_sysid.py {folder}  first")
    res = json.loads(path.read_text())
    fit = res.get("double_fit")
    if not fit:
        sys.exit(f"{path} has no double-pendulum fit (was it a single-arm run?)")
    return {k: float(fit[k]) for k in KEYS}, res


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    fits = [ratios_from(f) for f in sys.argv[1:]]
    r = {k: sum(f[0][k] for f in fits) / len(fits) for k in KEYS}
    for _, res in fits:
        df = res["double_fit"]
        print(f"{res['run']}: modes {df['mode_freq_hz'][0]:.3f} / {df['mode_freq_hz'][1]:.3f} Hz, "
              f"fit {df['fit_pct_arm1']:.1f}% / {df['fit_pct_joint']:.1f}%, cart scale {df['cart_scale']:.3f}")
        if abs(df["cart_scale"] - 1) > 0.05:
            print("  note: cart scale is more than 5% from 1.00: check DISTANCE_PER_6400_STEPS_M")
    text = "SYSID_RATIOS = {" + ", ".join(f'"{k}": {r[k]:.6g}' for k in KEYS) + "}"
    print("\n" + text + "\n")
    done = 0
    for name in TARGETS:
        p = HERE / name
        if not p.exists():
            continue
        src = p.read_text()
        new, n = re.subn(r"^SYSID_RATIOS = .*$", text, src, count=1, flags=re.M)
        if n != 1:
            print(f"{name}: no SYSID_RATIOS line found, not changed")
            continue
        shutil.copy2(p, p.with_suffix(".py.bak"))
        p.write_text(new)
        print(f"{name}: updated (old file kept as {name}.bak)")
        done += 1
    if not done:
        sys.exit("No script updated: put this file in the same folder as PythonMaster2_swingup.py")
    print("\nNext:  python3 PythonMaster2_swingup.py --plan")


if __name__ == "__main__":
    main()
