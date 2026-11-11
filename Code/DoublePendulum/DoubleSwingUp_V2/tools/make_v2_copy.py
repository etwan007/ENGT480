"""Make PythonMaster2_swingup_V2.py from the team's PythonMaster2_swingup.py (the original is only read).

usage: python3 make_v2_copy.py <original PythonMaster2_swingup.py> <output .py>
"""
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()


def sub(pattern, repl, flags=re.M, count=1):
    global s
    s, n = re.subn(pattern, repl, s, count=count, flags=flags)
    assert n == count, pattern


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (old, s.count(old))
    s = s.replace(old, new)


# 1. fitted model for the two V2 arms
sub(r"^SYSID_RATIOS = .*$",
    'SYSID_RATIOS = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}\n'
    "# ^ V2 arms (both), fitted 2026-11-04 from PendulumExcite run excite_20261104_093436 (2139 s):\n"
    "#   predicts both arms within about 2-3 deg over 3 s. The value above it in the original file was\n"
    "#   for the old arm 2. The plans below were made for exactly these numbers.")

# 2. which planned swing-up to run
sub(r'^TRAJECTORY_FILE = HERE / "double_swingup_trajectory.json"$',
    '''# Which planned swing-up to run. Each one is a file double_swingup_V2_<name>.json next to this
# script, made for the V2 arms (SYSID_RATIOS above) with |x| <= 0.30 m, |v| <= 0.75 m/s, 8 m/s^2.
# Pick one here or on the command line:  python3 PythonMaster2_swingup_V2.py --path same_3.0s
#   same_3.0s  same way up, 3.0 s, 5 cart reversals (fastest; least forgiving of model errors)
#   same_3.5s  same way up, 3.5 s, 6 cart reversals (medium)
#   same_4.0s  same way up, 4.0 s, 6 cart reversals (slowest; most forgiving)
#   whip_3.0s  whip: arm 2 makes 1.5 turns, 3.0 s, 5 reversals (fastest whip)
#   whip_3.5s  whip, 3.5 s, 4 reversals
#   whip_4.5s  whip, 4.5 s, 6 reversals
#   folded_4.5s  folded: the arms turn opposite ways, 4.5 s, 7 reversals
# --list prints the plan files found in this folder.
SWING_PATH = "same_4.0s"
TRAJECTORY_FILE = HERE / f"double_swingup_V2_{SWING_PATH}.json"''')

# 3. track the plans use (0.30 m, the swing aborts at SWING_CART_ABORT_M = 0.36 m)
sub(r"^PLAN_TRACK_M = 0\.25$", "PLAN_TRACK_M = 0.30   # V2 plans use +/-0.30 m (0.25 in the original)")

# 4. which receiver slot carries arm 2's joint encoder (TX2)
sub(r"^(ESP32_PORT, ESP32_BAUD = .*)$",
    r'''\1
# Receiver slot that carries the joint encoder (TX2): 2, 3, or "auto" (the first of slots 2 and 3
# that is valid when the script starts). On 2026-11-04 TX2 was sending as slot 3 (SENDER_ID 2);
# "auto" works either way and prints which slot it uses.
ARM2_CHANNEL = "auto"''')

rep('''    def read_angles(esp32):
        try:
            packet = reader.read_latest(esp32)
        except AngleProtocolError as exc:
            state["packet"] = None
            state["msg"] = str(exc)
            raise MissingAnglePacketError(str(exc)) from exc
        state["packet"] = packet
        state["msg"] = packet.missing_message((0, 1))
        return packet.angles
''', '''    def read_angles(esp32):
        """Returns (arm 1 counts, joint counts, the other slot's counts): the joint reading always comes
        second, from the slot chosen by ARM2_CHANNEL."""
        try:
            packet = reader.read_latest(esp32)
        except AngleProtocolError as exc:
            state["packet"] = None
            state["msg"] = str(exc)
            raise MissingAnglePacketError(str(exc)) from exc
        ang = packet.angles
        if "j2" not in state:
            if ARM2_CHANNEL in (2, 3):
                state["j2"] = int(ARM2_CHANNEL) - 1
                log("START", f"joint encoder (arm 2) read from receiver slot {ARM2_CHANNEL} (ARM2_CHANNEL)")
            elif ang[1] != 0xFFFF or ang[2] != 0xFFFF:
                state["j2"] = 1 if ang[1] != 0xFFFF else 2
                log("START", f"joint encoder (arm 2) found on receiver slot {state['j2'] + 1} (ARM2_CHANNEL = auto)")
        j = state.get("j2", 1)
        state["packet"] = packet
        state["msg"] = packet.missing_message((0, j))
        return ang[0], ang[j], ang[3 - j]
''')
rep("d2 = (min(ages[1], 50) + SENSOR_EXTRA_DELAY_MS) / 1000.0",
    "d2 = (min(ages[state.get('j2', 1)], 50) + SENSOR_EXTRA_DELAY_MS) / 1000.0")

# 5. --plan keeps the stored plan's starting branch (whip and folded plans start arm 2 at 3*pi or -pi)
rep("def plan_problem(r, T, M=None, S=10, X=None, V=None, A=None, Vtop=None):",
    "def plan_problem(r, T, M=None, S=10, X=None, V=None, A=None, Vtop=None, start=None):")
rep("    z0 = np.array([math.pi, 0, math.pi, 0, 0, 0.0])\n    zT = np.zeros(6)\n    nu = M + 1",
    "    z0 = np.array([math.pi, 0, math.pi, 0, 0, 0.0]) if start is None else np.array([start[0], 0, start[1], 0, 0, 0.0])\n"
    "    zT = np.zeros(6)\n    nu = M + 1")
rep('''    r0 = old.get("model_ratios")
    p = np.array(old["p"])
    T = old["T"]
    if not r0 or len(p) != len(plan_problem(r, T)["p0"]):''', '''    r0 = old.get("model_ratios")
    p = np.array(old["p"])
    T = old["T"]
    start = (old["z"][0][0], old["z"][0][2])
    if not r0 or len(p) != len(plan_problem(r, T, start=start)["p0"]):''')
rep("        pr = plan_problem(rr, T)\n        res, viol = plan_solve(pr, plan_refine(pr, p), maxiter=1000)",
    "        pr = plan_problem(rr, T, start=start)\n        res, viol = plan_solve(pr, plan_refine(pr, p), maxiter=1000)")

# 6. run log says which script and plan ran
rep('"script": "PythonMaster2_swingup.py",', '"script": "PythonMaster2_swingup_V2.py", "swing_path": SWING_PATH, "trajectory_file": TRAJECTORY_FILE.name,')
rep('log("START", f"plan {traj[\'N\'] * SAMPLE_PERIOD_S:.2f} s:', 'log("START", f"plan {TRAJECTORY_FILE.name} {traj[\'N\'] * SAMPLE_PERIOD_S:.2f} s:')

# 7. command line: --path NAME, --list
rep('''    ap = argparse.ArgumentParser(description="Double pendulum swing-up and balance.")
    ap.add_argument("--plan", action="store_true", help="make a new swing-up plan for the arm values in this file")
    if ap.parse_args().plan:
        plan(None)
    else:
        run()''', '''    ap = argparse.ArgumentParser(description="Double pendulum swing-up and balance (V2 arms).")
    ap.add_argument("--plan", action="store_true", help="re-make the chosen plan for the arm values in this file")
    ap.add_argument("--path", help="which plan to run, e.g. same_4.0s or whip_3.5s (default: SWING_PATH; --list shows all)")
    ap.add_argument("--list", action="store_true", help="list the plan files in this folder and stop")
    args = ap.parse_args()
    if args.list:
        for f in sorted(HERE.glob("double_swingup_V2_*.json")):
            d = json.loads(f.read_text())
            pk = d["peak"]
            print(f"{f.stem[len('double_swingup_V2_'):]:12s} {d['T']:.1f} s, cart within +/-{pk['track_m'] * 1000:.0f} mm, "
                  f"up to {pk['speed_m_s']:.2f} m/s and {pk['accel_m_s2']:.1f} m/s^2  {d.get('style', '')}")
        sys.exit(0)
    if args.path:
        SWING_PATH = args.path
        TRAJECTORY_FILE = HERE / f"double_swingup_V2_{SWING_PATH}.json"
    if not TRAJECTORY_FILE.exists() and not args.plan:
        sys.exit(f"{TRAJECTORY_FILE.name} not found next to this script (try --list)")
    if args.plan:
        plan(None)
    else:
        run()''')

head = '''# PythonMaster2_swingup_V2.py: COPY of PythonMaster2_swingup.py for the two V2 arms (2026-11-04).
# The original is not changed. Source: etwan007/ENGT480 experiment/legacy-pi-recovery ba019f9,
# LegacyPi/Code/DoublePendulum/PythonMaster2_swingup.py (the Pi's 2026-10-28 version).
# Changed here only:
#   - SYSID_RATIOS = the V2-arm fit (2026-11-04),
#   - plan files double_swingup_V2_<name>.json, chosen with SWING_PATH or --path (--list shows them),
#   - PLAN_TRACK_M 0.30 (the plans use +/-0.30 m; abort stays 0.36 m, soft limit 0.40 m),
#   - ARM2_CHANNEL: reads the joint encoder from receiver slot 2 or 3 (auto by default),
#   - --plan keeps the stored plan's starting branch (needed for the whip/folded plans).
# Everything else (tracking gains, balance, recovery, limits, logging) is the original's.
# Simulation only so far [SIM]: checked on the digital twin with the fitted arms.
'''
open(dst, "w", encoding="utf-8").write(head + s)
print("ok")
