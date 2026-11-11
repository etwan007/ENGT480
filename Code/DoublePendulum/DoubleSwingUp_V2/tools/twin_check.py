"""Run PythonMaster2_swingup_V2.py (unchanged apart from SWING_PATH) on the digital twin (emulated Arduino,
receiver, transmitters, stepper, nonlinear arms) in CPython, for each plan and a set of arm errors [SIM].

usage: twin_check.py <twinpy dir> <pkg dir> <plans comma list> <out.json> [seconds]
"""
import json, os, sys, math, tempfile, re, io, contextlib
from multiprocessing import Pool

TW, PKG, PLANS, OUT = sys.argv[1:5]
SECONDS = float(sys.argv[5]) if len(sys.argv) > 5 else 22.0
R = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}


def physical(scale=None):
    """Masses/lengths that give exactly the fitted ratios (J1 taken from the STL model, 8.956e-3 kg m^2)."""
    J1, L1, m1, m2 = 8.956e-3, 0.2, 0.0981, 0.1618
    c2 = R["P2"] * J1 / m2
    I2 = R["J2r"] * J1 - m2 * c2 ** 2
    c1 = (R["P1"] * J1 - m2 * L1) / m1
    I1 = J1 - m1 * c1 ** 2 - m2 * L1 ** 2
    a1 = {"m": m1, "c": c1, "I": I1, "L": L1, "b": R["B1"] * J1}
    a2 = {"m": m2, "c": c2, "I": I2, "L": 0.25, "b": R["B2"] * J1}
    for (arm, key), f in (scale or {}).items():
        (a1 if arm == 1 else a2)[key] *= f
    return a1, a2


CASES = {
    "nominal": {},
    "arm2 mass +4%": {(2, "m"): 1.04},
    "arm2 mass -4%": {(2, "m"): 0.96},
    "arm2 I +10%": {(2, "I"): 1.10},
    "arm2 I -10%": {(2, "I"): 0.90},
    "arm1 I +10%": {(1, "I"): 1.10},
    "arm2 CoM +3%": {(2, "c"): 1.03},
    "joint friction x3": {(2, "b"): 3.0},
    "pivot friction x3": {(1, "b"): 3.0},
    "TX2 on slot 3": "slot3",
    "radio loss 2% + 3 ms": "radio",
}


def one(job):
    plan, case, seed = job
    sys.path.insert(0, TW)
    import twin_hw, twin_runner
    if CASES[case] == "slot3":
        orig = twin_hw.Twin._rx_frame

        def swapped(self, ms):
            self.rx_samples[1], self.rx_samples[2] = self.rx_samples[2], self.rx_samples[1]
            try:
                orig(self, ms)
            finally:
                self.rx_samples[1], self.rx_samples[2] = self.rx_samples[2], self.rx_samples[1]
        twin_hw.Twin._rx_frame = swapped
    files = {}
    for n in os.listdir(PKG):
        p = os.path.join(PKG, n)
        if os.path.isfile(p) and n.endswith((".py", ".json")):
            files[n] = open(p, encoding="utf-8").read()
    s = files["PythonMaster2_swingup_V2.py"]
    s, k = re.subn(r'^SWING_PATH = "[^"]*"$', f'SWING_PATH = "{plan}"', s, count=1, flags=re.M)
    assert k == 1
    files["PythonMaster2_swingup_V2.py"] = s
    a1, a2 = physical(CASES[case] if isinstance(CASES[case], dict) else None)
    cfg = {"arms": 2, "arm1": a1, "arm2": a2, "run": {"seconds": SECONDS, "seed": seed, "after_s": 1.0}}
    if CASES[case] == "radio":
        cfg["radio"] = {"loss_pct": 2.0, "latency_ms": 3.0}
    root = tempfile.mkdtemp()
    lines, ev = [], []

    def emit(kind, data):
        if kind == "out":
            lines.append(data)
        elif kind == "event":
            ev.append(data)
    with contextlib.redirect_stdout(io.StringIO()):
        st = twin_runner.run(root, "DoublePendulum", files, "PythonMaster2_swingup_V2.py", cfg, emit)
    sys.stdout = sys.__stdout__
    text = "\n".join(l["s"] for l in lines)
    catches = len(re.findall(r"\*\*\* CATCH", text))
    attempts = len(re.findall(r"SWING-UP attempt", text))
    lost = len(re.findall(r"balance lost", text))
    aborts = re.findall(r"attempt \d+: ([^\n]*?) at ([0-9.]+) s; recovering", text)
    faults = [l["s"] for l in lines if "[FAULT]" in l["s"]][:2]
    slot = re.findall(r"joint encoder \(arm 2\) [^\n]*", text)
    # balanced at the end = caught and not lost afterwards
    last_catch = text.rfind("*** CATCH"); last_lost = max(text.rfind("balance lost"), text.rfind("recovering"))
    ok_end = catches > 0 and last_catch > last_lost
    return dict(plan=plan, case=case, seed=seed, attempts=attempts, catches=catches, lost=lost, balanced_at_end=ok_end,
                first_try=(attempts >= 1 and catches >= 1 and text.find("*** CATCH (attempt 1)") >= 0),
                aborts=aborts[:3], faults=faults, slot=slot[:1], status=str(st)[:80])


if __name__ == "__main__":
    plans = PLANS.split(",")
    jobs = [(p, c, s) for p in plans for c in CASES for s in (1, 2, 3)]
    res = []
    with Pool(4) as pool:
        for r in pool.imap_unordered(one, jobs):
            print(f"{r['plan']:11s} {r['case']:22s} attempts {r['attempts']} catches {r['catches']} first_try {r['first_try']} "
                  f"end_balanced {r['balanced_at_end']} aborts {r['aborts']} {r['faults']} {r['slot']}", flush=True)
            res.append(r)
            json.dump(res, open(OUT, "w"), indent=1)
