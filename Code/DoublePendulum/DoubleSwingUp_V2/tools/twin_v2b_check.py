"""V2 vs V2b on the twin [SIM]: rig-like radio (fitted to swingup_double_20261104_153309), clean radio, arm errors.
usage: TWINPY=<twinpy dir> python3 twin_v2b_check.py <dir with V2 package> <dir with V2b package as PythonMaster2_swingup_V2.py>"""
import sys, json, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from multiprocessing import Pool
from twin_rig import run_job
ORIG, FIX = sys.argv[1], sys.argv[2]
J2 = dict(lam=[8, 3, 0], dmin=4, dmean=10, qmax=8, retries=6, fail=30, a0=0.3, a1=1.8, exp=True)
STALL = {"tx_hz": 200.0, "latency_ms": 1.0, "loss_pct": 0.2, "stall_per_min": 30.0, "stall_ms": 80.0, "fail_pct": 0.0, "tx2_single": False}
COND = {"rig radio": dict(radio_f=J2), "clean": dict(), "clean, arm2 I +10%": dict(scale={(2, "I"): 1.10}), "clean, arm2 m -4%": dict(scale={(2, "m"): 0.96}),
        "80 ms stalls 30/min": dict(radio=STALL)}
CFG = {"V2": ORIG, "V2b": FIX}
SEEDS = {"rig radio": range(61, 79), "clean": range(61, 65), "clean, arm2 I +10%": range(61, 65), "clean, arm2 m -4%": range(61, 65), "80 ms stalls 30/min": range(61, 65)}
jobs = [dict(pkg=pk, plan="same_4.0s", seed=s, seconds=40.0, case=f"{cn}|{kn}", **kw) for cn, pk in CFG.items() for kn, kw in COND.items() for s in SEEDS[kn]]
if __name__ == "__main__":
    agg = {}; faults = {}
    with Pool(8) as pool:
        for r in pool.imap_unordered(run_job, jobs, chunksize=1):
            a = agg.setdefault(r["case"], [0, 0, 0, 0, 0, 0]); a[0] += r["attempts"]; a[1] += r["catches"]; a[2] += int(r["first_try"]); a[3] += r["lost"]; a[4] += int(bool(r["faults"])); a[5] += 1
            if r["faults"]: faults.setdefault(r["case"], r["faults"][0][:160])
    json.dump({"agg": agg, "faults": faults}, open("final_check.json", "w"), indent=1)
    for kn in COND:
        print(kn)
        for cn in CFG:
            a = agg[f"{cn}|{kn}"]
            print(f"   {cn:4s} runs {a[5]:2d}: attempts {a[0]:3d} catches {a[1]:3d} ({100*a[1]/max(1,a[0]):.0f}%), first try {a[2]}/{a[5]}, balance lost {a[3]}, run stopped by fault {a[4]}")
    print(json.dumps(faults, indent=1))
