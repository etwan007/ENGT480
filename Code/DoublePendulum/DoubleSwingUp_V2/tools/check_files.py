"""Check plan files with the V2 copy's own plan_problem (dynamics + limits) and load_trajectory."""
import importlib.util, json, sys, types, glob, os
import numpy as np
sys.modules.setdefault("serial", types.ModuleType("serial"))
sys.dont_write_bytecode = True
pkg = sys.argv[1]
spec = importlib.util.spec_from_file_location("v2", os.path.join(pkg, "PythonMaster2_swingup_V2.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
r = m.arm_ratios()
print("ratios", r)
for f in sorted(glob.glob(os.path.join(pkg, "double_swingup_V2_*.json"))):
    d = json.load(open(f))
    start = (d["z"][0][0], d["z"][0][2])
    pr = m.plan_problem(d["model_ratios"], d["T"], start=start)
    p = np.array(d["p"])[None]
    ceq = pr["ceq"](p); cin = pr["cin"](p)
    tr, a = pr["sim"](p)
    z = np.concatenate([tr[0, s, :-1] for s in range(pr["S"])] + [tr[0, -1, -1:]])
    m.TRAJECTORY_FILE = m.Path(f)
    traj = m.load_trajectory(r)
    bal = m.balance_design(r)
    Ks = m.tvlqr(traj, r, bal["P"])
    print(os.path.basename(f), f"unknowns {len(pr['p0'])}/{p.shape[1]}", f"max dyn mismatch {np.abs(ceq).max():.2e}",
          f"min limit margin {cin.min():+.2e}", f"state diff vs file {np.abs(z - np.array(d['z'])).max():.2e}",
          f"N {traj['N']}", f"|K| max {np.abs(Ks).max():.1f}")
