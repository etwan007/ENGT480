"""Write chosen gallery plans in PythonMaster2_swingup.py's trajectory format."""
import json, sys, datetime
import numpy as np
R0 = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}
LIMITS = {"track_m": 0.30, "speed_m_s": 0.75, "accel_m_s2": 8.0, "motor_top_speed_m_s": 1.0}
STYLE = {"same": "same way up", "opposite": "folded (arms turn opposite ways)", "whip": "whip (arm 2 makes 1.5 turns)",
         "whip_opp": "counter-whip"}
plans = {p["key"]: p for p in json.load(open(sys.argv[1]))}
picks = json.loads(sys.argv[2])     # {"same_3.0s": "same_3.0_16", ...}
outdir = sys.argv[3]
for name, key in picks.items():
    p = plans[key]
    Z = np.array(p["Z"]); a = np.array(p["a"]); T = p["T"]
    N = Z.shape[1] - 1; S = 10; seg = N // S; M = int(round(T / 0.05))
    # knots: a is linear between knots every 0.05 s -> knot values are a at multiples of 5 samples
    knots = a[::5]
    assert len(knots) == M + 1, (len(knots), M)
    pv = np.concatenate([knots] + [Z[:, s * seg] for s in range(1, S)])
    d = {"version": 1, "T": T, "dt": 0.01, "M": M, "S": S, "p": pv.tolist(), "z": Z.T.tolist(), "a_knots": a.tolist(),
         "model_ratios": R0, "effort": float(np.sum(a ** 2) * 0.01), "limits": LIMITS,
         "peak": {"track_m": float(np.abs(Z[4]).max()), "speed_m_s": float(np.abs(Z[5]).max()), "accel_m_s2": float(np.abs(a).max())},
         "made": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
         "style": STYLE[p["name"]], "gallery_key": key,
         "note": "made with CasADi/IPOPT (PUC Math/shared/nlink_cart.plan_team) for the V2-arm fit, not by --plan [SIM]"}
    json.dump(d, open(f"{outdir}/double_swingup_V2_{name}.json", "w"))
    print(name, key, T, d["peak"])
