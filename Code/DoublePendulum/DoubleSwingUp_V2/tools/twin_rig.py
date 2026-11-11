"""Run PythonMaster2_swingup_V2.py (or a patched copy) on the twin with a configurable radio [SIM].
usage from python: run_job(dict(pkg=..., plan=..., seed=..., seconds=..., radio={...}, patches=[(regex, repl)], case=name, keep_csv=dir))"""
import json, os, sys, re, io, glob, shutil, tempfile, contextlib
TW = os.environ.get("TWINPY", "twinpy")   # folder with twin_hw.py / twin_runner.py (from the TwinV3 artifact, see twin_check.py)
R = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}
def physical(scale=None):
    J1, L1, m1, m2 = 8.956e-3, 0.2, 0.0981, 0.1618
    c2 = R["P2"] * J1 / m2; I2 = R["J2r"] * J1 - m2 * c2 ** 2
    c1 = (R["P1"] * J1 - m2 * L1) / m1; I1 = J1 - m1 * c1 ** 2 - m2 * L1 ** 2
    a1 = {"m": m1, "c": c1, "I": I1, "L": L1, "b": R["B1"] * J1}
    a2 = {"m": m2, "c": c2, "I": I2, "L": 0.25, "b": R["B2"] * J1}
    for (arm, key), f in (scale or {}).items():
        (a1 if arm == 1 else a2)[key] *= f
    return a1, a2
def patch_radio(twin_hw, rq):
    """Bursty ESP-NOW model fitted to the 2026-11-04 rig run: blackouts (channel busy / retries) during
    which the TX queue keeps only the newest `qmax` packets; when the blackout ends they go out one
    every `gap` ms, so the first frames after a gap carry readings older than their radio age says."""
    import math
    def tx_event(self, i):
        tx = self.tx[i]; r = self.rng
        st = tx.setdefault("rq", {"until": -1.0, "queue": [], "flushing": False})
        tx["next"] += tx["period"]
        self._phys_to(self.t)
        angle = self._raw(i)
        t = self.t
        lam = rq["lam"][i]
        if t >= st["until"] and not st["flushing"] and r.random() < lam * tx["period"]:
            st["until"] = t + (rq["dmin"] + r.expovariate(1.0 / rq["dmean"])) / 1000.0
        if t < st["until"] or st["flushing"] or st["queue"]:
            st["queue"].append(angle)
            if len(st["queue"]) > rq["qmax"]:
                st["queue"].pop(0)
                tx["lost"] += 1
            if not st["flushing"]:
                st["flushing"] = True
                self.at(max(t, st["until"]) + rq["gap"] / 1000.0, self._rq_flush, i)
        elif r.random() * 100.0 >= rq["loss"]:
            lat = rq["lat"] / 1000.0 * r.uniform(0.6, 1.4)
            self.at(t + lat, self._radio_rx, (i, angle))
        else:
            tx["lost"] += 1
        self.at(tx["next"], self._tx_event, i)
    def flush(self, i):
        st = self.tx[i]["rq"]
        if st["queue"]:
            self._radio_rx((i, st["queue"].pop(0)))
        if st["queue"]:
            self.at(self.t + rq["gap"] / 1000.0, self._rq_flush, i)
        else:
            st["flushing"] = False
    twin_hw.Twin._tx_event = tx_event
    twin_hw.Twin._rq_flush = flush


def patch_radio_fifo(twin_hw, rf):
    """ESP-NOW as the TX firmware really uses it: 3-byte packets with no sequence number, esp_now_send
    queues each one, the radio sends them in order with MAC retries, and the receiver stamps the
    receive time. Channel blackouts (busy channel) make packets wait in the queue; when it clears the
    queue drains, so the receiver gets OLD readings with a fresh receive time (hidden delay)."""
    def tx_event(self, i):
        tx = self.tx[i]; r = self.rng
        st = tx.setdefault("rf", {"until": -1.0, "q": [], "busy": False, "tries": 0, "nb": 0.0})
        tx["next"] += tx["period"]
        self._phys_to(self.t)
        angle = self._raw(i)
        t = self.t
        if st["nb"] <= 0:
            st["nb"] = t + r.expovariate(rf["lam"][i]) if rf["lam"][i] > 0 else 1e9
        if t >= st["nb"]:
            st["until"] = t + (rf["dmin"] + r.expovariate(1.0 / rf["dmean"])) / 1000.0
            st["nb"] = st["until"] + (r.expovariate(rf["lam"][i]) if rf["lam"][i] > 0 else 1e9)
        if len(st["q"]) < rf["qmax"]:
            st["q"].append(angle)
        else:
            tx["lost"] += 1
        if not st["busy"]:
            st["busy"] = True; st["tries"] = 0
            self.at(t + attempt_time(r), self._rf_done, i)
        self.at(tx["next"], self._tx_event, i)
    def attempt_time(r):
        if rf.get("exp"):
            return (rf["a0"] + r.expovariate(1.0 / rf["a1"])) / 1000.0
        return (rf["a0"] + r.uniform(0.0, rf["a1"])) / 1000.0
    def done(self, i):
        tx = self.tx[i]; st = tx["rf"]; r = self.rng
        bad = self.t < st["until"] or r.random() < rf["fail"] / 100.0
        if not bad:
            self._radio_rx((i, st["q"].pop(0)))
            st["tries"] = 0
        else:
            st["tries"] += 1
            if st["tries"] > rf["retries"]:
                st["q"].pop(0); tx["lost"] += 1; st["tries"] = 0
        if st["q"]:
            self.at(self.t + attempt_time(r), self._rf_done, i)
        else:
            st["busy"] = False
    twin_hw.Twin._tx_event = tx_event
    twin_hw.Twin._rf_done = done


def run_job(job):
    sys.path.insert(0, TW)
    import twin_hw, twin_runner
    # undo any radio patch left by an earlier job in this worker process
    if not hasattr(twin_hw, "_ORIG_TX_EVENT"):
        twin_hw._ORIG_TX_EVENT = twin_hw.Twin._tx_event
    twin_hw.Twin._tx_event = twin_hw._ORIG_TX_EVENT
    if job.get("radio_q"):
        patch_radio(twin_hw, job["radio_q"])
    if job.get("radio_f"):
        patch_radio_fifo(twin_hw, job["radio_f"])
    files = {}
    for n in os.listdir(job["pkg"]):
        p = os.path.join(job["pkg"], n)
        if os.path.isfile(p) and n.endswith((".py", ".json")):
            files[n] = open(p, encoding="utf-8").read()
    s = files["PythonMaster2_swingup_V2.py"]
    s, k = re.subn(r'^SWING_PATH = "[^"]*"$', f'SWING_PATH = "{job["plan"]}"', s, count=1, flags=re.M); assert k == 1
    for pat, rep in job.get("patches", []):
        s, k = re.subn(pat, rep, s, count=1, flags=re.M)
        assert k == 1, pat
    files["PythonMaster2_swingup_V2.py"] = s
    a1, a2 = physical(job.get("scale"))
    cfg = {"arms": 2, "arm1": a1, "arm2": a2, "run": {"seconds": job.get("seconds", 30.0), "seed": job["seed"], "after_s": 1.0}}
    if job.get("radio"): cfg["radio"] = job["radio"]
    if job.get("motor"): cfg["motor"] = job["motor"]
    root = tempfile.mkdtemp()
    lines = []
    def emit(kind, data):
        if kind == "out": lines.append(data)
    with contextlib.redirect_stdout(io.StringIO()):
        st = twin_runner.run(root, "DoublePendulum", files, "PythonMaster2_swingup_V2.py", cfg, emit)
    sys.stdout = sys.__stdout__
    text = "\n".join(l["s"] for l in lines)
    catches = len(re.findall(r"\*\*\* CATCH", text)); attempts = len(re.findall(r"SWING-UP attempt", text))
    aborts = re.findall(r"attempt \d+: ([^\n]*?) at ([0-9.]+) s; recovering", text)
    lost = len(re.findall(r"balance lost", text))
    faults = [l["s"] for l in lines if "[FAULT]" in l["s"]][:2]
    csvs = glob.glob(os.path.join(root, "Code", "DoublePendulum", "run_logs", "*.csv"))
    out = None
    if job.get("keep_csv") and csvs:
        os.makedirs(job["keep_csv"], exist_ok=True)
        out = os.path.join(job["keep_csv"], f"{job.get('case','x')}_{job['plan']}_s{job['seed']}.csv")
        shutil.copy(csvs[0], out)
    shutil.rmtree(root, ignore_errors=True)
    return dict(case=job.get("case"), plan=job["plan"], seed=job["seed"], attempts=attempts, catches=catches, lost=lost,
                first_try="*** CATCH (attempt 1)" in text, aborts=aborts[:6], faults=faults, csv=out)
