"""usage: python3 make_v2b.py PythonMaster2_swingup_V2.py PythonMaster2_swingup_V2b.py
Make the V2b script from PythonMaster2_swingup_V2.py (which stays unchanged): swing observer that
knows about the radio's timing noise and skips repeated (stale) readings."""
import re, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:80])
    s = s.replace(old, new)

rep('''# PythonMaster2_swingup_V2.py: COPY of PythonMaster2_swingup.py for the two V2 arms (2026-11-04).
''', '''# PythonMaster2_swingup_V2b.py: PythonMaster2_swingup_V2.py with a radio-aware swing observer
# (2026-11-04, after rig run swingup_double_20261104_153309: 5 attempts of same_4.0s all drifted off
# the plan after ~1.5 s). PythonMaster2_swingup_V2.py is not changed. V2b changes only:
#   - the swing observer trusts each angle reading less when the arm is fast (radio timing noise,
#     SWING_TIMING_NOISE_MS) and skips readings it has already used (radio gaps), class SwingKF,
#   - SWING_DISTURBANCE_RAD_S2 3 -> 10 and SWING_ACCEL_M_S2 5 -> 4 (tuned on the twin with that),
#   - a radio gap during a swing ends only that attempt; while settling, up to MAX_SETTLE_MISSES.
# Same plans, model, balance, recovery and logging as V2.
# PythonMaster2_swingup_V2.py: COPY of PythonMaster2_swingup.py for the two V2 arms (2026-11-04).
''')
rep('''SWING_DISTURBANCE_RAD_S2 = 3.0
''', '''SWING_DISTURBANCE_RAD_S2 = 10.0   # V2b: 3.0 in V2 (the timing-noise term below now does the smoothing)
# V2b (2026-11-04): the radio adds timing noise. On the rig run swingup_double_20261104_153309 a
# reading's real age was off by about 2-3 ms (spread), so a fast arm reads up to ~1-2 deg off; the
# original observer assumed 0.07 deg and chased that noise, and the cart over-corrected. The
# observer below now treats each reading as ENCODER_NOISE_COUNTS plus this timing noise times the
# arm's speed, and skips readings it has already used (radio gaps). 0 = the original V2 observer.
SWING_TIMING_NOISE_MS = 3.0
''')

rep('''def tvlqr(traj, r, P_final):''', '''class SwingKF:
    """Per arm (angle, rate) error covariance for the swing observer. The model does the prediction;
    this only sets how much each reading is trusted: less when the arm is fast (radio timing noise),
    none for a reading already used, more after a gap."""

    def __init__(self):
        T = SAMPLE_PERIOD_S
        self.F = np.array([[1.0, T], [0.0, 1.0]])
        s = SWING_DISTURBANCE_RAD_S2
        self.Qn = s * s * np.array([[T ** 4 / 4, T ** 3 / 2], [T ** 3 / 2, T * T]])
        self.r_enc = (ENCODER_NOISE_COUNTS * RAD_PER_COUNT) ** 2
        self.reset()

    def reset(self):
        H = np.array([[1.0, 0.0]])
        P = solve_discrete_are(self.F.T, H.T, self.Qn, np.array([[self.r_enc]]))
        self.P = [P.copy(), P.copy()]
        self.prev = [None, None]          # (raw counts, age ms) of the last reading of each sensor

    def is_new(self, n, raw, age):
        """A reading is new unless it repeats the last one (same counts and its age grew by a frame)."""
        p = self.prev[n]
        self.prev[n] = (raw, age)
        return p is None or raw != p[0] or age < p[1] + 8

    def gains(self, use1, use2, w1, w2):
        jt = SWING_TIMING_NOISE_MS / 1000.0
        R = (self.r_enc + (jt * w1) ** 2,
             2 * self.r_enc + jt * jt * (w1 * w1 + (w2 - w1) ** 2))
        out = []
        for n, use in enumerate((use1, use2)):
            P = self.F @ self.P[n] @ self.F.T + self.Qn
            if use:
                S = P[0, 0] + R[n]
                K = P[:, 0] / S
                P = P - np.outer(K, P[0, :])
            else:
                K = np.zeros(2)
            self.P[n] = 0.5 * (P + P.T)
            out.append(K)
        return out


def tvlqr(traj, r, P_final):''')

rep('''    L_swing = swing_observer_gain()
''', '''    L_swing = swing_observer_gain()
    kf = SwingKF() if SWING_TIMING_NOISE_MS > 0 else None
''')

rep('''            L = bal["L"] if mode == "BALANCE" else L_swing
''', '''            L = bal["L"] if mode == "BALANCE" else L_swing
            if kf is not None and mode != "BALANCE":
                pk_ = state.get("packet")
                ag = pk_.ages_ms if pk_ is not None else (0, 0, 0)
                new1 = ok1 and kf.is_new(0, raw1, ag[0])
                new2 = ok2 and kf.is_new(1, raw2, ag[state.get('j2', 1)])
                K1, K2 = kf.gains(new1, ok1 and ok2 and (new1 or new2), x_hat[1], x_hat[3])
                L = np.zeros((6, 3))
                L[0, 0], L[1, 0] = K1
                L[2, 1], L[3, 1] = K2
                L[4, 2] = 0.9
''')

# fresh covariance at each attempt start (the arms hang still, the estimate is good)
rep('''                    mode, mode_t0, k = "SWING", cycle_started, 0
''', '''                    mode, mode_t0, k = "SWING", cycle_started, 0
                    if kf is not None:
                        kf.reset()
''')
rep('''"script": "PythonMaster2_swingup_V2.py"''', '''"script": Path(__file__).name, "swing_timing_noise_ms": SWING_TIMING_NOISE_MS''')
rep('''SWING_ACCEL_M_S2 = 5.0
''', '''SWING_ACCEL_M_S2 = 4.0   # V2b: 5.0 in V2
''')
rep('''# Pick one here or on the command line:  python3 PythonMaster2_swingup_V2.py --path same_3.0s''',
    '''# Pick one here or on the command line:  python3 PythonMaster2_swingup_V2b.py --path same_3.0s''')
rep('''MAX_RUNNING_CONSECUTIVE_MISSES = 5   # unusable angle frames in a row allowed (prediction fills in); more stops the run (5 on the Pi)
''', '''MAX_RUNNING_CONSECUTIVE_MISSES = 5   # unusable angle frames in a row allowed (prediction fills in); more stops the run (5 on the Pi)
# V2b: during a swing, more than MAX_RUNNING_CONSECUTIVE_MISSES ends only that attempt (the cart
# brakes and the arms settle for a retry). While waiting or settling, up to this many are allowed
# before the run stops (the rig run of 2026-11-04 stopped on one 100 ms radio gap while settling).
MAX_SETTLE_MISSES = 12
''')
rep('''                misses_in_row += 1
                if misses_in_row > MAX_RUNNING_CONSECUTIVE_MISSES:
                    raise RuntimeError(f"{misses_in_row} unusable angle frames in a row; stopping. {state['msg']}")
''', '''                misses_in_row += 1
                if mode == "SWING" and misses_in_row > MAX_RUNNING_CONSECUTIVE_MISSES:
                    log("RUN", f"attempt {attempt}: {misses_in_row} angle frames missing at {k * SAMPLE_PERIOD_S:.2f} s; "
                               f"recovering. {state['msg']}")
                    mode, mode_t0 = "RECOVER", cycle_started
                    if run_logger is not None:
                        run_logger.event("abort", f"mode=SWING; k={k}; angle frames missing", cycle_started - run_started_at)
                limit = MAX_RUNNING_CONSECUTIVE_MISSES if mode == "BALANCE" else max(MAX_SETTLE_MISSES, MAX_RUNNING_CONSECUTIVE_MISSES)
                if misses_in_row > limit:
                    raise RuntimeError(f"{misses_in_row} unusable angle frames in a row; stopping. {state['msg']}")
''')
open(dst, "w", encoding="utf-8").write(s)
print("ok", dst)
