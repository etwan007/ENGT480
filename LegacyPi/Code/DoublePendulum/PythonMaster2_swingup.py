"""PythonMaster2_swingup.py - swing the double pendulum up from hanging and balance it.

Run from Code/DoublePendulum on the Pi, next to angle_protocol.py, angle_packet_fallback.py,
run_csv_logger.py and double_swingup_trajectory.json:
    python3 PythonMaster2_swingup.py            swing up, catch, balance (Ctrl+C stops)
    python3 PythonMaster2_swingup.py --plan     re-plan the swing-up for the arm values below
                                                (do this after changing them; 1-10 minutes on a Pi)

Prompts:
    1. motor power OFF, cart centred, both arms hanging still  -> hanging calibration
    2. motor power ON, arms hanging still, hands clear, Enter  -> it swings up by itself

How it works
  1. PLAN (done once, stored in double_swingup_trajectory.json): a 2.5-second cart motion that takes
     both arms from hanging to upright inside the track, speed and acceleration limits (the stored
     plan: cart within +/-0.20 m, up to 0.65 m/s and 7 m/s^2). It is found by optimisation on the arm
     model, minimising how hard the cart has to accelerate. --plan re-does it, starting from the
     stored one, for new arm values.
  2. SWING: the cart plays the planned motion, and a time-varying LQR corrects it every 10 ms from
     the measured arm angles, so small model errors and a slightly different start don't add up.
  3. CATCH and BALANCE: at the end of the plan the arms are upright and still; the balance controller
     from PythonMaster2_auto_improved.py (stepper model, auto-trim, near-standstill) takes over.
  4. RECOVER: if the arms get too far from the plan or fall while balancing, the cart brakes, takes
     the swing out of the arms, waits for them to hang still, and tries again (up to MAX_ATTEMPTS).

Everything depends on the arm model. On the digital twin the swing-up succeeded in all 12 tests with
the arms up to about 5% different from the model (arm 2's mass +/-4%, its inertia +/-6%, arm 1's
inertia +8%, combined errors), pivot friction up to 3x, noisy encoders and a lossy radio. With arm 2's
mass 8% heavier, its inertia 15% higher or pivot friction 5x it failed: the 2.5 s plan is followed mostly open loop, and errors that
large add up faster than the feedback can fix within the track. So on the real rig:
    1. run PendulumSysID.py (DoublePendulum folder, --plan decay or full) and analyze_sysid.py,
    2. copy the fitted ratios into SYSID_RATIOS below,
    3. run  python3 PythonMaster2_swingup.py --plan  (makes a plan for those arms),
    4. then run it normally. If attempts keep failing at the same point, the model is still off.
Balancing after the catch is far less sensitive (see PythonMaster2_auto_improved.py).

CSV: same columns as PythonMaster2_auto.py; the "force" columns hold cart acceleration in m/s^2 and
the message column says which phase each row is in.
"""

import argparse
import json
import math
import struct
import sys
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are

HERE = Path(__file__).resolve().parent

# =====================================================================================
# 1. ARM MODEL  (same meaning as in PythonMaster2_auto_improved.py)
# The swing-up is planned from these. Unlike balancing, the swing-up needs them close (within about
# 5%): the plan is a 2.5-second motion that the feedback only trims. Measured values from
# PendulumSysID.py (SYSID_RATIOS below) are the most reliable way to get there.
# After changing any of them, run with --plan to make a new trajectory.
# =====================================================================================

# Arm 1 alone (rod, weights, TX1 board, battery, screws), kg.
ARM1_MASS_KG = 0.207
# Main pivot axis to arm 1's centre of mass, m (balance the removed arm on a knife edge).
ARM1_COM_M = 0.087
# Arm 1's moment of inertia about its own centre of mass, kg m^2.
ARM1_INERTIA_KG_M2 = 0.00123324992
# Main pivot axis to arm 2's joint axis, m.
ARM1_LENGTH_M = 0.2
# Arm 2 alone, kg.
ARM2_MASS_KG = 0.210
# Arm 2's joint axis to its centre of mass, m.
ARM2_COM_M = 0.106
# Arm 2's moment of inertia about its own centre of mass, kg m^2.
ARM2_INERTIA_KG_M2 = 0.00197563667
# Viscous friction at the pivot and the joint, N m s/rad. Raise them if the arms lose their swing
# much faster than the model (the plan then pushes a little harder to make up for it).
ARM1_PIVOT_FRICTION = 0.0009
ARM2_JOINT_FRICTION = 0.0001
# Optional: ratios fitted by analyze_sysid.py (they replace every value above). Most accurate option.
# Example: SYSID_RATIOS = {"P1": 5.357, "P2": 1.985, "J2r": 0.3866, "H": 0.3972, "B1": 0.012, "B2": 0.0098}
# Measured on the rig on 2026-10-16. double_swingup_trajectory.json was planned for exactly these.
SYSID_RATIOS = {"P1": 5.16428, "P2": 1.80161, "J2r": 0.39142, "H": 0.39247, "B1": 0.0, "B2": 0.11396}
G = 9.81   # m/s^2

# =====================================================================================
# 2. SWING-UP PLAN (used by --plan)
# =====================================================================================

# File holding the planned motion. --plan writes it; a normal run reads it.
TRAJECTORY_FILE = HERE / "double_swingup_trajectory.json"

# Duration of the swing-up, s. Shorter needs harder accelerations and higher speeds; longer needs
# more swings and more travel. On the twin, 2.5 s plans tolerated model errors better than 2.0 s
# ones. The planner tries this first and then nearby durations if it cannot find a plan.
PLAN_DURATION_S = 2.5

# Track the plan may use, m from the start. Keep it at least 0.06 m inside SWING_CART_ABORT_M: the
# feedback needs room around the plan to correct model errors. Less track needs more acceleration
# or a longer plan.
PLAN_TRACK_M = 0.25

# Fastest cart speed the plan may use, m/s. Steppers lose torque at speed; if the cart misses
# steps during the swing, lower this and re-plan (the swing then needs more acceleration or travel).
# 0.75 (was 0.65): with the measured joint friction (B2 = 0.169) no plan exists at 0.65 / 7.0.
PLAN_SPEED_M_S = 0.75

# Hardest acceleration the plan may use from standstill, m/s^2. Lower it if the stepper skips;
# raise it (up to ~12) if no plan can be found within the other limits. Keep it well below
# CART_ACCEL_MAX_M_S2: the feedback adds its corrections on top.
# 8.0 (was 7.0), for the same reason. On the twin the stepper dropped ~20 steps (2 mm) per swing
# at these limits, which the feedback absorbed.
PLAN_ACCEL_M_S2 = 8.0

# --plan walks the stored plan over to new arm values in steps, halving the step whenever one
# fails. It gives up once a step would be smaller than this fraction of the whole change.
# Smaller tries harder (more steps, each about a minute on a Pi); larger gives up sooner.
CONTINUATION_MIN_STEP = 0.1

# A stepper's torque falls as it speeds up. The plan only asks for PLAN_ACCEL_M_S2 x (1 - speed /
# MOTOR_TOP_SPEED_M_S) when speeding up further, and full acceleration when slowing down. Set this
# to roughly the speed at which your motor stalls with no load; lower it if the cart skips steps
# during fast parts of the swing (then re-plan). The twin's motor is modelled with 1.0 m/s.
MOTOR_TOP_SPEED_M_S = 1.0

# =====================================================================================
# 3. SWING-UP TRACKING (time-varying LQR around the plan)
# Same idea as the balance weights: the size of each error that counts as "a lot".
# Smaller = corrected harder.
# =====================================================================================

# Arm angle error from the plan that counts as large, degrees. Smaller follows the plan more
# tightly but reacts harder to sensor noise and model errors. 3-10.
SWING_ANGLE_DEG = 5.0
# Arm angular speed error, deg/s. 50-300.
SWING_RATE_DEG_S = 120.0
# Cart position error from the plan, m. Smaller keeps the cart on its planned path (safer near
# the track ends) but leaves less freedom to fix the arms.
SWING_CART_M = 0.25
# Cart speed error from the plan, m/s.
SWING_CART_SPEED_M_S = 0.3
# Extra cart acceleration on top of the plan that counts as large, m/s^2. Larger lets the feedback
# fight harder to stay on the plan (more robust to model errors, but more motor stress).
SWING_ACCEL_M_S2 = 5.0

# Give up on an attempt if either arm gets this far from the planned angle, degrees. Past ~40 deg
# the feedback cannot bring it back; the cart then brakes and the arms are settled for a retry.
SWING_ABORT_DEG = 35.0

# Also give up on an attempt if the cart gets this far from the centre during the swing, m. The
# plan uses up to PLAN_TRACK_M; the feedback needs room beyond that. Keep it at least 0.03 m inside
# TRACK_SOFT_LIMIT_M so the cart can brake before the motor is cut.
SWING_CART_ABORT_M = 0.36

# Number of swing-up attempts before giving up (each failed one is followed by settling the arms).
MAX_ATTEMPTS = 30

# =====================================================================================
# 4. BALANCE AFTER THE CATCH
# Copied from PythonMasterDouble-Claude.py on the Pi (2026-10-21), which balances the real rig
# well. See that file's comments for what each one does.
# =====================================================================================
# Joint friction used for the balance design only. The rig balances best with 0.03; the swing
# plan and its tracking keep the measured 0.169 in SYSID_RATIOS (the friction in big swings).
# None uses SYSID_RATIOS for the balance too.
BALANCE_B2 = 0.03
# Delay assumed by the balance design, ms (the Pi file uses 9). The swing uses CONTROL_LATENCY_MS below.
BALANCE_LATENCY_MS = 9.0
LQR_ARM1_ANGLE_DEG = 2.0     # smaller = stiffer arm 1, more cart motion
LQR_ARM2_ANGLE_DEG = 2.0     # smaller = stiffer arm 2; keep at or below arm 1's
LQR_ARM_RATE_DEG_S = 60.0    # smaller damps wobble harder but amplifies encoder noise
LQR_CART_M = 0.05            # smaller pulls the cart back to centre faster
LQR_CART_SPEED_M_S = 0.3     # smaller = slower, smoother cart (0.3 on the Pi)
LQR_CART_ACCEL_M_S2 = 1.3    # main calm-vs-stiff knob: smaller = stiller cart, larger = stronger recovery (1.3 on the Pi)
ENCODER_NOISE_COUNTS = 3.0   # raise if the cart jitters while balanced
ARM_DISTURBANCE_RAD_S2 = 1.0  # raise if the balance reacts late to pushes (1.0 on the Pi)
AUTO_TRIM = True             # find the true balance point while balancing
TRIM_TIME_CONSTANT_S = 6.0   # how fast the trim follows, s
TRIM_START_DELAY_S = 1.5     # after the catch, s
TRIM_MAX_DEG = 3.0           # largest trim per reference, degrees
ARM1_OFFSET_DEG = 0.0        # starting offsets (the run prints the trims it found at Ctrl+C)
JOINT2_OFFSET_DEG = 0.0
SPEED_DEADBAND_M_S = 0.0003  # tiny speeds hold position with a 1-step dither instead of creeping

# =====================================================================================
# 5. ESTIMATOR DURING THE SWING
# =====================================================================================
# The arm speeds are predicted with the full nonlinear model and corrected from the encoders.
# This is how hard the real arms may differ from the model during the swing, rad/s^2. Larger
# trusts the encoders more: follows the real arms closely, but the radio's 1-2 ms timing jitter
# on fast-moving arms then turns into noisy speed estimates and a shaky swing (on the twin, 8 or
# more failed whenever the radio added a few ms of delay). Smaller trusts the model: smoother,
# but slower to notice the real arms drifting from it. 1-5 worked best on the twin. If the real
# arms keep drifting from the plan in the same way every attempt, the model is off: re-measure
# the arms (or use SYSID_RATIOS) before raising this.
SWING_DISTURBANCE_RAD_S2 = 3.0

# =====================================================================================
# 6. RECOVERY BETWEEN ATTEMPTS
# =====================================================================================
# Cart acceleration used to take the swing out of the arms, m/s^2 (limit). Larger settles them
# faster but moves the cart more. 2-5.
RECOVER_ACCEL_M_S2 = 4.0
# Damping gain, m/s^2 per unit of arm "swing power". Larger damps harder until the limit above.
RECOVER_GAIN = 0.20
# The arms count as hanging still when both have stayed within this many degrees of hanging for the
# last STILL_TIME_S seconds. The plan starts from rest, so a swing left over makes the attempt start
# off the plan. Raise it a little only if noisy encoders keep the script waiting.
STILL_ANGLE_DEG = 2.5
STILL_TIME_S = 1.0
# Give up waiting for the arms to settle after this long, s.
RECOVER_TIMEOUT_S = 40.0

# =====================================================================================
# 7. LIMITS, TIMING AND HARDWARE
# =====================================================================================
# Fastest speed ever sent, m/s. Must be above PLAN_SPEED_M_S plus room for corrections.
CART_SPEED_MAX_M_S = 1.0
# Hardest acceleration ever sent, m/s^2 (from standstill; it falls with speed like the plan's, see
# MOTOR_TOP_SPEED_M_S). Must be above PLAN_ACCEL_M_S2 to leave room for corrections, but asking a
# stepper for more than it can do makes it skip steps, and then the step count lies about where the
# cart is. Lower it if you hear skipping during the swing.
CART_ACCEL_MAX_M_S2 = 16.0
# The motor stops for good if the cart gets this far from where it started, m (inside the limit switches).
TRACK_SOFT_LIMIT_M = 0.4
# While balancing, an arm leaning further than this counts as a fall (-> recover and retry), degrees.
BALANCE_LOST_DEG = 30.0
# Time from the start of a control cycle to the cart actually changing speed, ms: the Pi and USB
# (1-2 ms) and the motor and belt catching up (about 5 ms). The swing-up is sensitive to it. If the
# swing drifts off the plan in its second half, try +/-2 ms.
# Used for the swing only; the balance uses BALANCE_LATENCY_MS.
CONTROL_LATENCY_MS = 8.0

# How old an angle reading is beyond the age the receiver reports, ms: the radio hop from the
# transmitter (about 1 ms) plus the USB transfer (about 0.5 ms). The observer compares each reading
# with where it thinks the arm was at that moment, which matters when the arms turn at 10 rad/s.
SENSOR_EXTRA_DELAY_MS = 1.5
# Largest believable change in the Arduino's step count between two replies (1 m/s = 100 steps per
# cycle). A bigger jump is a bad reply, not cart motion, and stops the run.
MAX_STEPS_PER_REPLY = 400
SAMPLE_PERIOD_S = 0.01       # one cycle per receiver frame (10 ms); change only with the receiver firmware
HANGING_SAMPLES = 50         # readings averaged for the hanging calibration; more = steadier reference
SAMPLE_SPACING_S = 0.02      # pause between those readings, s
MAX_HANGING_DEVIATION_COUNTS = 80    # arms must hang this still while calibrating (counts; 1 count = 0.022 deg)
MAX_RUNNING_CONSECUTIVE_MISSES = 5   # unusable angle frames in a row allowed (prediction fills in); more stops the run (5 on the Pi)
MAX_SETUP_CONSECUTIVE_MISSES = 5     # the same during calibration
MAX_HANGING_TOTAL_MISSES = 5         # total unusable frames allowed during calibration
ESP32_PORT, ESP32_BAUD = "/dev/ttyUSB0", 921600        # receiver (protocol v1); change if the Pi names it differently
ARDUINO_PORT, ARDUINO_BAUD = "/dev/ttyACM0", 115200    # Arduino Mega running ArduinoSlave.ino
DISTANCE_PER_6400_STEPS_M = 0.638175  # cart travel for 6400 steps (measured; as in the repo)
STATUS_INTERVAL_S = 0.5      # how often the status line prints, s
SHOW_BALANCING_LOGS = True   # False hides the status lines (faults still print)
SAVE_RUN_CSV = True          # one CSV per run in run_logs/, written off the control thread
RUN_LOG_DIRECTORY = HERE / "run_logs"
# When the run stops (Ctrl+C or a fault), the cart is braked to a standstill at this rate before
# the motor is switched off, m/s^2. A switched-off stepper lets a moving cart coast into the end.
STOP_BRAKE_M_S2 = 10.0
# ... and then held in place for this long, s, while the arms swing down: with the motor off, arms
# still swinging hard can push the free cart into a limit switch. A second Ctrl+C cuts it short.
# 0 switches the motor off as soon as the cart has stopped.
STOP_HOLD_S = 6.0

# -------------------------------------------------------------------------------------
ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
RAD_PER_COUNT = 2 * math.pi / ENCODER_COUNTS
M_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_STEP_HZ = 125000.0
started_at = time.monotonic()


def log(tag, message):
    if not SHOW_BALANCING_LOGS and tag == "RUN":
        return
    print(f"[{time.monotonic() - started_at:7.2f}s] [{tag}] {message}", flush=True)


# ------------------------------------------------------------------ model
def arm_ratios():
    if SYSID_RATIOS:
        r = {k: float(v) for k, v in SYSID_RATIOS.items()}
        r.setdefault("B1", 0.0)
        r.setdefault("B2", 0.0)
        return r
    J1 = ARM1_INERTIA_KG_M2 + ARM1_MASS_KG * ARM1_COM_M ** 2 + ARM2_MASS_KG * ARM1_LENGTH_M ** 2
    p1 = ARM1_MASS_KG * ARM1_COM_M + ARM2_MASS_KG * ARM1_LENGTH_M
    p2 = ARM2_MASS_KG * ARM2_COM_M
    J2 = ARM2_INERTIA_KG_M2 + ARM2_MASS_KG * ARM2_COM_M ** 2
    h = ARM2_MASS_KG * ARM1_LENGTH_M * ARM2_COM_M
    return {"P1": p1 / J1, "P2": p2 / J1, "J2r": J2 / J1, "H": h / J1,
            "B1": ARM1_PIVOT_FRICTION / J1, "B2": ARM2_JOINT_FRICTION / J1}


def f(z, a, r):
    """Nonlinear dynamics. z = [..., 6] = theta1, rate1, theta2, rate2, cart, cart speed (angles from
    upright, absolute, repo sign convention); a = cart acceleration."""
    th1, w1, th2, w2, x, v = (z[..., i] for i in range(6))
    d = th1 - th2
    c, s = np.cos(d), np.sin(d)
    rel = w2 - w1
    R1 = G * r["P1"] * np.sin(th1) - r["P1"] * np.cos(th1) * a - r["H"] * s * w2 * w2 - r["B1"] * w1 + r["B2"] * rel
    R2 = G * r["P2"] * np.sin(th2) - r["P2"] * np.cos(th2) * a + r["H"] * s * w1 * w1 - r["B2"] * rel
    M12 = r["H"] * c
    det = r["J2r"] - M12 * M12
    return np.stack([w1, (r["J2r"] * R1 - M12 * R2) / det, w2, (R2 - M12 * R1) / det, v,
                     a * np.ones_like(v)], axis=-1)


def rk4(z, a0, a1, dt, r):
    am = 0.5 * (a0 + a1)
    k1 = f(z, a0, r)
    k2 = f(z + dt / 2 * k1, am, r)
    k3 = f(z + dt / 2 * k2, am, r)
    k4 = f(z + dt * k3, a1, r)
    return z + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def kick(z, dv, r):
    """The cart speed jumps by dv (a stepper changes speed at once): each arm's angular speed jumps too."""
    th1, th2 = z[..., 0], z[..., 2]
    M12 = r["H"] * np.cos(th1 - th2)
    det = r["J2r"] - M12 * M12
    q1, q2 = -r["P1"] * np.cos(th1) * dv, -r["P2"] * np.cos(th2) * dv
    z = z.copy()
    z[..., 1] += (r["J2r"] * q1 - M12 * q2) / det
    z[..., 3] += (q2 - M12 * q1) / det
    z[..., 5] += dv
    return z


def coast(z, dt, r, n=2):
    for _ in range(n):
        if dt > 0:
            z = rk4(z, 0.0, 0.0, dt / n, r)
    return z


def step(z, u_prev, u, r, T=SAMPLE_PERIOD_S, tau=None):
    """One control period with the stepper's staircase speed. u is the speed change decided at this
    sample, divided by T (an equivalent acceleration); it reaches the cart tau after the sample. If tau
    is longer than one period, the previous sample's change lands in this period instead."""
    tau = CONTROL_LATENCY_MS / 1000.0 if tau is None else tau
    if tau < T:
        z = coast(z, tau, r)
        z = kick(z, u * T, r)
        return coast(z, T - tau, r)
    t1 = tau - T
    z = coast(z, t1, r)
    z = kick(z, u_prev * T, r)
    return coast(z, T - t1, r)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def linearize(z, a_prev, a, r, tau=None):
    """Discrete A (7x7, state + previous acceleration) and B (7x1) of step() at a point."""
    eps = 1e-6
    base = np.concatenate([z, [a_prev]])
    A = np.zeros((7, 7))
    pts = np.vstack([base + eps * e for e in np.eye(7)] + [base - eps * e for e in np.eye(7)])
    zz = step(pts[:, :6], pts[:, 6], a, r, tau=tau)
    A[:6] = ((zz[:7] - zz[7:]) / (2 * eps)).T
    B = np.zeros((7, 1))
    zp = step(np.vstack([z, z]), np.array([a_prev, a_prev]), np.array([a + eps, a - eps]), r, tau=tau)
    B[:6, 0] = (zp[0] - zp[1]) / (2 * eps)
    B[6, 0] = 1.0                # next "previous acceleration" is this one
    return A, B


def balance_design(r):
    r = dict(r)
    if BALANCE_B2 is not None:
        r["B2"] = BALANCE_B2
    A = np.zeros((6, 6))
    A[0, 1] = A[2, 3] = A[4, 5] = 1.0
    zero = np.zeros(6)
    eps = 1e-6
    for i in range(6):
        dz = np.zeros(6)
        dz[i] = eps
        A[:, i] = (f(zero + dz, 0.0, r) - f(zero - dz, 0.0, r)) / (2 * eps)
    Ad, Bd = linearize(zero, 0.0, 0.0, r, tau=BALANCE_LATENCY_MS / 1000.0)
    q = [1 / math.radians(LQR_ARM1_ANGLE_DEG) ** 2, 1 / math.radians(LQR_ARM_RATE_DEG_S) ** 2,
         1 / math.radians(LQR_ARM2_ANGLE_DEG) ** 2, 1 / math.radians(LQR_ARM_RATE_DEG_S) ** 2,
         1 / LQR_CART_M ** 2, 1 / LQR_CART_SPEED_M_S ** 2, 0.0]
    Q, R = np.diag(q), np.array([[1 / LQR_CART_ACCEL_M_S2 ** 2]])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    # Kalman gain for balancing (linear model at upright)
    T = SAMPLE_PERIOD_S
    Phi = Ad[:6, :6]
    C = np.zeros((3, 6))
    C[0, 0] = C[1, 2] = C[2, 4] = 1.0
    w = ARM_DISTURBANCE_RAD_S2 * T
    Qn = np.diag([(w * T / 2) ** 2, w ** 2, (w * T / 2) ** 2, w ** 2, (M_PER_STEP / 4) ** 2, (M_PER_STEP / T / 4) ** 2])
    sa = ENCODER_NOISE_COUNTS * RAD_PER_COUNT
    Rn = np.diag([sa ** 2, (sa * 1.4) ** 2, (M_PER_STEP / math.sqrt(12)) ** 2])
    Pk = solve_discrete_are(Phi.T, C.T, Qn, Rn)
    L = Pk @ C.T @ np.linalg.inv(C @ Pk @ C.T + Rn)
    return {"K": K, "P": P, "L": L, "C": C}


def swing_observer_gain():
    """Constant gain for the swing: per arm, angle -> (angle, rate) from a Kalman filter on
    angle'' = disturbance. The nonlinear model does the prediction."""
    T = SAMPLE_PERIOD_S
    F = np.array([[1, T], [0, 1]])
    s = SWING_DISTURBANCE_RAD_S2
    Qn = s * s * np.array([[T ** 4 / 4, T ** 3 / 2], [T ** 3 / 2, T * T]])
    Rn = np.array([[(ENCODER_NOISE_COUNTS * RAD_PER_COUNT) ** 2]])
    H = np.array([[1.0, 0.0]])
    P = solve_discrete_are(F.T, H.T, Qn, Rn)
    g = (P @ H.T @ np.linalg.inv(H @ P @ H.T + Rn))[:, 0]
    L = np.zeros((6, 3))
    L[0, 0], L[1, 0] = g
    L[2, 1], L[3, 1] = g
    L[4, 2] = 0.9
    return L


def tvlqr(traj, r, P_final):
    """Time-varying LQR gains along the plan, ending in the balance controller's cost."""
    z_ref, a_ref = traj["z"], traj["a"]
    N = len(a_ref)
    Q = np.diag([1 / math.radians(SWING_ANGLE_DEG) ** 2, 1 / math.radians(SWING_RATE_DEG_S) ** 2,
                 1 / math.radians(SWING_ANGLE_DEG) ** 2, 1 / math.radians(SWING_RATE_DEG_S) ** 2,
                 1 / SWING_CART_M ** 2, 1 / SWING_CART_SPEED_M_S ** 2, 0.0])
    R = np.array([[1 / SWING_ACCEL_M_S2 ** 2]])
    P = P_final.copy()
    Ks = [None] * N
    for k in range(N - 1, -1, -1):
        a_prev = a_ref[k - 1] if k > 0 else 0.0
        A, B = linearize(z_ref[k], a_prev, a_ref[k], r)
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P = Q + A.T @ P @ (A - B @ K)
        P = 0.5 * (P + P.T)
        Ks[k] = K
    return Ks


# ------------------------------------------------------------------ planning (--plan)
def plan_problem(r, T, M=None, S=10, X=None, V=None, A=None, Vtop=None):
    Vtop = MOTOR_TOP_SPEED_M_S if Vtop is None else Vtop
    X = PLAN_TRACK_M if X is None else X
    V = PLAN_SPEED_M_S if V is None else V
    A = PLAN_ACCEL_M_S2 if A is None else A
    dt = SAMPLE_PERIOD_S
    N = int(round(T / dt / S)) * S
    T = N * dt
    M = M or int(round(T / 0.05))
    seg = N // S
    tg = np.arange(N + 1) * dt
    W = np.zeros((N + 1, M + 1))
    for i, t in enumerate(tg):
        j = min(int(t / (T / M)), M - 1)
        s_ = t / (T / M) - j
        W[i, j] += 1 - s_
        W[i, j + 1] += s_
    z0 = np.array([math.pi, 0, math.pi, 0, 0, 0.0])
    zT = np.zeros(6)
    nu = M + 1

    def sim(p):
        u = p[:, :nu]
        Z = p[:, nu:].reshape(len(p), S - 1, 6)
        a = u @ W.T
        z = np.concatenate([np.repeat(z0[None, None], len(p), 0), Z], axis=1)
        out = np.zeros((len(p), S, seg + 1, 6))
        out[:, :, 0] = z
        for k in range(seg):
            idx = np.arange(S) * seg + k
            z = rk4(z, a[:, idx], a[:, idx + 1], dt, r)
            out[:, :, k + 1] = z
        return out, a

    def ceq(p):
        tr, _ = sim(p)
        ends, starts = tr[:, :, -1], tr[:, :, 0]
        return np.concatenate([(ends[:, :-1] - starts[:, 1:]).reshape(len(p), -1), ends[:, -1] - zT], axis=1)

    def cin(p):
        tr, a = sim(p)
        xs = tr[..., 4].reshape(len(p), -1)[:, ::2]
        vs = tr[..., 5].reshape(len(p), -1)[:, ::2]
        # speed on the time grid (segment ends overlap the next segment's start)
        vg = np.concatenate([tr[:, :, :-1, 5].reshape(len(p), -1), tr[:, -1, -1:, 5]], axis=1)
        av = np.sqrt(vg * vg + 1e-4)
        torque = A * (1 - av / Vtop) - a * vg / av        # accelerating further only up to the torque left
        return np.concatenate([X - xs, X + xs, V - vs, V + vs, torque[:, ::2]], axis=1)

    def jac(fun):
        def j(p):
            P = np.vstack([p, p + 1e-6 * np.eye(len(p))])
            F = fun(P)
            return ((F[1:] - F[0]).T) / 1e-6
        return j

    def cost(p):
        a = (p[None, :nu] @ W.T)[0]
        return float(np.sum(a * a) * dt)

    def grad(p):
        g = np.zeros_like(p)
        a = p[:nu] @ W.T
        g[:nu] = 2 * dt * (W.T @ a)
        return g

    Zg = np.array([z0 + (zT - z0) * (s_ / S) for s_ in range(1, S)])
    Zg[:, 1] = Zg[:, 3] = -math.pi / T
    p0 = np.concatenate([np.zeros(nu), Zg.ravel()])
    return dict(sim=sim, ceq=ceq, cin=cin, jac=jac, cost=cost, grad=grad, p0=p0, nu=nu, M=M, S=S, N=N, T=T,
                bounds=[(-A, A)] * nu + [(None, None)] * (S - 1) * 6, X=X, V=V, A=A, Vtop=Vtop)


def plan_solve(pr, p0, maxiter=500):
    from scipy.optimize import minimize
    cons = [dict(type="eq", fun=lambda p: pr["ceq"](p[None])[0], jac=pr["jac"](pr["ceq"])),
            dict(type="ineq", fun=lambda p: pr["cin"](p[None])[0], jac=pr["jac"](pr["cin"]))]
    res = minimize(pr["cost"], p0, jac=pr["grad"], constraints=cons, bounds=pr["bounds"], method="SLSQP",
                   options=dict(maxiter=maxiter, ftol=1e-8))
    viol = float(np.abs(pr["ceq"](res.x[None])[0]).max())
    return res, viol


def plan_refine(pr, p0, max_nfev=200):
    """Fast Gauss-Newton pass from a nearby plan (e.g. the stored one for slightly different arms):
    joins the shooting segments, keeps the limits, and keeps the effort low. SLSQP then polishes it."""
    from scipy.optimize import least_squares
    nu, dt = pr["nu"], SAMPLE_PERIOD_S

    def res(P):
        P = np.atleast_2d(P)
        _, a = pr["sim"](P)
        return np.concatenate([100 * pr["ceq"](P), 0.05 * math.sqrt(dt) * a, 100 * np.minimum(pr["cin"](P), 0)], axis=1)

    def jac(p):
        F = res(np.vstack([p, p + 1e-6 * np.eye(len(p))]))
        return ((F[1:] - F[0]).T) / 1e-6
    lb = np.full(len(p0), -np.inf)
    ub = np.full(len(p0), np.inf)
    lb[:nu], ub[:nu] = -pr["A"], pr["A"]
    p0 = np.clip(p0, lb + 1e-9, ub - 1e-9)
    out = least_squares(lambda p: res(p[None])[0], p0, jac=jac, bounds=(lb, ub), max_nfev=max_nfev, xtol=1e-10, ftol=1e-10)
    return out.x


def plan_from_stored(old, r, t_start):
    """Walk the stored plan over to these arms in small steps (continuation). One jump fails when the
    new values are far from the old ones (e.g. measured joint friction 19x the calculated value);
    small steps keep each solve close to a known answer. Returns (problem, result) or None."""
    r0 = old.get("model_ratios")
    p = np.array(old["p"])
    T = old["T"]
    if not r0 or len(p) != len(plan_problem(r, T)["p0"]):
        return None
    lam, step, last = 0.0, 1.0, None
    while lam < 1 - 1e-9:
        nxt = min(1.0, lam + step)
        rr = {k: r0.get(k, 0.0) + nxt * (r[k] - r0.get(k, 0.0)) for k in r}
        pr = plan_problem(rr, T)
        res, viol = plan_solve(pr, plan_refine(pr, p), maxiter=1000)
        ok = viol < 1e-6 and res.success
        log("PLAN", f"stored plan -> {nxt * 100:3.0f}% of the way to these arms (B1 {rr['B1']:.3f}, B2 {rr['B2']:.3f}): "
                    f"{'ok' if ok else 'no plan'} (effort {res.fun:.1f}, {time.time() - t_start:.0f} s so far)")
        if ok:
            lam, p, last = nxt, res.x, (pr, res)
            step = min(2 * step, 1.0)
        else:
            step /= 2
            if step < CONTINUATION_MIN_STEP:
                break
    if lam >= 1 - 1e-9:
        return last
    log("PLAN", f"the plan only reached {lam * 100:.0f}% of the way to these arms within the limits "
                f"(B2 up to {r0.get('B2', 0) + lam * (r['B2'] - r0.get('B2', 0)):.3f} works)")
    return None


def plan(args):
    r = arm_ratios()
    log("PLAN", f"arm model P1={r['P1']:.4f} P2={r['P2']:.4f} J2/J1={r['J2r']:.4f} H={r['H']:.4f} "
                f"B1={r['B1']:.4f} B2={r['B2']:.4f}; limits +/-{PLAN_TRACK_M} m, {PLAN_SPEED_M_S} m/s, "
                f"{PLAN_ACCEL_M_S2} m/s^2")
    t_start = time.time()
    best = None
    if TRAJECTORY_FILE.exists():
        best = plan_from_stored(json.loads(TRAJECTORY_FILE.read_text()), r, t_start)
    if best is None:
        # from scratch: a few guesses only (each can take minutes on a Pi)
        for seed in (2, None, 1):
            pr = plan_problem(r, PLAN_DURATION_S)
            p0 = pr["p0"].copy()
            if seed is not None:
                p0[:pr["nu"]] += np.random.default_rng(seed).normal(0, 3, pr["nu"])
            res, viol = plan_solve(pr, p0)
            ok = viol < 1e-6 and res.success
            log("PLAN", f"from guess {seed}: {'found' if ok else 'no plan'} (effort {res.fun:.1f}, mismatch {viol:.1e}, "
                        f"{time.time() - t_start:.0f} s so far)")
            if ok:
                best = (pr, res)
                break
    if best is None:
        sys.exit("No swing-up plan found within the limits. Either raise PLAN_ACCEL_M_S2 (8-10) and "
                 "PLAN_SPEED_M_S (0.75-0.8), or lower B2 in SYSID_RATIOS to the value printed above.")
    pr, res = best
    tr, a = pr["sim"](res.x[None])
    z = np.concatenate([tr[0, s, :-1] for s in range(pr["S"])] + [tr[0, -1, -1:]])
    a = a[0]
    out = {"version": 1, "T": pr["T"], "dt": SAMPLE_PERIOD_S, "M": pr["M"], "S": pr["S"], "p": res.x.tolist(),
           "z": z.tolist(), "a_knots": a.tolist(), "model_ratios": r, "effort": float(res.fun),
           "limits": {"track_m": pr["X"], "speed_m_s": pr["V"], "accel_m_s2": pr["A"], "motor_top_speed_m_s": pr["Vtop"]},
           "peak": {"track_m": float(np.abs(z[:, 4]).max()), "speed_m_s": float(np.abs(z[:, 5]).max()),
                    "accel_m_s2": float(np.abs(a).max())},
           "made": datetime.now().astimezone().isoformat(timespec="seconds")}
    TRAJECTORY_FILE.write_text(json.dumps(out))
    log("PLAN", f"saved {TRAJECTORY_FILE.name}: {pr['T']:.2f} s, cart within +/-{out['peak']['track_m']:.3f} m, "
                f"up to {out['peak']['speed_m_s']:.2f} m/s and {out['peak']['accel_m_s2']:.1f} m/s^2")


def load_trajectory(r):
    if not TRAJECTORY_FILE.exists():
        sys.exit(f"{TRAJECTORY_FILE.name} not found: run  python3 {Path(__file__).name} --plan  first")
    d = json.loads(TRAJECTORY_FILE.read_text())
    z = np.array(d["z"])
    knots = np.array(d["a_knots"])                   # cart acceleration every dt, as planned
    N = len(z) - 1
    tau = CONTROL_LATENCY_MS / 1000.0
    T = d["dt"]
    tg = np.arange(len(knots)) * T
    # speed change decided at sample k lands at k*T + tau: step from the planned speed half a period
    # before that to half a period after
    vz = z[:, 5]
    a = np.array([(np.interp(k * T + tau + T / 2, tg, vz) - np.interp(k * T + tau - T / 2, tg, vz)) / T
                  for k in range(N)])
    off = max(abs(d["model_ratios"][k] / r[k] - 1) for k in ("P1", "P2", "J2r", "H"))
    if off > 0.01:
        log("WARN", f"the stored plan was made for arm values up to {off * 100:.0f}% different from the ones in "
                    f"this file: run with --plan")
    return {"z": z, "a": a, "N": N, "meta": d}


# ------------------------------------------------------------------ hardware helpers
def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f"{port.port}: received {len(data)}/{count} bytes (limit switch or stop button? "
                           "reset the Arduino)")
    return bytes(data)


def signed_count_delta(raw, reference):
    return (raw - reference + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def circular_mean_counts(readings):
    phase = np.asarray(readings, dtype=float) * RAD_PER_COUNT
    return (np.angle(np.mean(np.exp(1j * phase))) % (2 * np.pi)) / RAD_PER_COUNT


def timer_for_speed(v, hold_error_m, deadband):
    if abs(v) < deadband or abs(v) < 2.0 * M_PER_STEP:
        return (65535 if hold_error_m > 0 else -65535), 0.0
    step_hz = -v / M_PER_STEP
    icr = max(1, min(65535, int(round(TIMER_STEP_HZ / abs(step_hz))) - 1))
    return (icr if step_hz > 0 else -icr), step_hz


# ------------------------------------------------------------------ main
def run():
    import serial
    from run_csv_logger import DOUBLE_COLUMNS, RunCsvLogger
    from angle_protocol import AnglePacketReader, AngleProtocolError, packet_csv_values
    from angle_packet_fallback import MissingAnglePacketError, collect_valid_samples

    r = arm_ratios()
    traj = load_trajectory(r)
    bal = balance_design(r)
    Ks = tvlqr(traj, r, bal["P"])
    L_swing = swing_observer_gain()
    pk = traj["meta"]["peak"]
    log("START", f"plan {traj['N'] * SAMPLE_PERIOD_S:.2f} s: cart within +/-{pk['track_m'] * 1000:.0f} mm, up to "
                 f"{pk['speed_m_s']:.2f} m/s and {pk['accel_m_s2']:.1f} m/s^2; gains ready")
    reader = AnglePacketReader()
    state = {"msg": ""}

    def read_angles(esp32):
        try:
            packet = reader.read_latest(esp32)
        except AngleProtocolError as exc:
            state["packet"] = None
            state["msg"] = str(exc)
            raise MissingAnglePacketError(str(exc)) from exc
        state["packet"] = packet
        state["msg"] = packet.missing_message((0, 1))
        return packet.angles

    def read_required(esp32):
        raw = read_angles(esp32)
        if raw[0] == 0xFFFF or raw[1] == 0xFFFF:
            raise MissingAnglePacketError(state["msg"])
        return raw

    esp32 = arduino = None
    run_logger = None
    run_stop_reason = "program exit"
    motor_command_sent = False
    cycle = 0
    off1, offj = math.radians(ARM1_OFFSET_DEG), math.radians(JOINT2_OFFSET_DEG)
    brake = {"v": 0.0}             # last speed sent, for braking at the stop
    stage = "opening serial ports"
    try:
        esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
        log("START", "serial ports open; waiting 3 s for the Arduino")
        time.sleep(3)
        arduino.reset_input_buffer()
        esp32.reset_input_buffer()

        stage = "hanging calibration"
        log("READY", "Motor power OFF; center the cart and let BOTH arms hang still")
        input("Press Enter when both arms have stopped swinging: ")
        reader.discard_pending(esp32)
        readings, misses = collect_valid_samples(lambda: read_required(esp32)[:2], HANGING_SAMPLES,
                                                 interval_s=SAMPLE_SPACING_S, max_total_misses=MAX_HANGING_TOTAL_MISSES,
                                                 max_consecutive_misses=MAX_SETUP_CONSECUTIVE_MISSES)
        readings = np.asarray(readings, dtype=float)
        down1, aligned2 = circular_mean_counts(readings[:, 0]), circular_mean_counts(readings[:, 1])
        for ch, ref in ((0, down1), (1, aligned2)):
            dev = float(np.max(np.abs(signed_count_delta(readings[:, ch], ref))))
            if dev > MAX_HANGING_DEVIATION_COUNTS:
                raise RuntimeError(f"arm {ch + 1} moved during calibration ({dev:.0f} counts); let it settle and restart")
        upright1 = (down1 + HALF_TURN) % ENCODER_COUNTS
        log("CAL", f"arm 1 hanging {down1:.1f} counts (upright {upright1:.1f}); arm 2 aligned {aligned2:.1f} counts")

        if SAVE_RUN_CSV:
            filename = datetime.now().astimezone().strftime("swingup_double_%Y%m%d_%H%M%S_%f.csv")
            run_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=DOUBLE_COLUMNS)
            run_logger.start()
            settings = {k: (str(v) if isinstance(v, Path) else v) for k, v in globals().items()
                        if k.isupper() and isinstance(v, (int, float, str, bool, type(None), Path))}
            run_logger.event("config", json.dumps({
                "schema_version": 2, "angle_protocol_version": 1, "pendulum_count": 2,
                "script": "PythonMaster2_swingup.py", "force_columns": "cart acceleration, m/s^2",
                "settings": settings, "model_ratios": r, "trajectory": {k: traj["meta"][k] for k in ("T", "peak", "limits", "made")},
                "reference_upright_1_counts": float(upright1), "reference_aligned_2_counts": float(aligned2),
                "balance_gain": bal["K"].tolist()}, separators=(",", ":"), default=str), 0.0)
            log("LOG", f"Recording every cycle to {run_logger.path}")

        stage = "waiting for motor power"
        log("READY", "Turn motor power ON. Arms hanging still, hands and face clear of the arms' circle")
        input("Then press Enter to swing up: ")
        run_started_at = time.monotonic()
        reader.discard_pending(esp32)
        motor_command_sent = True
        arduino.write(struct.pack("<i", -65535))
        steps0 = struct.unpack(">h", read_exact(arduino, 2))[0]
        arduino.reset_input_buffer()
        steps, t_reply, cart_m = steps0, time.monotonic(), 0.0
        log("START", f"Arduino replied: step count {steps0} is the cart's zero")

        # ---- control state
        mode = "WAIT"                     # WAIT -> SWING -> BALANCE; RECOVER on failure
        attempt = 0
        k = 0
        mode_t0 = time.monotonic()
        hang_hist = deque(maxlen=max(2, int(STILL_TIME_S / SAMPLE_PERIOD_S)))
        x_hat = np.array([math.pi, 0.0, math.pi, 0.0, 0.0, 0.0])
        a_prev = 0.0
        v_state = v_sent = 0.0
        hold_x = 0.0
        misses_in_row = 0
        catch_t = None
        recent_x = deque(maxlen=200)
        next_status = time.monotonic()
        first = True
        log("RUN", "control loop started; checking the arms hang still")

        while True:
            stage = "control loop"
            cycle += 1
            cycle_started = time.monotonic()
            try:
                raw1, raw2, raw3 = read_angles(esp32)
            except MissingAnglePacketError:
                raw1 = raw2 = raw3 = None
            cart_now = cart_m + v_sent * max(0.0, time.monotonic() - t_reply)

            # ---- measurement: angles from upright, repo convention
            ok1 = raw1 is not None and raw1 != 0xFFFF
            ok2 = raw2 is not None and raw2 != 0xFFFF
            th1_m = -signed_count_delta(raw1, upright1) * RAD_PER_COUNT - off1 if ok1 else None
            jt_m = -signed_count_delta(raw2, aligned2) * RAD_PER_COUNT - offj if ok2 else None
            if not (ok1 and ok2):
                misses_in_row += 1
                if misses_in_row > MAX_RUNNING_CONSECUTIVE_MISSES:
                    raise RuntimeError(f"{misses_in_row} unusable angle frames in a row; stopping. {state['msg']}")
            else:
                misses_in_row = 0
            if first and ok1 and ok2:
                # start with the angles on the +180 deg branch, where the plan starts
                x_hat[0] = math.pi + wrap(th1_m - math.pi)
                x_hat[2] = x_hat[0] + wrap(th1_m + jt_m - th1_m)
                first = False
            if abs(cart_now) > TRACK_SOFT_LIMIT_M:
                raise RuntimeError(f"cart at {cart_now:+.3f} m passed TRACK_SOFT_LIMIT_M={TRACK_SOFT_LIMIT_M}")

            # ---- observer correction (innovation wrapped: angles stay continuous through full turns)
            L = bal["L"] if mode == "BALANCE" else L_swing
            innov = np.zeros(3)
            pkt = state.get("packet")
            ages = pkt.ages_ms if pkt is not None else (0, 0, 0)
            d1 = (min(ages[0], 50) + SENSOR_EXTRA_DELAY_MS) / 1000.0     # when each reading was taken
            d2 = (min(ages[1], 50) + SENSOR_EXTRA_DELAY_MS) / 1000.0
            if ok1:
                innov[0] = wrap(th1_m - (x_hat[0] - x_hat[1] * d1))
            if ok1 and ok2:
                # arm 2's absolute angle mixes both readings: compare at their own times
                th2_then = (x_hat[0] - x_hat[1] * d1) + ((x_hat[2] - x_hat[3] * d2) - (x_hat[0] - x_hat[1] * d2))
                innov[1] = wrap(th1_m + jt_m - th2_then)
            innov[2] = cart_now - x_hat[4]
            x_hat = x_hat + L @ innov
            x_hat[5] = v_state
            th1, th2 = x_hat[0], x_hat[2]
            t_mode = cycle_started - mode_t0

            # ---- mode logic and acceleration
            msg = ""
            if ok1 and ok2:
                hang_hist.append(max(abs(wrap(th1_m - math.pi)), abs(wrap(th1_m + jt_m - math.pi))))
            hang_dev = max(hang_hist) if hang_hist else math.pi
            if mode == "WAIT":
                hang_err = hang_dev
                a = 30.0 * (0.0 - cart_now) - 12.0 * v_state
                calm = len(hang_hist) == hang_hist.maxlen and hang_dev < math.radians(STILL_ANGLE_DEG) and abs(cart_now) < 0.01
                if not calm and t_mode > 3.0 + STILL_TIME_S:
                    mode, mode_t0 = "RECOVER", cycle_started
                    log("RUN", "arms not hanging still: damping the swing first")
                elif calm:
                    attempt += 1
                    if attempt > MAX_ATTEMPTS:
                        raise RuntimeError(f"no catch after {MAX_ATTEMPTS} attempts")
                    x_hat[0] = math.pi + wrap(th1 - math.pi)        # back onto the plan's branch
                    x_hat[2] = math.pi + wrap(th2 - math.pi)
                    mode, mode_t0, k = "SWING", cycle_started, 0
                    log("RUN", f"*** SWING-UP attempt {attempt} ***")
                    if run_logger is not None:
                        run_logger.event("swing", f"attempt={attempt}", cycle_started - run_started_at)
                msg = f"mode=WAIT; hang_err_deg={math.degrees(hang_err):.2f}"
            if mode == "SWING":
                zr = traj["z"][k]
                e = np.concatenate([x_hat - zr, [a_prev - (traj["a"][k - 1] if k > 0 else 0.0)]])
                e[0], e[2] = wrap(e[0]), wrap(e[2])
                a = traj["a"][k] - float((Ks[k] @ e)[0])
                err = max(abs(e[0]), abs(e[2]))
                msg = f"mode=SWING; k={k}; err_deg={math.degrees(err):.1f}"
                if err > math.radians(SWING_ABORT_DEG) or abs(cart_now) > SWING_CART_ABORT_M:
                    why = (f"{math.degrees(err):.0f} deg off the plan" if err > math.radians(SWING_ABORT_DEG)
                           else f"cart at {cart_now * 1000:+.0f} mm")
                    log("RUN", f"attempt {attempt}: {why} at {k * SAMPLE_PERIOD_S:.2f} s; recovering")
                    mode, mode_t0 = "RECOVER", cycle_started
                    if run_logger is not None:
                        run_logger.event("abort", msg, cycle_started - run_started_at)
                else:
                    k += 1
                    if k >= traj["N"]:
                        x_hat[0], x_hat[2] = wrap(x_hat[0]), wrap(x_hat[2])
                        mode, mode_t0, catch_t = "BALANCE", cycle_started, cycle_started
                        log("RUN", f"*** CATCH (attempt {attempt}) *** arm1 {math.degrees(x_hat[0]):+.1f} deg, "
                                   f"arm2 {math.degrees(x_hat[2]):+.1f} deg, cart {cart_now * 1000:+.0f} mm; balancing")
                        if run_logger is not None:
                            run_logger.event("catch", f"attempt={attempt}", cycle_started - run_started_at)
            if mode == "BALANCE":
                th1, th2 = wrap(x_hat[0]), wrap(x_hat[2])
                x_hat[0], x_hat[2] = th1, th2
                if max(abs(th1), abs(th2)) > math.radians(BALANCE_LOST_DEG):
                    log("RUN", f"balance lost (arm1 {math.degrees(th1):+.0f}, arm2 {math.degrees(th2):+.0f} deg): recovering")
                    mode, mode_t0 = "RECOVER", cycle_started
                    if run_logger is not None:
                        run_logger.event("lost", "", cycle_started - run_started_at)
                else:
                    a = -float((bal["K"] @ np.concatenate([x_hat, [a_prev]]))[0])
                    msg = "mode=BALANCE"
            if mode == "RECOVER":
                if t_mode < 0.5 and abs(v_state) > 0.05:
                    # stop the cart first (full braking, which the motor always has torque for)
                    a = -math.copysign(CART_ACCEL_MAX_M_S2, v_state)
                    if abs(a * SAMPLE_PERIOD_S) > abs(v_state):
                        a = -v_state / SAMPLE_PERIOD_S
                else:
                    p1, p2 = r["P1"], r["P2"]
                    power = p1 * math.cos(x_hat[0]) * x_hat[1] + p2 * math.cos(x_hat[2]) * x_hat[3]
                    a = max(-RECOVER_ACCEL_M_S2, min(RECOVER_ACCEL_M_S2, RECOVER_GAIN * power))
                    if abs(cart_now) > 0.12 and a * cart_now > 0:
                        a = 0.0                      # near the track ends, only damp toward the centre
                    a += -2.25 * cart_now - 3.0 * v_state
                msg = f"mode=RECOVER; hang_err_deg={math.degrees(hang_dev):.1f}"
                if hang_dev < math.radians(STILL_ANGLE_DEG * 2) and abs(cart_now) < 0.01:
                    mode, mode_t0 = "WAIT", cycle_started
                elif t_mode > RECOVER_TIMEOUT_S:
                    raise RuntimeError("the arms did not settle; stopping")

            # ---- limits, speed command (the new speed is sent now; "a" is the change divided by T)
            a = max(-CART_ACCEL_MAX_M_S2, min(CART_ACCEL_MAX_M_S2, a))
            if a * v_state > 0:      # speeding up further: only the torque the motor has left at this speed
                a_lim = CART_ACCEL_MAX_M_S2 * max(0.05, 1.0 - abs(v_state) / MOTOR_TOP_SPEED_M_S)
                a = max(-a_lim, min(a_lim, a))
            v_cmd = v_state + a * SAMPLE_PERIOD_S
            if abs(v_cmd) > CART_SPEED_MAX_M_S:
                v_cmd = math.copysign(CART_SPEED_MAX_M_S, v_cmd)
            deadband = SPEED_DEADBAND_M_S if mode in ("BALANCE", "WAIT") else 0.0
            if abs(v_cmd) >= max(deadband, 2.0 * M_PER_STEP):
                hold_x = cart_now + v_cmd * SAMPLE_PERIOD_S
            timer, step_hz = timer_for_speed(v_cmd, cart_now - hold_x, deadband)
            v_sent = 0.0 if timer in (65535, -65535) else -math.copysign(TIMER_STEP_HZ / (abs(timer) + 1), timer) * M_PER_STEP
            arduino.write(struct.pack("<i", int(timer)))
            brake["v"] = v_sent
            steps_before = steps if steps is not None else steps0
            steps = struct.unpack(">h", read_exact(arduino, 2))[0]
            arduino.reset_input_buffer()
            t_reply = time.monotonic()
            jump = ((steps - steps_before + 32768) & 0xFFFF) - 32768
            if abs(jump) > MAX_STEPS_PER_REPLY:
                raise RuntimeError(f"step count jumped {jump:+d} steps in one cycle ({steps_before} -> {steps}); the cart "
                                   "cannot move that fast, so the count is wrong. Check the D5 -> D18 wire, make sure "
                                   "ModemManager is off, press the Mega's RESET button and run again.")
            cart_m = (steps - steps0) * M_PER_STEP

            # ---- predict the next cycle with the nonlinear model, using the speed really sent
            a = (v_sent - v_state) / SAMPLE_PERIOD_S
            x_pred = step(x_hat, a_prev, a, r)
            a_before, a_prev = a_prev, a
            v_state = v_sent
            x_pred[5] = v_state

            # ---- auto-trim while balancing
            if mode == "BALANCE" and AUTO_TRIM and cycle_started - catch_t > TRIM_START_DELAY_S and ok1 and ok2:
                kt = SAMPLE_PERIOD_S / TRIM_TIME_CONSTANT_S
                lim = math.radians(TRIM_MAX_DEG)
                d1 = max(-lim - off1, min(lim - off1, kt * wrap(th1_m)))
                dj = max(-lim - offj, min(lim - offj, kt * wrap(jt_m)))
                off1 += d1
                offj += dj
                x_pred[0] -= d1
                x_pred[2] -= d1 + dj

            if run_logger is not None:
                deg = math.degrees
                run_logger.sample((
                    "sample", cycle_started - run_started_at, cycle, reader.last_packet.sequence if reader.last_packet else "",
                    raw1, raw2, raw3, upright1 - deg(off1) * ENCODER_COUNTS / 360.0, aligned2 - deg(offj) * ENCODER_COUNTS / 360.0,
                    th1_m if ok1 else "", deg(th1_m) if ok1 else "", (th1_m + jt_m) if ok1 and ok2 else "",
                    deg(th1_m + jt_m) if ok1 and ok2 else "", jt_m if ok2 else "", deg(jt_m) if ok2 else "",
                    cart_now, steps - steps0, cart_m, *x_hat, *x_hat, a_before, a, v_cmd, 1.0, step_hz, timer,
                    upright1 - deg(off1) * ENCODER_COUNTS / 360.0, "", "", (time.monotonic() - cycle_started) * 1000, "",
                    *packet_csv_values(state.get("packet"), reader), msg))
            x_hat = x_pred
            recent_x.append(cart_m)
            now = time.monotonic()
            if now >= next_status:
                extra = ""
                if mode == "BALANCE":
                    xs = np.asarray(recent_x)
                    extra = (f" spread(2s)={np.ptp(xs) * 1000:.1f}mm trim arm1={math.degrees(off1):+.3f} "
                             f"joint={math.degrees(offj):+.3f}deg")
                log("RUN", f"{mode:<7} arm1={math.degrees(wrap(th1)):+7.1f}deg arm2={math.degrees(wrap(th2)):+7.1f}deg "
                           f"cart={cart_m * 1000:+6.0f}mm v={v_cmd:+.2f}m/s a={a:+6.1f}m/s^2{extra}")
                next_status = now + STATUS_INTERVAL_S

    except KeyboardInterrupt:
        run_stop_reason = "Ctrl+C"
        log("STOP", f"Ctrl+C after {cycle} cycles")
    except Exception as exc:
        run_stop_reason = f"{type(exc).__name__}: {exc}"
        if run_logger is not None:
            run_logger.event("fault", f"stage={stage}; cycle={cycle}; {run_stop_reason}", "")
        log("FAULT", f"{stage}, cycle {cycle}: {type(exc).__name__}: {exc}")
        traceback.print_exc()
    finally:
        try:
            if arduino is not None and arduino.is_open:
                if motor_command_sent and abs(brake["v"]) > 0.005 and "Arduino" not in run_stop_reason \
                        and "received" not in run_stop_reason:
                    # bring the cart to rest first: a switched-off stepper would let it coast
                    try:
                        v = brake["v"]
                        while abs(v) > 0.005:
                            v -= math.copysign(min(abs(v), STOP_BRAKE_M_S2 * SAMPLE_PERIOD_S), v)
                            timer, _ = timer_for_speed(v, 0.0, 0.0)
                            arduino.write(struct.pack("<i", int(timer)))
                            read_exact(arduino, 2)
                            arduino.reset_input_buffer()
                            time.sleep(SAMPLE_PERIOD_S * 0.8)
                        log("STOP", f"cart braked to a stop from {brake['v']:+.2f} m/s")
                    except Exception as exc:
                        log("FAULT", f"could not brake the cart: {exc}")
                if motor_command_sent and STOP_HOLD_S > 0 and "received" not in run_stop_reason:
                    log("STOP", f"holding the cart for {STOP_HOLD_S:.0f} s while the arms swing down "
                                "(Ctrl+C again to switch the motor off now)")
                    try:
                        t_end = time.monotonic() + STOP_HOLD_S
                        hold = None
                        steps_now = 0
                        while time.monotonic() < t_end:
                            arduino.write(struct.pack("<i", 65535 if hold is not None and steps_now > hold else -65535))
                            steps_now = struct.unpack(">h", read_exact(arduino, 2))[0]
                            arduino.reset_input_buffer()
                            hold = steps_now if hold is None else hold
                            time.sleep(SAMPLE_PERIOD_S * 0.8)
                    except KeyboardInterrupt:
                        pass
                    except Exception as exc:
                        log("FAULT", f"could not hold the cart: {exc}")
                if motor_command_sent:
                    try:
                        arduino.write(struct.pack("<i", -1))       # timer off, motor disabled
                        arduino.flush()
                        log("STOP", "Sent Arduino stop command")
                    except Exception as exc:
                        log("FAULT", f"Could not send stop command: {exc}")
                arduino.close()
            if esp32 is not None and esp32.is_open:
                esp32.close()
            if cycle:
                log("STOP", f"Trims found: ARM1_OFFSET_DEG = {math.degrees(off1):.3f}, JOINT2_OFFSET_DEG = "
                            f"{math.degrees(offj):.3f}")
        finally:
            if run_logger is not None:
                warning = run_logger.stop(run_stop_reason)
                log("STOP", warning or f"CSV saved: {run_logger.path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Double pendulum swing-up and balance.")
    ap.add_argument("--plan", action="store_true", help="make a new swing-up plan for the arm values in this file")
    if ap.parse_args().plan:
        plan(None)
    else:
        run()
