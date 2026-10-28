"""PythonMaster2_auto_improved.py - balance the double pendulum with the cart standing (almost) still.

Drop-in alternative to PythonMaster2_auto.py. Same folder, same helpers (angle_protocol.py,
angle_packet_fallback.py, run_csv_logger.py), same ports, same two startup prompts:
    1. motor power OFF, cart centred, both arms hanging still  -> hanging calibration
    2. motor power ON, hold both arms upright, press Enter      -> balancing starts
Ctrl+C stops the motor.

What is different from PythonMaster2_auto.py
  * The model treats the cart the way the stepper actually moves it: the controller decides a
    cart ACCELERATION, the script integrates it into the speed it sends. The old script computed
    a force for a cart model and then sent the Kalman filter's velocity, which does not match a
    stepper (it moves exactly as told, whatever the force).
  * The LQR and Kalman gains are worked out at startup from the arm values below (numpy and
    scipy only), so changing a weight here is all that is needed. control_matrices2.txt and
    PythonMasterMath2.py are not used.
  * The 5-7 ms between reading the angles and the motor reacting is part of the design.
  * Auto-trim: while balancing, the upright references drift slowly to the arms' true balance
    point. With the references right, holding the arms up needs no cart motion, so the cart
    settles at the centre and stays within a few millimetres. Encoder offsets otherwise make
    the cart park off centre or run into a limit switch.
  * Tiny speed commands become a one-step dither around a held position instead of a slow
    1.9 step/s creep (the Arduino cannot stop the step timer without disabling the motor).
  * Speed, acceleration, track and angle limits, a short start ramp, the cart position zeroed
    at the first reply (the step counter drifts while the motor is off), and a status line
    that shows how still the cart is.

A perfect standstill is not possible: an inverted pendulum always needs some correction, and the
encoders only resolve 0.022 deg, so the controller keeps nudging. On the digital twin, once
balanced, the cart stays within about 4 mm (0.7 mm typical deviation), moving slowly. The
current script runs into a limit switch there within 7 s.

Quick tuning guide (details above each setting):
  cart jitters or buzzes while balanced  -> ENCODER_NOISE_COUNTS up, or LQR_CART_ACCEL_M_S2 down
  cart slowly wanders a centimetre or more -> check the printed trims; LQR_CART_M down a little
  arms wobble or a light tap knocks them over -> LQR_CART_ACCEL_M_S2 up, LQR_ARM*_ANGLE_DEG down
  oscillation at a few Hz that grows      -> CONTROL_LATENCY_MS up 2-4 ms
  stepper skips (cart ends off its mark)  -> CART_ACCEL_MAX_M_S2 and LQR_CART_ACCEL_M_S2 down
  cart parks off centre at the start      -> copy the printed trims into ARM1/JOINT2_OFFSET_DEG

CSV: same columns as PythonMaster2_auto.py (run_logs/balance_double_*.csv, RunReview works).
The "force" columns hold the cart acceleration command in m/s^2 here, not newtons.
"""

import json
import math
import struct
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from scipy.linalg import expm, solve_discrete_are

from run_csv_logger import DOUBLE_COLUMNS, RunCsvLogger
from angle_protocol import AnglePacketReader, AngleProtocolError, packet_csv_values
from angle_packet_fallback import MissingAnglePacketError, collect_valid_samples


# =====================================================================================
# 1. ARM MODEL
# The controller is designed from these numbers at startup. They only need to be roughly
# right (within 10-20%); the LQR copes with the rest. Better numbers give a calmer, stiller
# balance. Defaults are the measured values in PythonMasterMath2.py.
# =====================================================================================

# Mass of arm 1 alone (everything that swings with it except arm 2: rod, weights, TX1 board,
# battery, screws), kg. Heavier arm 1 -> the model expects arm 1 to resist the cart more.
# Re-weigh it if you add or remove anything on the arm.
ARM1_MASS_KG = 0.207

# Distance from the main pivot axis to arm 1's centre of mass, m (balance the removed arm on
# a knife edge). This sets how fast arm 1 falls: further out = slower fall.
ARM1_COM_M = 0.087

# Arm 1's moment of inertia about its OWN centre of mass, kg m^2 (not about the pivot).
# Larger -> arm 1 reacts more slowly in the model. If the measured swing period is longer than
# the model's, this is usually too small.
ARM1_INERTIA_KG_M2 = 0.00123324992

# Main pivot axis to arm 2's joint axis, m. Measure it on the rig; it couples the two arms.
ARM1_LENGTH_M = 0.2

# Mass of arm 2 alone (rod, weights, TX2 board, battery), kg.
ARM2_MASS_KG = 0.210

# Distance from arm 2's joint axis to arm 2's centre of mass, m.
ARM2_COM_M = 0.106

# Arm 2's moment of inertia about its own centre of mass, kg m^2.
ARM2_INERTIA_KG_M2 = 0.00197563667

# Viscous friction at the main pivot and at the joint, N m s/rad. They barely affect the
# design; leave them small. Raise them if the free swings die out much faster than the model says.
ARM1_PIVOT_FRICTION = 0.0002
ARM2_JOINT_FRICTION = 0.0001

# Optional: the ratios analyze_sysid.py fits from a PendulumSysID.py run. They describe the arms
# better than any measurement with a ruler and a scale. If set, they replace every value above.
# Example: SYSID_RATIOS = {"P1": 5.357, "P2": 1.985, "J2r": 0.3866, "H": 0.3972, "B1": 0.012, "B2": 0.0098}
SYSID_RATIOS = None

G = 9.81   # m/s^2

# =====================================================================================
# 2. LQR WEIGHTS (Bryson's rule: "how big an error of each kind is acceptable")
# Each value is the size of error the controller treats as "a lot". A SMALLER value makes the
# controller fight that error HARDER (more cart motion spent on it); a LARGER value makes it
# care less. Only the ratios between them matter.
# =====================================================================================

# Arm 1 angle from upright that counts as a large error, degrees.
# Decrease (e.g. 1.0) for a stiffer arm 1: it holds closer to vertical, but the cart moves
# more often and harder, and sensor noise shows up as jitter. Increase (3-4) for a softer
# balance. 2-4 both held fine on the twin.
LQR_ARM1_ANGLE_DEG = 2.0

# The same for arm 2's angle from vertical (absolute, not the joint angle). Arm 2 is the one
# that falls fastest, so keep this at or below LQR_ARM1_ANGLE_DEG.
LQR_ARM2_ANGLE_DEG = 2.0

# Angular speed of either arm that counts as large, deg/s. Smaller damps arm wobble harder but
# amplifies encoder noise (speeds are estimated from changing angles). 30-120 is reasonable.
LQR_ARM_RATE_DEG_S = 30.0
# Cart distance from the centre that counts as large, m. Smaller pulls the cart back to the
# centre faster (more motion while it does); larger lets it wander further but keeps it calmer.
# If the cart reaches a limit switch, decrease this (and check the auto-trim below is on).
LQR_CART_M = 0.05

# Cart speed that counts as large, m/s. Smaller makes the cart move slower and more smoothly
# (good for a still cart); too small and it cannot move fast enough to save a big wobble.
LQR_CART_SPEED_M_S = 0.2

# Cart acceleration that counts as large, m/s^2 (the cost on the control effort itself).
# This is the main "calm vs. stiff" knob. Smaller = gentler, smaller corrections, stiller cart,
# but slower recovery from a push. Larger = snappier, more aggressive corrections. If the stepper
# misses steps or buzzes, lower it. If a light tap knocks the arms over, raise it.
LQR_CART_ACCEL_M_S2 = 1.5

# Time from the angle being measured to the motor reacting, ms (receiver age about 3 ms, USB
# about 1 ms, the Arduino about 1-2 ms). The design compensates for it. If the balance
# oscillates at a few Hz, increase it by 2-4 ms; if it is sluggish, decrease it.
CONTROL_LATENCY_MS = 6.0

# Control period, s. The receiver sends a frame every 10 ms and the loop runs once per frame.
# Only change it if the receiver firmware's frame rate changes.
SAMPLE_PERIOD_S = 0.01

# =====================================================================================
# 3. ESTIMATOR (Kalman filter: turns noisy angles into angles + angular speeds)
# =====================================================================================

# Encoder noise, counts (1 count = 0.022 deg). Larger trusts the encoders less: smoother
# estimates but a little more lag. Raise it (5-10) if the cart buzzes or jitters while balanced;
# lower it (1) if the balance feels late or sloppy. 3 cut the cart's jitter by more than half
# compared with 1 on the twin.
ENCODER_NOISE_COUNTS = 3.0

# How hard unmodelled things (air, cable tug, friction, model errors) can push the arms, in
# rad/s^2. Larger makes the filter follow the measurements more closely: quicker, noisier.
# Smaller trusts the model more: smoother, but slow to notice a real push. 0.3-5.
ARM_DISTURBANCE_RAD_S2 = 1.0

# =====================================================================================
# 4. STANDSTILL: AUTO-TRIM AND STEP DITHER
# =====================================================================================

# Auto-trim the upright references while balancing. When the arms are balanced and the cart is
# still, both arms are exactly vertical, so the average measured angle IS the encoder's offset
# from true vertical. The trim follows that average, so the controller's "upright" ends up at the
# real balance point and holding it needs no cart motion. Turn off only to test a fixed offset.
AUTO_TRIM = True

# How fast the trim follows, s (time constant). It is mostly there after about 2x this.
# Smaller finds the balance point sooner but follows the arms' own wobble more; much larger
# (15 s+) lets the cart wander by several millimetres while it catches up. 3-10 s all worked
# the same on the twin.
TRIM_TIME_CONSTANT_S = 6.0

# Wait this long after the start before trimming, s, so the catch from your hands isn't averaged in.
TRIM_START_DELAY_S = 1.0

# Largest trim allowed on each reference, degrees. If the end-of-run printout shows a trim
# at this limit, the hanging calibration is off by more than that: fix the arm or raise this.
TRIM_MAX_DEG = 3.0

# Starting offsets, degrees, added before balancing (positive subtracts from the reported angle).
# At Ctrl+C the script prints the trims it found; put them here so the next run starts at the
# balance point. JOINT2 is arm 2 relative to arm 1.
ARM1_OFFSET_DEG = 0.0
JOINT2_OFFSET_DEG = 0.0

# Speed commands smaller than this hold the cart at a fixed position with a +/-1 step dither,
# m/s. The Arduino's slowest real speed is 1.9 steps/s (0.19 mm/s), so smaller requests would
# otherwise creep away. 0.0003-0.001 work; above about 0.002 the cart sits still for a moment,
# then has to catch up, and the balance turns into a slow back-and-forth.
SPEED_DEADBAND_M_S = 0.0003

# =====================================================================================
# 5. LIMITS AND SAFETY
# =====================================================================================

# Fastest cart speed ever sent, m/s. Lower it if the stepper stalls at speed; the balance
# itself rarely needs more than 0.3 m/s.
CART_SPEED_MAX_M_S = 0.6

# Hardest acceleration ever sent, m/s^2. Steppers lose steps if asked for too much. Lower it
# (5-8) if you hear the motor skip; raise it (up to 20) if pushes are not recovered.
CART_ACCEL_MAX_M_S2 = 12.0

# The motor stops if the cart gets this far from where it started, m. Set it inside the
# limit switches (they are about 0.4 m out on the twin; measure yours).
TRACK_SOFT_LIMIT_M = 0.30

# The motor stops if either arm leans further than this from upright, degrees. Leans this large
# cannot be recovered with this cart; the stop just ends the run cleanly instead of the cart
# chasing the arms into a limit switch. 20-35.
MAX_RUNNING_ANGLE_DEG = 30.0

# The correction ramps from 0 to full over this time after Enter, s, so the cart does not jump
# when your hands are still letting go. 0 disables it.
START_RAMP_S = 0.15

# Startup checks, as in PythonMaster2_auto.py.
MAX_START_ANGLE_DEG = 12          # both arms must be within this of upright when you press Enter
HANGING_SAMPLES = 50              # readings averaged for the hanging calibration
UPRIGHT_SAMPLES = 10              # readings for the upright check
SAMPLE_SPACING_S = 0.02           # pause between setup readings
MAX_HANGING_DEVIATION_COUNTS = 80  # arms must hang this still (counts) while calibrating
MAX_RUNNING_CONSECUTIVE_MISSES = 1  # unusable angle frames in a row allowed while balancing (prediction fills in)
MAX_SETUP_CONSECUTIVE_MISSES = 1
MAX_HANGING_TOTAL_MISSES = 5
MAX_UPRIGHT_TOTAL_MISSES = 3

# =====================================================================================
# 6. HARDWARE AND OUTPUT
# =====================================================================================
ESP32_PORT, ESP32_BAUD = "/dev/ttyUSB0", 921600        # receiver (protocol v1); change if the Pi names it differently
ARDUINO_PORT, ARDUINO_BAUD = "/dev/ttyACM0", 115200    # Arduino Mega running ArduinoSlave.ino
DISTANCE_PER_6400_STEPS_M = 0.638175   # cart travel for 6400 steps (measured; as in the repo)
STATUS_INTERVAL_S = 0.5                # how often the status line prints
SHOW_BALANCING_LOGS = True             # False hides the status lines (faults still print)
SAVE_RUN_CSV = True                    # one CSV per run in run_logs/, written off the control thread
RUN_LOG_DIRECTORY = Path(__file__).resolve().parent / "run_logs"

# -------------------------------------------------------------------------------------
ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
RAD_PER_COUNT = 2 * math.pi / ENCODER_COUNTS
M_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_STEP_HZ = 125000.0             # ArduinoSlave.ino: step rate = 125000 / (|timer| + 1)

started_at = time.monotonic()
balancing_output_active = False
esp32_packet_count = 0
latest_raw_angles = None
angle_reader = AnglePacketReader()
latest_angle_packet = None
latest_angle_message = ""


def log(tag, message):
    if balancing_output_active and not SHOW_BALANCING_LOGS and tag not in ("FAULT", "SENSOR", "STOP"):
        return
    print(f"[{time.monotonic() - started_at:7.2f}s] [{tag}] {message}", flush=True)


# ------------------------------------------------------------------ model and gains
def arm_ratios():
    """Arm dynamics divided by arm 1's inertia about the pivot (J1)."""
    if SYSID_RATIOS:
        r = dict(SYSID_RATIOS)
        r.setdefault("B1", 0.0)
        r.setdefault("B2", 0.0)
        r["source"] = "SYSID_RATIOS"
        return r
    J1 = ARM1_INERTIA_KG_M2 + ARM1_MASS_KG * ARM1_COM_M ** 2 + ARM2_MASS_KG * ARM1_LENGTH_M ** 2
    p1 = ARM1_MASS_KG * ARM1_COM_M + ARM2_MASS_KG * ARM1_LENGTH_M
    p2 = ARM2_MASS_KG * ARM2_COM_M
    J2 = ARM2_INERTIA_KG_M2 + ARM2_MASS_KG * ARM2_COM_M ** 2
    h = ARM2_MASS_KG * ARM1_LENGTH_M * ARM2_COM_M
    return {"P1": p1 / J1, "P2": p2 / J1, "J2r": J2 / J1, "H": h / J1,
            "B1": ARM1_PIVOT_FRICTION / J1, "B2": ARM2_JOINT_FRICTION / J1, "source": "arm values"}


def continuous_model(r):
    """x = [theta1, theta1_rate, theta2, theta2_rate, cart, cart_speed], input = cart acceleration.
    Angles from upright, absolute, in this script's (and the repo's) sign convention."""
    M = np.array([[1.0, r["H"]], [r["H"], r["J2r"]]])
    Mi = np.linalg.inv(M)
    Gm = Mi @ np.diag([G * r["P1"], G * r["P2"]])
    Dm = Mi @ np.array([[r["B1"] + r["B2"], -r["B2"]], [-r["B2"], r["B2"]]])
    Pa = Mi @ np.array([r["P1"], r["P2"]])
    A = np.zeros((6, 6))
    A[0, 1] = A[2, 3] = A[4, 5] = 1.0
    A[1, 0], A[1, 2], A[3, 0], A[3, 2] = Gm[0, 0], Gm[0, 1], Gm[1, 0], Gm[1, 1]
    A[1, 1], A[1, 3], A[3, 1], A[3, 3] = -Dm[0, 0], -Dm[0, 1], -Dm[1, 0], -Dm[1, 1]
    B = np.zeros((6, 1))
    B[1, 0], B[3, 0], B[5, 0] = -Pa[0], -Pa[1], 1.0
    return A, B


def discretize_with_delay(A, B, T, tau):
    """x[k+1] = Phi x[k] + G0 u[k] + G1 u[k-1] for an input that takes effect tau after the sample."""
    n = A.shape[0]

    def zoh(t):
        Mx = np.zeros((n + 1, n + 1))
        Mx[:n, :n], Mx[:n, n:] = A, B
        E = expm(Mx * t)
        return E[:n, :n], E[:n, n:]
    tau = min(max(tau, 0.0), T)
    Phi, _ = zoh(T)
    E1, G0 = zoh(T - tau)
    _, Gt = zoh(tau)
    return Phi, G0, E1 @ Gt


def design():
    r = arm_ratios()
    A, B = continuous_model(r)
    T, tau = SAMPLE_PERIOD_S, CONTROL_LATENCY_MS / 1000.0
    Phi, G0, G1 = discretize_with_delay(A, B, T, tau)
    # LQR on [state; previous acceleration] so the delay is part of the design
    Az = np.block([[Phi, G1], [np.zeros((1, 6)), np.zeros((1, 1))]])
    Bz = np.vstack([G0, [[1.0]]])
    q = [1 / math.radians(LQR_ARM1_ANGLE_DEG) ** 2, 1 / math.radians(LQR_ARM_RATE_DEG_S) ** 2,
         1 / math.radians(LQR_ARM2_ANGLE_DEG) ** 2, 1 / math.radians(LQR_ARM_RATE_DEG_S) ** 2,
         1 / LQR_CART_M ** 2, 1 / LQR_CART_SPEED_M_S ** 2, 0.0]
    Q = np.diag(q)
    R = np.array([[1 / LQR_CART_ACCEL_M_S2 ** 2]])
    P = solve_discrete_are(Az, Bz, Q, R)
    K = np.linalg.solve(R + Bz.T @ P @ Bz, Bz.T @ P @ Az)
    # Kalman filter (steady state): angles and cart position measured
    C = np.zeros((3, 6))
    C[0, 0] = C[1, 2] = C[2, 4] = 1.0
    w = ARM_DISTURBANCE_RAD_S2 * T
    Qn = np.diag([(w * T / 2) ** 2, w ** 2, (w * T / 2) ** 2, w ** 2, (M_PER_STEP / 4) ** 2, (M_PER_STEP / T / 4) ** 2])
    s_ang = ENCODER_NOISE_COUNTS * RAD_PER_COUNT
    Rn = np.diag([s_ang ** 2, (s_ang * 1.4) ** 2, (M_PER_STEP / math.sqrt(12)) ** 2])
    Pk = solve_discrete_are(Phi.T, C.T, Qn, Rn)
    L = Pk @ C.T @ np.linalg.inv(C @ Pk @ C.T + Rn)
    cl = np.abs(np.linalg.eigvals(Az - Bz @ K))
    ol = np.linalg.eigvals(A)
    return {"ratios": r, "A": A, "B": B, "Phi": Phi, "G0": G0, "G1": G1, "C": C, "K": K, "L": L,
            "closed_loop_max_abs_eig": float(cl.max()), "open_loop_eig": ol}


# ------------------------------------------------------------------ sensors and motor
def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f"{port.port}: received {len(data)}/{count} bytes in {timeout_s:.2f}s "
                           "(limit switch or stop button pressed? reset the Arduino)")
    return bytes(data)


def read_angles(esp32):
    """Read a validated v1 frame, keeping receiver reasons and wireless ages."""
    global esp32_packet_count, latest_raw_angles, latest_angle_packet, latest_angle_message
    latest_angle_packet = None
    latest_angle_message = ""
    try:
        packet = angle_reader.read_latest(esp32)
    except AngleProtocolError as exc:
        latest_raw_angles = (None, None, None)
        latest_angle_message = str(exc)
        log("SENSOR", f"ESP32_PACKET_TIMEOUT: {exc}")
        raise MissingAnglePacketError(str(exc)) from exc
    latest_angle_packet = packet
    latest_raw_angles = packet.angles
    esp32_packet_count += 1
    latest_angle_message = packet.missing_message((0, 1))
    return packet.angles


def read_required_angles(esp32):
    raw = read_angles(esp32)
    missing = [str(i + 1) for i in (0, 1) if raw[i] == 0xFFFF]
    if missing:
        log("SENSOR", f"ANGLE_MISSING channels={','.join(missing)} raw={raw}; {latest_angle_message}")
        raise MissingAnglePacketError(f"angle channel(s) {', '.join(missing)} missing; {latest_angle_message}")
    return raw


def signed_count_delta(raw, reference):
    return (raw - reference + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def circular_mean_counts(readings):
    phase = np.asarray(readings, dtype=float) * RAD_PER_COUNT
    return (np.angle(np.mean(np.exp(1j * phase))) % (2 * np.pi)) / RAD_PER_COUNT


def calibrate_hanging(esp32):
    """Arm 1 upright reference and arm 2 aligned reference, in counts, from the hanging arms."""
    log("CAL", f"Reading {HANGING_SAMPLES} packets with both arms hanging")
    readings, misses = collect_valid_samples(
        lambda: read_required_angles(esp32)[:2], HANGING_SAMPLES, interval_s=SAMPLE_SPACING_S,
        max_total_misses=MAX_HANGING_TOTAL_MISSES, max_consecutive_misses=MAX_SETUP_CONSECUTIVE_MISSES)
    if misses:
        log("CAL", f"Skipped {misses} unusable ESP32 packet(s)")
    readings = np.asarray(readings, dtype=float)
    down1 = circular_mean_counts(readings[:, 0])
    aligned2 = circular_mean_counts(readings[:, 1])
    for ch, ref in ((0, down1), (1, aligned2)):
        largest = float(np.max(np.abs(signed_count_delta(readings[:, ch], ref))))
        log("CAL", f"angle{ch + 1} first={int(readings[0, ch])} last={int(readings[-1, ch])} "
                   f"distinct={len(np.unique(readings[:, ch]))} largest_deviation={largest:.1f} counts")
        if largest > MAX_HANGING_DEVIATION_COUNTS:
            raise RuntimeError(f"Arm {ch + 1} moved during the hanging calibration ({largest:.1f} counts); "
                               "let both arms settle and restart")
    upright1 = (down1 + HALF_TURN) % ENCODER_COUNTS
    log("CAL", f"angle1 hanging={down1:.1f}, upright={upright1:.1f}; angle2 aligned={aligned2:.1f} counts")
    return upright1, aligned2


def angles_from_raw(raw1, raw2, upright1, aligned2, off1, offj):
    """Arm 1 and arm 2 absolute angles from upright and the joint angle, radians (repo sign convention)."""
    theta1 = -signed_count_delta(raw1, upright1) * RAD_PER_COUNT - off1
    joint2 = -signed_count_delta(raw2, aligned2) * RAD_PER_COUNT - offj
    return theta1, theta1 + joint2, joint2


def check_upright(esp32, upright1, aligned2, off1, offj):
    raw_samples, misses = collect_valid_samples(
        lambda: read_required_angles(esp32), UPRIGHT_SAMPLES, interval_s=SAMPLE_SPACING_S,
        max_total_misses=MAX_UPRIGHT_TOTAL_MISSES, max_consecutive_misses=MAX_SETUP_CONSECUTIVE_MISSES)
    if misses:
        log("CHECK", f"Skipped {misses} unusable ESP32 packet(s)")
    samples = [angles_from_raw(r[0], r[1], upright1, aligned2, off1, offj) for r in raw_samples]
    theta1, theta2, joint2 = np.median(np.asarray(samples), axis=0)
    log("CHECK", f"raw first={raw_samples[0][:2]} last={raw_samples[-1][:2]}; arm1={math.degrees(theta1):+.2f}deg "
                 f"arm2={math.degrees(theta2):+.2f}deg joint2={math.degrees(joint2):+.2f}deg")
    if max(abs(theta1), abs(theta2), abs(joint2)) > math.radians(MAX_START_ANGLE_DEG):
        raise RuntimeError("Upright check failed: hold both arms upright and restart. No motor command was sent.")
    return float(theta1), float(theta2)


def send_motor_command(arduino, timer_command):
    payload = struct.pack("<i", int(timer_command))
    if arduino.write(payload) != len(payload):
        raise OSError(f"Arduino accepted fewer than {len(payload)} command bytes")


def read_steps(arduino):
    steps = struct.unpack(">h", read_exact(arduino, 2))[0]
    arduino.reset_input_buffer()
    return steps


def timer_for_speed(v, hold_error_m):
    """Timer value for cart speed v (m/s, + = step count up). Below the deadband the cart holds a
    position: the slowest command (1.9 steps/s) is pointed back toward it every cycle."""
    if abs(v) < SPEED_DEADBAND_M_S:
        timer = 65535 if hold_error_m > 0 else -65535      # 65535 counts down, -65535 counts up
        return timer, 0.0
    step_hz = -v / M_PER_STEP
    icr = max(1, min(65535, int(round(TIMER_STEP_HZ / abs(step_hz))) - 1))
    timer = icr if step_hz > 0 else -icr
    return timer, step_hz


# ------------------------------------------------------------------ main
def main():
    global balancing_output_active, angle_reader
    angle_reader = AnglePacketReader()
    d = design()
    r = d["ratios"]
    Phi, G0, G1, C, K, L = d["Phi"], d["G0"], d["G1"], d["C"], d["K"], d["L"]
    log("START", f"model from {r['source']}: P1={r['P1']:.4f} P2={r['P2']:.4f} J2/J1={r['J2r']:.4f} H={r['H']:.4f}; "
                 f"falls with time constants {1000 / max(d['open_loop_eig'].real):.0f} and "
                 f"{1000 / sorted(d['open_loop_eig'].real)[-2]:.0f} ms")
    log("START", f"LQR gains (accel per [th1, th1', th2, th2', x, x', a_prev]) = {np.round(K, 2).tolist()}; "
                 f"closed-loop |eig| max {d['closed_loop_max_abs_eig']:.4f}")

    esp32 = arduino = None
    motor_command_sent = False
    stage = "opening serial ports"
    cycle = 0
    run_logger = None
    run_started_at = None
    run_stop_reason = "program exit"
    steps = None
    last_command = None
    off1 = math.radians(ARM1_OFFSET_DEG)
    offj = math.radians(JOINT2_OFFSET_DEG)
    try:
        esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
        log("START", "ESP32 and Arduino serial ports open; waiting 3s for Arduino")
        time.sleep(3)
        arduino.reset_input_buffer()
        esp32.reset_input_buffer()

        stage = "hanging calibration"
        log("READY", "Motor power OFF; center the cart and let BOTH arms hang still")
        input("Press Enter when both arms have stopped swinging: ")
        angle_reader.discard_pending(esp32)
        upright1, aligned2 = calibrate_hanging(esp32)
        log("CAL", f"starting offsets: arm1={ARM1_OFFSET_DEG:+.3f}deg joint2={JOINT2_OFFSET_DEG:+.3f}deg")

        if SAVE_RUN_CSV:
            filename = datetime.now().astimezone().strftime("balance_double_%Y%m%d_%H%M%S_%f.csv")
            run_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=DOUBLE_COLUMNS)
            run_logger.start()
            settings = {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float, str, bool, type(None)))}
            run_logger.event("config", json.dumps({
                "schema_version": 2, "angle_protocol_version": 1, "pendulum_count": 2,
                "script": "PythonMaster2_auto_improved.py",
                "force_columns": "cart acceleration command, m/s^2",
                "settings": settings, "sysid_ratios": SYSID_RATIOS, "model_ratios": {k: v for k, v in r.items() if k != "source"},
                "reference_upright_1_counts": float(upright1), "reference_aligned_2_counts": float(aligned2),
                "discrete_A": Phi.tolist(), "discrete_B": G0.tolist(), "discrete_B_delayed": G1.tolist(),
                "discrete_C": C.tolist(), "controller_gain": K.tolist(), "estimator_gain": L.tolist(),
            }, separators=(",", ":")), 0.0)
            log("LOG", f"Recording every control cycle to {run_logger.path}")

        stage = "upright check"
        log("READY", "Turn motor power ON and hold BOTH arms upright")
        input("Press Enter when both arms are upright: ")
        balancing_output_active = True
        run_started_at = time.monotonic()
        if run_logger is not None:
            run_logger.event("start", "Double pendulum balancing requested", 0.0)
        angle_reader.discard_pending(esp32)
        theta1, theta2 = check_upright(esp32, upright1, aligned2, off1, offj)

        stage = "initial Arduino command"
        motor_command_sent = True
        send_motor_command(arduino, -65535)
        last_command = -65535
        steps = read_steps(arduino)
        steps_zero = steps                       # the counter drifted while the motor was off
        t_reply = time.monotonic()
        log("START", f"Arduino replied: step count {steps} is the cart's zero")
        if run_logger is not None:
            run_logger.event("motor_start", f"initial_command={last_command}; initial_cart_steps={steps}",
                             time.monotonic() - run_started_at)

        x_hat = np.array([[theta1], [0.0], [theta2], [0.0], [0.0], [0.0]])
        x_pred = x_hat.copy()
        a_prev = 0.0          # acceleration decided last cycle
        v_state = 0.0         # cart speed at the end of the last cycle (what the model tracks)
        v_sent = 0.0          # speed actually sent to the Arduino
        hold_x = 0.0          # position held while the speed is inside the deadband
        cart_m = 0.0
        balance_t0 = time.monotonic()
        consecutive_misses = 0
        next_status = time.monotonic()
        last_status_at, last_status_cycle = next_status, 0
        previous_cycle_started = None
        recent_x = deque(maxlen=int(2.0 / SAMPLE_PERIOD_S))
        max_cycle_ms = 0.0
        log("RUN", "Double pendulum control loop started")

        while True:
            stage = "control loop"
            cycle += 1
            cycle_started = time.monotonic()
            cycle_period_ms = "" if previous_cycle_started is None else (cycle_started - previous_cycle_started) * 1000
            previous_cycle_started = cycle_started
            t_run = cycle_started - balance_t0

            stage = "reading ESP32 angles"
            sensor_read_started = time.monotonic()
            try:
                raw1, raw2, raw3 = read_angles(esp32)
            except MissingAnglePacketError:
                raw1 = raw2 = raw3 = None
            sensor_read_ms = (time.monotonic() - sensor_read_started) * 1000

            # cart position now: last reply plus the distance travelled since at the sent speed
            cart_now = cart_m + v_sent * max(0.0, time.monotonic() - t_reply)

            # ---- measurement (prediction fills in a missing channel)
            missing = []
            if raw1 is None or raw1 == 0xFFFF:
                th1_meas = float(x_pred[0, 0])
                missing.append(1)
            else:
                th1_meas = -signed_count_delta(raw1, upright1) * RAD_PER_COUNT - off1
            if raw2 is None or raw2 == 0xFFFF:
                joint_meas = float(x_pred[2, 0] - x_pred[0, 0])
                missing.append(2)
            else:
                joint_meas = -signed_count_delta(raw2, aligned2) * RAD_PER_COUNT - offj
            th2_meas = th1_meas + joint_meas
            angle_packet_message = ""
            if missing:
                consecutive_misses += 1
                angle_packet_message = (f"angle channel(s) {missing} unavailable; {latest_angle_message}; "
                                        f"using prediction, consecutive={consecutive_misses}")
                log("SENSOR", f"{angle_packet_message} at cycle={cycle}")
                if run_logger is not None:
                    run_logger.event("sensor_fallback", angle_packet_message, time.monotonic() - run_started_at)
                if consecutive_misses > MAX_RUNNING_CONSECUTIVE_MISSES:
                    raise RuntimeError(f"{consecutive_misses} consecutive unusable angle frames "
                                       f"(allowed {MAX_RUNNING_CONSECUTIVE_MISSES}); stopping motor")
            else:
                consecutive_misses = 0
            if max(abs(th1_meas), abs(th2_meas)) > math.radians(MAX_RUNNING_ANGLE_DEG):
                raise RuntimeError(f"Angle limit: arm1={math.degrees(th1_meas):+.1f}deg arm2={math.degrees(th2_meas):+.1f}deg")
            if abs(cart_now) > TRACK_SOFT_LIMIT_M:
                raise RuntimeError(f"Cart at {cart_now:+.3f} m passed TRACK_SOFT_LIMIT_M={TRACK_SOFT_LIMIT_M}")

            # ---- Kalman correction
            stage = "updating controller state"
            y = np.array([[th1_meas], [th2_meas], [cart_now]])
            predicted = x_pred
            x_hat = x_pred + L @ (y - C @ x_pred)
            x_hat[5, 0] = v_state                 # the cart speed is known exactly: we set it

            # ---- control: cart acceleration
            z = np.vstack([x_hat, [[a_prev]]])
            a = -float((K @ z)[0, 0])
            if START_RAMP_S > 0 and t_run < START_RAMP_S:
                a *= t_run / START_RAMP_S
            a = max(-CART_ACCEL_MAX_M_S2, min(CART_ACCEL_MAX_M_S2, a))
            v_end = v_state + a * SAMPLE_PERIOD_S
            if abs(v_end) > CART_SPEED_MAX_M_S:
                v_end = math.copysign(CART_SPEED_MAX_M_S, v_end)
                a = (v_end - v_state) / SAMPLE_PERIOD_S
            # send the average speed over the coming period, so the position matches the model's ramp
            v_cmd = v_state + 0.5 * a * SAMPLE_PERIOD_S
            if abs(v_cmd) >= SPEED_DEADBAND_M_S:
                hold_x = cart_now + v_cmd * SAMPLE_PERIOD_S
            timer_command, step_hz = timer_for_speed(v_cmd, cart_now - hold_x)
            v_sent = 0.0 if timer_command in (65535, -65535) else -math.copysign(
                TIMER_STEP_HZ / (abs(timer_command) + 1), timer_command) * M_PER_STEP

            stage = "sending motor command"
            send_motor_command(arduino, timer_command)
            last_command = timer_command
            stage = "reading cart position"
            position_read_started = time.monotonic()
            steps = read_steps(arduino)
            t_reply = time.monotonic()
            position_read_ms = (t_reply - position_read_started) * 1000
            cart_m = (steps - steps_zero) * M_PER_STEP

            # ---- predict the next cycle
            x_pred = Phi @ x_hat + G0 * a + G1 * a_prev
            a_before = a_prev
            a_prev = a
            v_state = v_end
            x_pred[5, 0] = v_state

            # ---- auto-trim: follow the average measured angles (the true balance point)
            off1_before, offj_before = off1, offj
            if AUTO_TRIM and t_run > TRIM_START_DELAY_S and not missing:
                k = SAMPLE_PERIOD_S / TRIM_TIME_CONSTANT_S
                lim = math.radians(TRIM_MAX_DEG)
                d1 = max(-lim - off1, min(lim - off1, k * th1_meas))
                dj = max(-lim - offj, min(lim - offj, k * joint_meas))
                off1 += d1
                offj += dj
                # keep the estimator in the new angle coordinates
                x_hat[0, 0] -= d1
                x_hat[2, 0] -= d1 + dj
                x_pred[0, 0] -= d1
                x_pred[2, 0] -= d1 + dj

            recent_x.append(cart_m)
            now = time.monotonic()
            loop_ms = (now - cycle_started) * 1000
            max_cycle_ms = max(max_cycle_ms, loop_ms)
            if run_logger is not None:
                run_logger.sample((
                    "sample", cycle_started - run_started_at, cycle, esp32_packet_count, raw1, raw2, raw3,
                    upright1 - math.degrees(off1_before) * ENCODER_COUNTS / 360.0,
                    aligned2 - math.degrees(offj_before) * ENCODER_COUNTS / 360.0,
                    th1_meas, math.degrees(th1_meas), th2_meas, math.degrees(th2_meas),
                    joint_meas, math.degrees(joint_meas), cart_now, steps - steps_zero, cart_m,
                    *predicted.flat, *x_hat.flat, a_before, a,
                    v_cmd, min(t_run / START_RAMP_S, 1.0) if START_RAMP_S > 0 else 1.0, step_hz, timer_command,
                    upright1 - math.degrees(off1) * ENCODER_COUNTS / 360.0,
                    sensor_read_ms, position_read_ms, loop_ms, cycle_period_ms,
                    *packet_csv_values(latest_angle_packet, angle_reader), angle_packet_message,
                ))
            if SHOW_BALANCING_LOGS and now >= next_status:
                rate = (cycle - last_status_cycle) / max(now - last_status_at, 1e-6)
                xs = np.asarray(recent_x)
                travel_mm = float(np.sum(np.abs(np.diff(xs)))) * 1000 / max(len(xs) * SAMPLE_PERIOD_S, 1e-3) if len(xs) > 1 else 0.0
                log("RUN", f"cycle={cycle} arm1={math.degrees(th1_meas):+6.2f}deg arm2={math.degrees(th2_meas):+6.2f}deg "
                           f"cart={cart_m * 1000:+7.1f}mm v={v_cmd * 1000:+6.1f}mm/s "
                           f"spread(2s)={np.ptp(xs) * 1000 if len(xs) else 0:5.1f}mm travel={travel_mm:5.1f}mm/s "
                           f"trim arm1={math.degrees(off1):+.3f} joint={math.degrees(offj):+.3f}deg "
                           f"rate={rate:.0f}Hz max_cycle={max_cycle_ms:.1f}ms")
                next_status = now + STATUS_INTERVAL_S
                last_status_at, last_status_cycle = now, cycle
                max_cycle_ms = 0.0

    except KeyboardInterrupt:
        run_stop_reason = "Ctrl+C"
        log("STOP", f"Ctrl+C at {stage} after {cycle} cycles")
    except Exception as exc:
        run_stop_reason = f"{type(exc).__name__}: {exc}"
        if run_logger is not None:
            run_logger.event("fault", f"stage={stage}; cycle={cycle}; raw_angles={latest_raw_angles}; cart_steps={steps}; "
                                      f"motor_command={last_command}; angle_details={latest_angle_message}; {run_stop_reason}",
                             "" if run_started_at is None else time.monotonic() - run_started_at)
        log("FAULT", f"stage={stage}, cycle={cycle}: {type(exc).__name__}: {exc}")
        traceback.print_exc()
        raise
    finally:
        try:
            if arduino is not None and arduino.is_open:
                if motor_command_sent:
                    try:
                        send_motor_command(arduino, -1)          # 0xFFFFFFFF: timer off, motor disabled
                        arduino.flush()
                        log("STOP", "Sent Arduino stop command")
                    except (serial.SerialException, OSError) as exc:
                        log("FAULT", f"Could not send stop command: {exc}")
                arduino.close()
            if esp32 is not None and esp32.is_open:
                esp32.close()
            log("STOP", "Serial ports closed")
            if cycle:
                log("STOP", f"Trims found: ARM1_OFFSET_DEG = {math.degrees(off1):.3f}, "
                            f"JOINT2_OFFSET_DEG = {math.degrees(offj):.3f} (put these at the top to start there next run)")
        finally:
            if run_logger is not None:
                warning = run_logger.stop(run_stop_reason)
                if warning:
                    log("FAULT", warning)
                else:
                    log("STOP", f"CSV saved: {run_logger.path}")


if __name__ == "__main__":
    main()
