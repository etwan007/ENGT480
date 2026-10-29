import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys
import atexit
import traceback
import json
from datetime import datetime
from pathlib import Path

from run_csv_logger import RunCsvLogger
from live_offset_keys import LiveOffsetKeys
from angle_packet_fallback import (MissingAnglePacketError, collect_valid_samples,
                                   estimate_with_angle)
from sensor_angle_source import SensorAngleSource

SENSOR_MAX_AGE_US = 100_000
SENSOR_ANGLE_SIGN = 1.0  # Set to -1.0 if sensor_1 positive rotation opposes the controller model.
TIMESTAMP_MASK = (1 << 64) - 1
MAX_IDENTIFY_TIMESTAMP_GAP_US = 1_000_000
LOG_INTERVAL_SECONDS = 0.5
# Added to the automatically calibrated upright reference. Positive values
# make the reported angle smaller (for example, +0.30 subtracts 0.30 degrees).
UPRIGHT_OFFSET_DEG = 0.0
# During balancing: Up/Right adds this many degrees; Down/Left subtracts it.
LIVE_OFFSET_STEP_DEG = 0.05
# Set to False to hide routine output after the motor-on Enter prompt,
# including control-loop status and normal shutdown messages.
SHOW_BALANCING_LOGS = True
# Each control cycle is queued without waiting for disk I/O. A full queue drops
# samples instead of delaying the motor controller; the CSV reports the count.
SAVE_RUN_CSV = True
RUN_LOG_DIRECTORY = Path(__file__).resolve().parent / "run_logs"
started_at = time.monotonic()
balancing_output_active = False
run_logger = None
run_stop_reason = "program exit"
run_started_at = None
current_stage = "starting"
cycle_count = 0
latest_raw_angle = None
latest_sensor_sample = None
latest_position_steps = None
sensor_source = None
last_motor_command = None
offset_keys = LiveOffsetKeys()
latest_angle_message = ""
identify_last_timestamp_us = None
identify_elapsed_us = 0

SENSOR_CSV_COLUMNS = (
    "record_type", "elapsed_s", "cycle", "sensor_seq", "sensor_timestamp_us",
    "sensor_angle_rad", "sensor_rad_s", "sensor_age_us", "upright_reference_rad",
    "measured_angle_rad", "measured_angle_deg", "cart_used_m", "cart_steps", "cart_m",
    "pred_angle_rad", "pred_angle_rate_rad_s", "pred_cart_m", "pred_cart_velocity_m_s",
    "est_angle_rad", "est_angle_rate_rad_s", "est_cart_m", "est_cart_velocity_m_s",
    "previous_force_n", "force_command_n", "ramp_fraction", "step_frequency_hz",
    "motor_timer_command", "sensor_read_ms", "position_read_ms", "loop_ms",
    "cycle_period_ms", "message",
)

def log(tag, message):
    # Keep fault diagnostics visible even when routine balancing logs are off.
    if balancing_output_active and not SHOW_BALANCING_LOGS and tag not in ("FAULT", "SENSOR"):
        return
    elapsed = time.monotonic() - started_at
    print(f"[{elapsed:7.2f}s] [{tag}] {message}", flush=True)


def report_uncaught(exc_type, exc, tb):
    global run_stop_reason
    run_stop_reason = f"{exc_type.__name__}: {exc}"
    if run_logger is not None:
        run_logger.event("fault", f"stage={current_stage}; cycle={cycle_count}; "
                         f"sensor_sample={latest_sensor_sample}; cart_steps={latest_position_steps}; "
                         f"motor_command={last_motor_command}; angle_details={latest_angle_message}; {run_stop_reason}",
                         "" if run_started_at is None else time.monotonic() - run_started_at)
    log("FAULT", f"stage={current_stage}, cycle={cycle_count}: "
                 f"{exc_type.__name__}: {exc}")
    traceback.print_exception(exc_type, exc, tb)


sys.excepthook = report_uncaught

def read_exact(port, count, timeout_seconds=0.1):
    """Read a whole binary message, or fail instead of waiting forever."""
    deadline = time.monotonic() + timeout_seconds
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f"{port.port}: received {len(data)}/{count} bytes "
                           f"within {timeout_seconds:.2f} s")
    return data


def wrap_angle(angle):
    return (float(angle) + np.pi) % (2 * np.pi) - np.pi


def read_sensor1_angle():
    global latest_raw_angle, latest_sensor_sample
    global latest_angle_message
    latest_angle_message = ""
    sample = sensor_source.latest(max_age_us=SENSOR_MAX_AGE_US)
    if sample is None or sample.angle is None:
        age_sample = sensor_source.latest()
        latest_sensor_sample = age_sample
        if age_sample is None:
            detail = "no sensor_1 reading received"
        elif age_sample.angle is None:
            detail = f"sensor_1 angle invalid; age_us={age_sample.age_us}"
        else:
            detail = f"sensor_1 reading stale; age_us={age_sample.age_us}"
        latest_angle_message = detail
        raise MissingAnglePacketError(detail)
    latest_sensor_sample = sample
    latest_raw_angle = sample.angle
    return wrap_angle(SENSOR_ANGLE_SIGN * sample.angle)


def calibrate_from_hanging(sample_count=50):
    """Use the hanging position to estimate the opposite, upright position."""
    log("CAL", f"Reading {sample_count} sensor_1 angles with the motor off")
    sample_started = time.monotonic()
    readings, missed_packets = collect_valid_samples(read_sensor1_angle, sample_count)
    if missed_packets:
        log("CAL", f"Skipped {missed_packets} unusable sensor_1 reading(s)")
    readings = np.array(readings, dtype=float)
    down = float(np.angle(np.mean(np.exp(1j * readings))))
    deviations = np.array([wrap_angle(value - down) for value in readings])
    max_deviation = np.max(np.abs(deviations))
    log("CAL", f"angle first={readings[0]:+.4f}, last={readings[-1]:+.4f} rad, "
               f"distinct={len(np.unique(readings))}, "
               f"elapsed={time.monotonic() - sample_started:.2f}s, "
               f"largest deviation={np.rad2deg(max_deviation):.2f} degrees")
    if max_deviation > np.deg2rad(80 * 360 / 16384):
        raise RuntimeError("Hanging angle moved too much; let it settle and restart")
    upright = wrap_angle(down + np.pi)
    log("CAL", f"hanging={down:+.4f} rad, estimated upright reference={upright:+.4f} rad")
    return upright


def angleRead(cor):
    raw = read_sensor1_angle()
    return wrap_angle(raw - cor)

def positionRead():
    global latest_position_steps
    line2 = read_exact(arduino, 2)
    positionRaw = struct.unpack('>h', line2)[0]
    latest_position_steps = positionRaw
    arduino.reset_input_buffer()
    return (positionRaw * 0.638175) / 6400

#define symbols and symbol properties
t,g,l,m1,mcart,B_cart_drag = sp.symbols('t g l m1 mcart B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
x = sp.Function('x')(t) #define x as a function of t
I,H,V,F,T_drag = sp.symbols('I H V F T_drag', real = True)

#Sample Period
Ts = 1/100

#actual system values
length = 0.2
lval = 0.05
Ival = 0.003
m1val = 0.260

T_dragVal = 0.0
mcartVal = 0.969
B_cart_dragVal = 0.5
gval = 9.81
vals = {m1:m1val,mcart:mcartVal,l:lval,g:gval,I:Ival,T_drag:T_dragVal,B_cart_drag:B_cart_dragVal}

#define equations
T = (1/2)*(mcart+m1)*x.diff(t)**2 - m1*l*sp.cos(theta)*x.diff(t)*theta.diff(t) + (1/2)*(m1*l**2+I)*theta.diff(t)**2
U = m1*g*l*sp.cos(theta)

L = T - U

EL_x = sp.Eq(
    sp.diff(sp.diff(L, x.diff(t)), t) - sp.diff(L, x),
    F - B_cart_drag * x.diff(t)
    )

EL_theta = sp.Eq(
    sp.diff(sp.diff(L, theta.diff(t)), t) - sp.diff(L, theta),
    -T_drag
    )

Eq1 = EL_x.lhs - EL_x.rhs
Eq2 = EL_theta.lhs - EL_theta.rhs
x2div,theta2div = sp.symbols('x2div theta2div', real = True)
Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})
Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})

Subs = sp.solve([Eq1, Eq2], [x2div, theta2div], simplify = True)

Y1,Y2,Y3,Y4 = sp.symbols('Y1 Y2 Y3 Y4')
StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, x:Y3, sp.Derivative(x,t):Y4}
F1 = Subs[theta2div].subs(StateSubs)
F2 = Subs[x2div].subs(StateSubs)

NonLinMod = sp.Matrix([Y2, F1, Y4, F2])
Y = sp.Matrix([Y1, Y2, Y3, Y4])

A_sym = NonLinMod.jacobian(Y)
B_sym = NonLinMod.diff(F)
C = sp.Matrix([
    [1, 0, 0, 0],
    [0, 0, 1, 0],])
D = sp.Matrix([0, 0])

Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0}


def design_lqg(I_value):
    """Linearise at upright with inertia I_value, discretise, and compute the LQR and Kalman gains."""
    v = dict(vals)
    v[I] = I_value
    A_num = A_sym.subs(v).subs(Equilibrium)
    B_num = B_sym.subs(v).subs(Equilibrium)
    ss_disc = ctrl.c2d(ctrl.ss(A_num, B_num, C, D), Ts)
    #-----TUNING VALUES-----#
    K, _, _ = ctrl.dlqr(ss_disc, sp.diag(40,5,150,60), 1)   # raised from 5,5,5,5: firmer catch, faster return to centre
    L_gain, _, _ = ctrl.dlqe(ss_disc.A, sp.diag(1,1,1,1), ss_disc.C, sp.diag(0.5,0.5,0.5,0.5), (0.01)*sp.diag(1,1))
    return ss_disc, K, L_gain


# Designed from the model now; redesigned after IDENTIFY measures the real swing.
ssDisc, Kd, Ld = design_lqg(Ival)


# =====================================================================
# SWING-UP
# 1. IDENTIFY: a short cart wiggle sets the arm swinging, then the cart stops
#    and 3 s of hanging swings are timed. The measured natural frequency w0
#    replaces the model's in the energy below, and the LQG is redesigned with
#    an inertia that gives that w0, so neither depends on lval/Ival being right.
# 2. SWING: energy pumping. Normalised energy e = 0.5*(thetaDot/w0)**2 + cos(theta)
#    is -1 hanging still and +1 at rest upright. The cart accelerates with
#    sign(thetaDot*cos(theta)) until e reaches the target, plus a soft spring
#    that keeps the cart near the centre. The target rises after a swing that
#    falls short (friction) and drops after one that goes over too fast.
# 3. LQG: near upright, slow enough, with enough energy and the cart near the
#    centre, the estimator is seeded with the measured state and the LQG takes
#    over with no force ramp. While balancing, the upright reference trims
#    itself so an angle offset doesn't park the cart off centre. If the arm
#    falls past BALANCE_LOST_RAD, or the cart runs too far, it swings up again.
# =====================================================================
SWING_ACCEL_MAX     = 6.0    # m/s^2, cart acceleration while pumping
SWING_SPEED_MAX     = 0.6    # m/s, cart speed while pumping
SWING_ENERGY_GAIN   = 4.0    # full acceleration while e is more than 0.25 short
SWING_ENERGY_TARGET = 1.03   # starting target; a little over 1 covers pivot friction on the last swing
SWING_TARGET_STEP   = 0.03   # lowered by this after a swing that goes over the top too fast;
                             # raised by the shortfall after a swing that turns back near the top
SWING_TARGET_MIN    = 0.97
SWING_TARGET_MAX    = 1.30
SWING_CENTER_KP     = 12.0   # 1/s^2, cart centering spring while pumping
SWING_CENTER_KV     = 6.0    # 1/s, cart centering damping while pumping
SWING_CENTER_SHAPE  = 1      # 1: centre mostly while the arm is sideways, where it costs no swing energy

ID_KICK_ACCEL       = 3.0    # m/s^2, identification wiggle (+a, -a, -a, +a: returns to start)
ID_KICK_TIME        = 0.12   # s per wiggle segment
ID_SETTLE_S         = 0.3    # s after the wiggle before timing starts
ID_COAST_S          = 3.0    # s of free swinging that is timed, at most
ID_HALF_SWINGS      = 4      # ... or stop timing once this many half swings are seen

# While balancing, a constant angle error (encoder or calibration) parks the cart off centre.
# The upright reference is trimmed by (KX*cart + KV*cart_speed) degrees per second until it doesn't.
AUTO_TRIM_KX        = 4.0    # deg/s per metre of cart offset
AUTO_TRIM_KV        = 5.0    # deg/s per m/s of cart speed
AUTO_TRIM_MAX_DEG   = 3.0    # total trim limit
AUTO_TRIM_DELAY_S   = 0.5    # no trimming right after a catch, while the catch itself moves the cart

CATCH_ANGLE_RAD     = 0.30   # switch to LQG inside this angle from upright (~17 deg)
CATCH_RATE_RAD_S    = 3.5    # ... and below this angular speed
CATCH_ENERGY_MIN    = 0.95   # ... and with about the energy to reach upright
CATCH_ENERGY_MAX    = 1.15
CATCH_CART_MAX_M    = 0.12   # ... and with the cart near the start, so the catch has room
BALANCE_LOST_RAD    = 0.80   # LQG gives up past this angle and swings up again
BALANCE_CART_LIMIT_M = 0.22  # ... or when the cart gets this far from the start while catching
LQG_SPEED_MAX       = 0.8    # m/s, clip on the LQG cart speed command
LQG_ACCEL_MAX       = 15.0   # m/s^2, clip on how fast that command may change (avoids missed steps)
TRACK_SOFT_LIMIT_M  = 0.30   # stop the motor if the cart passes this (limit switches are farther out)

MODE_IDENTIFY  = 0
MODE_SWINGUP   = 1
MODE_LQG       = 2
MODE_NAMES     = {MODE_IDENTIFY: "IDENT", MODE_SWINGUP: "SWING", MODE_LQG: "LQG  "}
mode           = MODE_IDENTIFY

# Natural frequency used for the energy: the model's value until IDENTIFY measures it.
w0_model       = float(np.sqrt(m1val * gval * lval / (Ival + m1val * lval**2)))
w0             = w0_model
identify_started = None
identify_log   = []          # (unwrapped server timestamp ms, angle from hanging)
theta_dot_est  = 0.0         # sensor_client rad_s, already radians per second

# Swing-up cart velocity command and status
energy_target  = SWING_ENERGY_TARGET
swing_direction = 0          # sign of thetaDot with hysteresis, to spot turning points
xDiv_swing     = 0.0
xDiv_sent      = 0.0         # last cart speed command sent to the Arduino
swing_accel    = 0.0
swing_above_horizontal = False
catch_count    = 0
catch_time     = 0.0
e_norm         = -1.0
theta_prev_swing = None
swing_energy_peak = -1.0
# =====================================================================


def identify_time_ms(timestamp_us):
    """Unwrap the server's uint64 microsecond clock; re-seed after start/reconnect."""
    global identify_last_timestamp_us, identify_elapsed_us, identify_log
    if timestamp_us is None:
        return None
    timestamp_us = int(timestamp_us) & TIMESTAMP_MASK
    if identify_last_timestamp_us is None:
        identify_last_timestamp_us = timestamp_us
        return identify_elapsed_us / 1000.0
    delta_us = (timestamp_us - identify_last_timestamp_us) & TIMESTAMP_MASK
    identify_last_timestamp_us = timestamp_us
    if delta_us == 0:
        return None
    if delta_us > MAX_IDENTIFY_TIMESTAMP_GAP_US:
        # A start/reconnect resets the server clock origin. A normal uint64
        # rollover instead produces a small modular delta and remains valid.
        identify_log.clear()
        identify_elapsed_us = 0
        return None
    identify_elapsed_us += delta_us
    return identify_elapsed_us / 1000.0


def identify_accel(t_id):
    """Wiggle +a, -a, -a, +a: the cart ends where it started, at rest."""
    k = int(t_id // ID_KICK_TIME)
    return {0: ID_KICK_ACCEL, 1: -ID_KICK_ACCEL, 2: -ID_KICK_ACCEL, 3: ID_KICK_ACCEL}.get(k, 0.0)


def measure_natural_frequency(samples):
    """Return (w0, period_s, amplitude_rad, decay_per_swing) from timed free swings, or None."""
    if len(samples) < 50:
        return None
    t = np.array([s[0] for s in samples], dtype=float) / 1000.0
    a = np.array([s[1] for s in samples], dtype=float)
    a = a - np.mean(a)
    crossings = []
    for i in range(1, len(a)):
        if a[i - 1] < 0 <= a[i] or a[i - 1] > 0 >= a[i]:
            f = a[i - 1] / (a[i - 1] - a[i])
            crossings.append(t[i - 1] + f * (t[i] - t[i - 1]))
    if len(crossings) < 4:
        return None
    half = np.diff(crossings)
    period = 2.0 * float(np.median(half))
    amplitude = float(np.max(np.abs(a)))
    period0 = period / (1.0 + amplitude**2 / 16.0)     # remove the large-swing stretch
    if not 0.3 < period0 < 3.0:
        return None
    # Peak of each half swing, for how fast friction takes the swing away.
    edges = np.searchsorted(t, crossings)
    peaks = [float(np.max(np.abs(a[i:j]))) for i, j in zip(edges[:-1], edges[1:]) if j > i]
    decay = 0.0
    if len(peaks) >= 3 and peaks[0] > 0 and peaks[-1] > 0:
        decay = 1.0 - (peaks[-1] / peaks[0]) ** (2.0 / (len(peaks) - 1))
    return 2.0 * np.pi / period0, period0, amplitude, decay, len(half)


def swing_command(theta_now, theta_dot, cart_x, v_cmd, dt):
    """Energy pumping with a soft centering spring; returns (new speed, accel, e)."""
    e = 0.5 * (theta_dot / w0)**2 + np.cos(theta_now)
    pump = np.clip(SWING_ENERGY_GAIN * (energy_target - e), -1.0, 1.0)
    direction = np.sign(theta_dot * np.cos(theta_now))
    center = -SWING_CENTER_KP * cart_x - SWING_CENTER_KV * v_cmd
    if SWING_CENTER_SHAPE:
        # Cart acceleration changes the swing energy at a rate proportional to thetaDot*cos(theta).
        # Centre hardest where that is small: the arm sideways or about to turn back.
        center *= 1.0 / (1.0 + (theta_dot * np.cos(theta_now) / w0)**2)
    accel = SWING_ACCEL_MAX * pump * direction + center
    accel = float(np.clip(accel, -SWING_ACCEL_MAX, SWING_ACCEL_MAX))
    v_new = float(np.clip(v_cmd + accel * dt, -SWING_SPEED_MAX, SWING_SPEED_MAX))
    return v_new, accel, float(e)


#-----CONTROL CODE-------#
line = 0x00000000
positionRaw = 0x0000
angleRaw = 0x0000
x = 0 #cart position
theta = 0 #pendulum angle
xDiv = 0 #cart velocity
f = 0
conversion = 0
pulses = 0
u = np.array([[0.0]]) #control force
Ymeas = np.array([[0], [0]]) #measured states (no thetaDiv)
Yest = np.array([[0], [0], [0], [0]]) #estimated states
Ylast = np.array([[0], [0], [0], [0]]) #previous state storage
YfinalEst = np.array([[0], [0], [0], [0]]) #previous state storage

current_stage = "waiting for hanging pendulum and motor power off"
log("READY", "Turn motor power OFF. Center the cart; let pendulum 1 hang still")
input("With motor power OFF and the pendulum still, press Enter to reset the Arduino: ")

current_stage = "opening Arduino /dev/ttyACM0"
log("START", current_stage)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003,
                        write_timeout=0.2)
log("START", "Arduino port open at 115200 baud")
motor_command_sent = False

def stop_and_close():
    try:
        if arduino.is_open:
            if motor_command_sent:
                try:
                    log("STOP", "Sending 0xFFFFFFFF stop command to Arduino")
                    arduino.write(struct.pack('<I', 0xFFFFFFFF))
                    arduino.flush()
                except (serial.SerialException, OSError) as exc:
                    log("FAULT", f"Could not send Arduino stop command: {exc}")
            else:
                log("STOP", "No motor command was sent")
            arduino.close()
            log("STOP", "Arduino port closed")
    finally:
        if sensor_source is not None:
            try:
                sensor_source.close()
                log("STOP", "SensorClient stopped and disconnected")
            except Exception as exc:
                log("FAULT", f"SensorClient shutdown failed: {exc}")
        offset_keys.close()
        if run_logger is not None:
            warning = run_logger.stop(run_stop_reason)
            if warning:
                log("FAULT", warning)
            else:
                log("STOP", f"CSV saved: {run_logger.path}")

atexit.register(stop_and_close)
current_stage = "requesting Arduino reset"
log("START", "Requesting Arduino reset through USB DTR")
arduino.dtr = False
time.sleep(0.1)
arduino.dtr = True
current_stage = "waiting for Arduino startup"
log("START", "Reset requested; waiting 3 seconds for Arduino startup")
time.sleep(3)
log("START", "Serial startup complete")
line = 0x00000000

time.sleep(0.1)

arduino.reset_input_buffer()
log("START", "Cleared old Arduino serial input bytes")

log("READY", "Keep motor power OFF and pendulum 1 hanging still during sensor calibration")
current_stage = "connecting SensorClient to sensor_1"
sensor_source = SensorAngleSource()
sensor_source.start()
log("START", f"SensorClient connected to {sensor_source.uri}, subscribed to {sensor_source.sensor_name}")
current_stage = "calibrating hanging angle from sensor_1"
automatic_correction = calibrate_from_hanging()
correction = wrap_angle(automatic_correction + np.deg2rad(UPRIGHT_OFFSET_DEG))
log("CAL", f"manual upright offset={UPRIGHT_OFFSET_DEG:+.3f} degrees; "
           f"effective upright reference={correction:+.5f} rad")
current_stage = "waiting for motor power on (swing-up mode)"
if SAVE_RUN_CSV:
    filename = datetime.now().astimezone().strftime("balance_%Y%m%d_%H%M%S_%f.csv")
    new_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=SENSOR_CSV_COLUMNS)
    new_logger.start()
    run_logger = new_logger
    run_logger.event("config", json.dumps({
        "schema_version": 3, "sensor_api": "Sensor Server WebSocket", "sample_period_s": Ts,
        "sensor_name": sensor_source.sensor_name, "sensor_uri": sensor_source.uri,
        "sensor_angle_sign": SENSOR_ANGLE_SIGN,
        "pendulum_length_m": length, "center_of_mass_m": lval,
        "pendulum_inertia_kg_m2": Ival, "pendulum_mass_kg": m1val,
        "cart_mass_kg": mcartVal, "cart_drag": B_cart_dragVal,
        "pendulum_drag": T_dragVal, "gravity_m_s2": gval,
        "automatic_upright_reference_rad": float(automatic_correction),
        "manual_upright_offset_deg": UPRIGHT_OFFSET_DEG,
        "live_offset_step_deg": LIVE_OFFSET_STEP_DEG,
        "upright_reference_rad": float(correction),
        "controller_gain": np.asarray(Kd).tolist(),
        "estimator_gain": np.asarray(Ld).tolist(),
        "discrete_A": np.asarray(ssDisc.A).tolist(),
        "discrete_B": np.asarray(ssDisc.B).tolist(),
        "discrete_C": np.asarray(ssDisc.C).tolist(),
        "swing_up_control": "identify_then_normalised_energy_pumping",
        "swing_up_accel_max": SWING_ACCEL_MAX,
        "swing_up_speed_max": SWING_SPEED_MAX,
        "swing_up_energy_gain": SWING_ENERGY_GAIN,
        "swing_up_energy_target": SWING_ENERGY_TARGET,
        "swing_up_center_kp": SWING_CENTER_KP,
        "swing_up_center_kv": SWING_CENTER_KV,
        "swing_up_model_w0_rad_s": w0_model,
        "catch_angle_rad": CATCH_ANGLE_RAD,
        "catch_rate_rad_s": CATCH_RATE_RAD_S,
        "balance_lost_rad": BALANCE_LOST_RAD,
        "track_soft_limit_m": TRACK_SOFT_LIMIT_M,
    }, separators=(",", ":")), 0.0)
    log("LOG", f"Recording every control cycle to {run_logger.path}")

log("READY", "SWING-UP MODE: Turn motor power ON with the pendulum hanging freely.")
log("READY", "The cart wiggles, times a few free swings, then swings up and catches the pendulum.")
log("READY", f"During balancing: Up/Right increases manual offset by {LIVE_OFFSET_STEP_DEG:.3f}deg; "
             "Down/Left decreases it. Focus this terminal; no Enter needed. Ctrl+C stops.")
input("Turn motor power ON (pendulum hanging freely), then press Enter: ")
balancing_output_active = True
run_started_at = time.monotonic()
if run_logger is not None:
    run_logger.event("start", f"upright_reference_rad={correction:.8f}", 0.0)

# Read initial angle from hanging position (no upright check needed for swing-up).
current_stage = "reading initial hanging angle from sensor_1"
initial_samples, missed_packets = collect_valid_samples(
    lambda: (angleRead(correction), latest_raw_angle), 10, max_total_misses=3)
initial_angles, initial_raw = zip(*initial_samples)
if missed_packets:
    log("CHECK", f"Skipped {missed_packets} unusable sensor_1 reading(s)")
theta1 = float(initial_angles[-1])
hanging_error_deg = np.rad2deg((np.median(np.abs(initial_angles)) - np.pi))
log("CHECK", f"initial sensor angle first={initial_raw[0]:+.4f}, last={initial_raw[-1]:+.4f} rad, "
             f"distinct={len(set(initial_raw))}, "
             f"angle={np.rad2deg(theta1):+.2f} degrees relative to estimated upright "
             f"(expect ≈ ±180° if hanging)")
if run_logger is not None:
    run_logger.event("check", f"initial_angle_deg={np.rad2deg(theta1):+.3f}; "
                    f"sensor_angle_first={initial_raw[0]}; sensor_angle_last={initial_raw[-1]}",
                    time.monotonic() - run_started_at)
if abs(hanging_error_deg) > 20:
    raise RuntimeError(f"Pendulum is {abs(hanging_error_deg):.0f} degrees from hanging; "
                       "let it hang still and restart. Motor command was not sent.")

current_stage = "enabling live manual offset keys"
if not offset_keys.start():
    log("SENSOR", "Live offset keys unavailable: run in an interactive terminal to use arrow keys")

#send low frequency control value to trigger arduino response
pulses = -65535
sendPulses = struct.pack('<i', int(pulses))
current_stage = "sending initial Arduino motor command"
arduino.write(sendPulses)
motor_command_sent = True
last_motor_command = int(pulses)
log("START", f"Sent initial motor command={pulses}; waiting for cart position")

current_stage = "waiting for initial Arduino cart position"
x = positionRead()
log("START", f"Arduino replied; initial step count={latest_position_steps}, "
             f"cart position={x:+.4f} m")
if run_logger is not None:
    run_logger.event("motor_start", f"initial_command={pulses}; "
                     f"initial_cart_steps={latest_position_steps}; initial_cart_m={x}",
                     time.monotonic() - run_started_at)

Ylast = np.array([[theta1], [0], [x], [0]])
x_start = x

# Seed the identify timestamp baseline. The API clock is reset by start(); later
# uint64 wraparound is handled by identify_time_ms().
if latest_sensor_sample is not None:
    identify_last_timestamp_us = latest_sensor_sample.timestamp_us

loop = 0
next_status_log = time.monotonic()
last_status_time = next_status_log
last_status_cycle = 0
max_cycle_ms = 0.0
previous_cycle_started = None
consecutive_angle_misses = 0
identify_started = time.monotonic()
log("RUN", f"Control loop started in IDENTIFY mode: wiggle, then time {ID_COAST_S:.1f} s of free swings "
           f"(model predicts a {2 * np.pi / w0_model:.3f} s swing)")

try:
    while True:
        cycle_count += 1
        cycle_started = time.monotonic()
        cycle_period_ms = ("" if previous_cycle_started is None else
                           (cycle_started - previous_cycle_started) * 1000)
        swing_dt = Ts if previous_cycle_started is None else min(cycle_period_ms / 1000.0, 0.03)
        previous_cycle_started = cycle_started
        if loop < 250:
            loop = loop + 1
        current_stage = "reading live manual offset keys"
        offset_steps = offset_keys.poll_steps()
        if offset_steps:
            offset_delta_deg = offset_steps * LIVE_OFFSET_STEP_DEG
            UPRIGHT_OFFSET_DEG += offset_delta_deg
            correction = wrap_angle(automatic_correction + np.deg2rad(UPRIGHT_OFFSET_DEG))
            Ylast[0, 0] -= np.deg2rad(offset_delta_deg)
            offset_message = (f"cycle={cycle_count}; manual_upright_offset_deg={UPRIGHT_OFFSET_DEG:+.6f}; "
                              f"delta_deg={offset_delta_deg:+.6f}; upright_reference_rad={correction:.8f}")
            if run_logger is not None:
                run_logger.event("manual_offset", offset_message,
                                 time.monotonic() - run_started_at)
        current_stage = "predicting controller state"
        previous_force_n = u.item()
        #calculate predicted states (Kalman filter part 1)
        Yest = ssDisc.A @ Ylast + ssDisc.B @ u

        current_stage = "reading sensor_1 angle"
        sensor_read_started = time.monotonic()
        try:
            measured_theta = angleRead(correction)
        except MissingAnglePacketError:
            measured_theta = None
        sensor_read_ms = (time.monotonic() - sensor_read_started) * 1000

        if measured_theta is not None and latest_sensor_sample is not None:
            # SensorClient rad_s is already signed radians per second.
            theta_dot_est = (0.0 if latest_sensor_sample.rad_s is None
                             else SENSOR_ANGLE_SIGN * latest_sensor_sample.rad_s)

        current_stage = "updating controller state"
        cart_used_m = x
        YfinalEst, theta1, consecutive_angle_misses = estimate_with_angle(
            Yest, measured_theta, x, ssDisc.C, Ld, consecutive_angle_misses)
        angle_packet_message = ""
        if measured_theta is None:
            angle_packet_message = f"{latest_angle_message}; using predicted angle"
            log("SENSOR", f"{angle_packet_message} at cycle={cycle_count}")
            if run_logger is not None:
                run_logger.event("sensor_fallback", angle_packet_message,
                                 time.monotonic() - run_started_at)
        Ylast = YfinalEst.copy()

        if abs(x - x_start) > TRACK_SOFT_LIMIT_M:
            raise RuntimeError(f"Cart passed the {TRACK_SOFT_LIMIT_M:.2f} m soft limit "
                               f"({x - x_start:+.3f} m from the start); stopping motor")

        e_norm = 0.5 * (theta_dot_est / w0)**2 + np.cos(theta1)
        mode_message = ""

        if mode == MODE_IDENTIFY:
            current_stage = "identifying natural frequency"
            t_id = cycle_started - identify_started
            kick_end = 4 * ID_KICK_TIME
            if t_id < kick_end:
                swing_accel = identify_accel(t_id)
                xDiv_swing = float(np.clip(xDiv_swing + swing_accel * swing_dt,
                                           -SWING_SPEED_MAX, SWING_SPEED_MAX))
            else:
                swing_accel = 0.0
                xDiv_swing = 0.0
                if (t_id > kick_end + ID_SETTLE_S and measured_theta is not None
                        and latest_sensor_sample is not None):
                    sample_time_ms = identify_time_ms(latest_sensor_sample.timestamp_us)
                    if sample_time_ms is not None:
                        identify_log.append((sample_time_ms,
                                             (theta1 % (2 * np.pi)) - np.pi))
            coast_done = t_id > kick_end + ID_SETTLE_S + ID_COAST_S
            if not coast_done and len(identify_log) > 50 and cycle_count % 10 == 0:
                early = measure_natural_frequency(identify_log)
                coast_done = early is not None and early[4] >= ID_HALF_SWINGS
            if coast_done:
                result = measure_natural_frequency(identify_log)
                if result is None:
                    log("RUN", f"IDENTIFY could not time the swings; using the model's "
                               f"{2 * np.pi / w0_model:.3f} s swing")
                    id_message = "identify=failed; using model w0"
                else:
                    w0, period0, amplitude, decay, _ = result
                    # Same mass and CoM, inertia set so the model swings at the measured w0.
                    # For a pendulum on a cart, w0 alone fixes its response to cart acceleration.
                    Ival_identified = max(m1val * gval * lval / w0**2 - m1val * lval**2, 1e-6)
                    ssDisc, Kd, Ld = design_lqg(Ival_identified)
                    log("RUN", f"IDENTIFY hanging swing period={period0:.3f} s (w0={w0:.2f} rad/s, "
                               f"amplitude {np.rad2deg(amplitude):.1f} deg, swing shrinks {100 * decay:.1f}% per period); "
                               f"the model predicts {2 * np.pi / w0_model:.3f} s")
                    log("RUN", f"LQG redesigned with Ival={Ival_identified:.5f} kg*m^2: "
                               f"Kd={np.round(np.asarray(Kd), 3).tolist()}")
                    id_message = (f"identify=ok; period_s={period0:.4f}; w0_rad_s={w0:.4f}; "
                                  f"amplitude_deg={np.rad2deg(amplitude):.2f}; decay_per_period={decay:.4f}; "
                                  f"model_period_s={2 * np.pi / w0_model:.4f}; "
                                  f"Ival_identified={Ival_identified:.6f}; Kd={np.asarray(Kd).tolist()}")
                if run_logger is not None:
                    run_logger.event("identify", id_message, time.monotonic() - run_started_at)
                mode = MODE_SWINGUP
                log("RUN", "Switching to SWING-UP (energy pumping)")
            mode_message = f"mode=IDENTIFY; t={t_id:.3f}; cart_velocity_command_m_s={xDiv_swing:.6f}"

        elif mode == MODE_SWINGUP:
            current_stage = "swing-up energy control"
            if measured_theta is not None:
                xDiv_swing, swing_accel, e_norm = swing_command(
                    theta1, theta_dot_est, x - x_start, xDiv_swing, swing_dt)
                # Friction differs from rig to rig: adjust the energy target from what each swing does.
                swing_energy_peak = max(swing_energy_peak, e_norm)
                new_direction = (1 if theta_dot_est > 0.3 else -1 if theta_dot_est < -0.3 else swing_direction)
                if swing_direction and new_direction != swing_direction:
                    # Turned back short of the catch zone although the energy said it had enough.
                    # At a turning point e = cos(theta) exactly, so 1 - cos(theta) is the shortfall.
                    if abs(theta1) < 0.6 and swing_energy_peak >= energy_target - 0.02:
                        shortfall = 1.0 - np.cos(theta1)
                        energy_target = min(energy_target + min(0.6 * shortfall, 0.05), SWING_TARGET_MAX)
                        log("RUN", f"swing turned back at {np.rad2deg(theta1):+.1f}deg: "
                                   f"energy target -> {energy_target:.3f}")
                    swing_energy_peak = -1.0
                swing_direction = new_direction
                if (theta_prev_swing is not None and theta_prev_swing * theta1 < 0
                        and abs(theta1) < 0.5 and abs(theta_dot_est) >= CATCH_RATE_RAD_S):
                    energy_target = max(energy_target - SWING_TARGET_STEP, SWING_TARGET_MIN)
                    log("RUN", f"swing went over the top too fast ({theta_dot_est:+.1f} rad/s): "
                               f"energy target -> {energy_target:.2f}")
                theta_prev_swing = theta1
            if (measured_theta is not None and not swing_above_horizontal
                    and abs(theta1) < np.pi / 2):
                swing_above_horizontal = True
                log("RUN", f"*** ABOVE HORIZONTAL *** theta={np.rad2deg(theta1):.1f}deg e={e_norm:.2f}")
            mode_message = (f"mode=SWING_UP; energy_norm={e_norm:.4f}; target={energy_target:.3f}; "
                            f"accel_m_s2={swing_accel:.3f}; cart_velocity_command_m_s={xDiv_swing:.6f}")
            # Catch: close to upright and slow enough for the LQG to hold.
            if (measured_theta is not None and abs(theta1) < CATCH_ANGLE_RAD
                    and abs(theta_dot_est) < CATCH_RATE_RAD_S
                    and CATCH_ENERGY_MIN <= e_norm <= CATCH_ENERGY_MAX
                    and abs(x - x_start) < CATCH_CART_MAX_M):
                mode = MODE_LQG
                catch_count += 1
                catch_time = cycle_started
                loop = 250                       # no force ramp: the arm is already moving
                Ylast = np.array([[theta1], [theta_dot_est], [x], [xDiv_swing]])
                YfinalEst = Ylast.copy()
                u = np.array([[0.0]])
                log("RUN", f"*** CATCH #{catch_count} *** switching to LQG: "
                           f"theta={np.rad2deg(theta1):.1f}deg thetaDot={theta_dot_est:.2f}rad/s "
                           f"cart={x:+.3f}m speed={xDiv_swing:+.3f}m/s")
                if run_logger is not None:
                    run_logger.event("mode_switch",
                                     f"to=LQG; theta_deg={np.rad2deg(theta1):.3f}; "
                                     f"thetaDot_rad_s={theta_dot_est:.3f}; energy_norm={e_norm:.4f}",
                                     time.monotonic() - run_started_at)

        elif mode == MODE_LQG:
            current_stage = "LQG balance control"
            if abs(theta1) > BALANCE_LOST_RAD or abs(x - x_start) > BALANCE_CART_LIMIT_M:
                mode = MODE_SWINGUP
                xDiv_swing = float(np.clip(YfinalEst[3].item(), -SWING_SPEED_MAX, SWING_SPEED_MAX))
                swing_above_horizontal = False
                swing_direction, theta_prev_swing, swing_energy_peak = 0, None, -1.0
                log("RUN", f"*** LOST BALANCE *** returning to swing-up: theta={np.rad2deg(theta1):.1f}deg "
                           f"cart={x - x_start:+.3f}m")
                if run_logger is not None:
                    run_logger.event("mode_switch",
                                     f"to=SWING_UP; theta_deg={np.rad2deg(theta1):.3f}; "
                                     f"thetaDot_rad_s={theta_dot_est:.3f}",
                                     time.monotonic() - run_started_at)
            elif (AUTO_TRIM_KX or AUTO_TRIM_KV) and cycle_started - catch_time > AUTO_TRIM_DELAY_S:
                # Slowly move the upright reference until the cart balances at the centre.
                trim_deg = (AUTO_TRIM_KX * x + AUTO_TRIM_KV * YfinalEst[3].item()) * swing_dt
                new_offset = float(np.clip(UPRIGHT_OFFSET_DEG + trim_deg, -AUTO_TRIM_MAX_DEG, AUTO_TRIM_MAX_DEG))
                trim_deg = new_offset - UPRIGHT_OFFSET_DEG
                UPRIGHT_OFFSET_DEG = new_offset
                correction = wrap_angle(automatic_correction + np.deg2rad(UPRIGHT_OFFSET_DEG))
                Ylast[0, 0] -= np.deg2rad(trim_deg)
            mode_message = f"mode=LQG; upright_offset_deg={UPRIGHT_OFFSET_DEG:+.4f}"

        # ---- output: force for the estimator, cart speed for the motor ----
        if mode == MODE_LQG:
            u = -Kd @ YfinalEst * 2.0
            if loop < 200:  # ramp force up to full over the first 200 cycles
                u = u * (loop / 200.0)
            force_command_n = u.item()
            ramp_fraction = min(loop / 200.0, 1.0)
            dv_max = LQG_ACCEL_MAX * swing_dt
            xDiv = float(np.clip(YfinalEst[3].item(), -LQG_SPEED_MAX, LQG_SPEED_MAX))
            xDiv = float(np.clip(xDiv, xDiv_sent - dv_max, xDiv_sent + dv_max))
        else:
            # The linear estimator is only valid near upright. Keep it tracking the
            # measured state so a single missing packet predicts something sensible.
            u = np.array([[0.0]])
            force_command_n = 0.0
            ramp_fraction = 1.0
            Ylast = np.array([[theta1], [theta_dot_est], [x], [xDiv_swing]])
            YfinalEst = Ylast.copy()
            xDiv = xDiv_swing
        angle_packet_message = "; ".join(m for m in (angle_packet_message, mode_message) if m)

        xDiv_sent = xDiv
        f = -(xDiv * 6400) / 0.638175 #conversion based on measured distance per pulse

        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535

        current_stage = "sending Arduino motor command"
        timer_command = int(pulses)
        sendPulses = struct.pack('<i', timer_command)
        arduino.write(sendPulses)
        last_motor_command = timer_command

        current_stage = "waiting for Arduino cart position"
        position_read_started = time.monotonic()
        x = positionRead()
        position_read_ms = (time.monotonic() - position_read_started) * 1000

        now = time.monotonic()
        loop_ms = (now - cycle_started) * 1000
        theta_degrees = float(np.rad2deg(theta1))
        if run_logger is not None:
            run_logger.sample((
                "sample", cycle_started - run_started_at, cycle_count,
                "" if latest_sensor_sample is None or latest_sensor_sample.seq is None else latest_sensor_sample.seq,
                "" if latest_sensor_sample is None or latest_sensor_sample.timestamp_us is None else latest_sensor_sample.timestamp_us,
                "" if latest_sensor_sample is None or latest_sensor_sample.angle is None else latest_sensor_sample.angle,
                "" if latest_sensor_sample is None or latest_sensor_sample.rad_s is None else latest_sensor_sample.rad_s,
                "" if latest_sensor_sample is None else latest_sensor_sample.age_us,
                correction, theta1, theta_degrees, cart_used_m,
                latest_position_steps, x,
                *Yest.flat, *YfinalEst.flat,
                previous_force_n, force_command_n, ramp_fraction,
                f, timer_command, sensor_read_ms, position_read_ms,
                loop_ms, cycle_period_ms, angle_packet_message,
            ))
        max_cycle_ms = max(max_cycle_ms, loop_ms)
        if SHOW_BALANCING_LOGS and now >= next_status_log:
            if last_status_cycle == 0:
                rate_text = "starting"
            else:
                observed_hz = ((cycle_count - last_status_cycle) /
                               max(now - last_status_time, 1e-6))
                rate_text = f"{observed_hz:.1f}Hz"
            log("RUN", f"[{MODE_NAMES[mode]}] cycle={cycle_count} sensor_angle={latest_raw_angle:+.4f}rad "
                       f"tilt={theta_degrees:+.2f}deg "
                       f"offset={UPRIGHT_OFFSET_DEG:+.3f}deg "
                       f"thetaDot={theta_dot_est:+.2f}r/s "
                       f"E={e_norm:+.3f} "
                       f"steps={latest_position_steps} cart={x:+.4f}m "
                       f"speed_cmd={xDiv:+.3f}m/s force_cmd={force_command_n:+.3f} "
                       f"step_hz={f:+.1f} timer_cmd={timer_command} "
                       f"rate={rate_text} max_cycle={max_cycle_ms:.1f}ms")
            last_status_time = now
            last_status_cycle = cycle_count
            max_cycle_ms = 0.0
            next_status_log = now + LOG_INTERVAL_SECONDS

#this section kills the program
except KeyboardInterrupt:
    run_stop_reason = "Ctrl+C"
    log("STOP", f"Ctrl+C received after {cycle_count} cycles")
finally:
    offset_keys.close()
    if catch_count:
        log("STOP", f"Upright offset at the end: {UPRIGHT_OFFSET_DEG:+.3f} deg "
                    f"(set UPRIGHT_OFFSET_DEG to this to start the trim there next run)")
