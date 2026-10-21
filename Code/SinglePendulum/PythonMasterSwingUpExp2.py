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
from angle_protocol import AnglePacketReader, AngleProtocolError, packet_csv_values
from live_offset_keys import LiveOffsetKeys
from angle_packet_fallback import (MissingAnglePacketError, collect_valid_samples,
                                   estimate_with_angle)

ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
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
latest_raw_angles = None
latest_position_steps = None
last_good_angle_read_at = None
esp32_packet_count = 0
last_motor_command = None
offset_keys = LiveOffsetKeys()
angle_reader = AnglePacketReader()
latest_angle_packet = None
latest_angle_message = ""

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
                         f"raw_angles={latest_raw_angles}; cart_steps={latest_position_steps}; "
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


def read_raw_angle1():
    global latest_raw_angle, latest_raw_angles, last_good_angle_read_at, esp32_packet_count
    global latest_angle_packet, latest_angle_message
    latest_angle_packet = None
    latest_angle_message = ""
    try:
        packet = angle_reader.read_latest(esp32)
    except AngleProtocolError as exc:
        latest_raw_angles = (None, None, None)
        latest_angle_message = str(exc)
        log("SENSOR", f"ESP32_PACKET_TIMEOUT cycle={cycle_count}: {exc}")
        raise MissingAnglePacketError(str(exc)) from exc
    latest_angle_packet = packet
    esp32_packet_count += 1
    latest_raw_angles = packet.angles
    received_at = time.monotonic()
    if packet.angles[0] == 0xFFFF:
        latest_angle_message = packet.channel_message(0)
        age_ms = (None if last_good_angle_read_at is None else
                  (received_at - last_good_angle_read_at) * 1000)
        age_text = "none" if age_ms is None else f"{age_ms:.1f}"
        log("SENSOR", f"ANGLE1_MISSING wall={datetime.now().astimezone().isoformat(timespec='milliseconds')} "
                      f"stage={current_stage} cycle={cycle_count} "
                      f"sequence={packet.sequence} {latest_angle_message} "
                      f"raw_angles={packet.angles} last_valid_raw={latest_raw_angle} "
                      f"ms_since_last_valid_Pi_read={age_text} "
                      f"last_cart_steps={latest_position_steps} last_motor_cmd={last_motor_command}")
        raise MissingAnglePacketError(latest_angle_message)
    latest_raw_angle = packet.angles[0]
    last_good_angle_read_at = received_at
    return packet.angles[0]


def calibrate_from_hanging(sample_count=50):
    """Use the hanging position to estimate the opposite, upright position."""
    log("CAL", f"Reading {sample_count} ESP32 angle packets with the motor off")
    sample_started = time.monotonic()
    readings, missed_packets = collect_valid_samples(read_raw_angle1, sample_count)
    if missed_packets:
        log("CAL", f"Skipped {missed_packets} unusable ESP32 packet(s)")
    readings = np.array(readings, dtype=float)
    phase = readings * (2 * np.pi / ENCODER_COUNTS)
    mean_phase = np.angle(np.mean(np.exp(1j * phase))) % (2 * np.pi)
    down = mean_phase * ENCODER_COUNTS / (2 * np.pi)
    deviations = (readings - down + HALF_TURN) % ENCODER_COUNTS - HALF_TURN
    max_deviation = np.max(np.abs(deviations))
    log("CAL", f"raw first={readings[0]:.0f}, last={readings[-1]:.0f}, "
               f"distinct={len(np.unique(readings))}, "
               f"elapsed={time.monotonic() - sample_started:.2f}s, "
               f"largest deviation={max_deviation:.1f} counts")
    if max_deviation > 80:
        raise RuntimeError("Hanging angle moved too much; let it settle and restart")
    upright = (down + HALF_TURN) % ENCODER_COUNTS
    log("CAL", f"hanging={down:.1f}, estimated upright={upright:.1f} counts")
    return upright


def angleRead(cor):
    raw = read_raw_angle1()
    delta = (raw - cor + HALF_TURN) % ENCODER_COUNTS - HALF_TURN
    return delta * (2 * np.pi / ENCODER_COUNTS)

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

A = NonLinMod.jacobian(Y)
B = NonLinMod.diff(F)
C = sp.Matrix([
    [1, 0, 0, 0],
    [0, 0, 1, 0],])
D = sp.Matrix([0, 0])

Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0}
A = A.subs(vals).subs(Equilibrium)
B = B.subs(vals).subs(Equilibrium)

ssCont = ctrl.ss(A, B, C, D)
ssDisc = ctrl.c2d(ssCont, Ts)


#-----TUNING VALUES-----#
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(5,5,5,5), 1)
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1), ssDisc.C, sp.diag(0.5,0.5,0.5,0.5), (0.01)*sp.diag(1,1))


# =====================================================================
# SWING-UP PARAMETERS — Resonant bang-bang (phase-synchronous)
#
# Simple principle: after the startup kick, the cart always moves in the
# direction the pendulum is RETURNING toward the hang position. The cart
# direction switches when theta_dot_est crosses zero (pendulum at its peak,
# about to reverse). This adds energy to the pendulum on every half-swing.
#
# Encoder sign convention (user-confirmed):
#   Clockwise encoder rotation  → LEFT  cart movement
#   Counter-clockwise rotation  → RIGHT cart movement
#   Motor command: f = -(xDiv * 6400) / 0.638175
#   Positive xDiv → negative step frequency → cart moves LEFT
#
# When theta_dot_est > 0: pendulum swinging counter-clockwise (right-ish),
#   about to return clockwise (left). Cart should move LEFT → xDiv_swing > 0.
# When theta_dot_est < 0: pendulum swinging clockwise (left-ish),
#   about to return counter-clockwise (right). Cart should move RIGHT → xDiv_swing < 0.
# So: cart_direction = +sign(theta_dot_est).
#
# Tuning guide:
#   v_max_swing:      Cart speed during pumping (m/s). 0.50 is a safe start.
#   switch_threshold: |theta_dot_est| must exceed this to update cart direction.
#                     Prevents noise at zero crossing from chattering the relay.
#                     0.15 rad/s is a good start; lower = more responsive, higher = safer.
#   startup_kick_v:   Initial push from stationary. Just enough to start the pendulum.
# =====================================================================
Ival_eff       = Ival + m1val * lval**2
E_ref          = m1val * gval * lval

v_max_swing      = 0.50   # cart speed during resonant pumping (m/s)
switch_threshold = 0.40   # |theta_dot_est| hysteresis to update direction (rad/s)
#                           Raised from 0.15 — a low threshold lets sensor noise
#                           chatter the relay direction near zero crossing.
startup_kick_v   = 0.20   # cart speed for initial push from stationary (m/s)

# Kept for logging only (energy-pump params no longer drive control)
k_energy       = 0.0
k_center       = 0.0

theta_catch    = 0.5    # radians (~29°) from upright: attempt LQG catch inside this
thetaDot_catch = 5.0    # max |θ̇| (rad/s) allowed when switching to LQG
theta_fallback = 1.0    # if LQG loses it past this angle (rad), return to swing-up

MODE_SWINGUP   = 0
MODE_LQG       = 1
mode           = MODE_SWINGUP

# Finite-difference θ̇ estimator state
theta_prev     = None    # initialised after the first angle read below
theta_prev_receiver_ms = None
theta_dot_est  = 0.0
alpha_td       = 0.6     # IIR coeff: 0 = raw derivative, 1 = completely frozen

# Swing-up motor velocity state.
xDiv_swing     = 0.0
cart_direction = 1      # ±1: direction the cart moves; updated by resonant pump
swing_phase    = "startup"  # "startup" or "pumping" — for status logging
swing_above_horizontal = False

# Initialised here so the logger can reference them before the first cycle
E_pend         = 0.0
E_err          = 0.0
# =====================================================================


def update_theta_velocity(measured_theta, packet):
    """Use receiver-frame elapsed time, including skipped or missing readings."""
    global theta_prev, theta_prev_receiver_ms, theta_dot_est
    if measured_theta is None:
        return
    receiver_ms = packet.receiver_ms
    if theta_prev is not None and theta_prev_receiver_ms is not None:
        elapsed_ms = (receiver_ms - theta_prev_receiver_ms) & 0xFFFFFFFF
        if 0 < elapsed_ms < 0x80000000:
            d_theta = (measured_theta - theta_prev + np.pi) % (2 * np.pi) - np.pi
            theta_dot_raw = d_theta / (elapsed_ms / 1000.0)
            theta_dot_est = alpha_td * theta_dot_est + (1 - alpha_td) * theta_dot_raw
        else:
            # Re-seed on receiver restart or a zero-time interval.
            theta_dot_est = 0.0
    theta_prev = measured_theta
    theta_prev_receiver_ms = receiver_ms


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

current_stage = "opening ESP32 /dev/ttyUSB0"
log("START", current_stage)
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003)
log("START", "ESP32 port open at 921600 baud")
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
        if esp32.is_open:
            esp32.close()
            log("STOP", "ESP32 port closed")
    finally:
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
esp32.reset_input_buffer()
log("START", "Cleared old serial input bytes")

log("READY", "Keep motor power OFF and pendulum 1 hanging still during calibration")
current_stage = "calibrating hanging angle from ESP32"
angle_reader.discard_pending(esp32)
automatic_correction = calibrate_from_hanging()
correction = ((automatic_correction +
               UPRIGHT_OFFSET_DEG * ENCODER_COUNTS / 360.0) % ENCODER_COUNTS)
log("CAL", f"manual upright offset={UPRIGHT_OFFSET_DEG:+.3f} degrees; "
           f"effective upright reference={correction:.1f} counts")
current_stage = "waiting for motor power on (swing-up mode)"
if SAVE_RUN_CSV:
    filename = datetime.now().astimezone().strftime("balance_%Y%m%d_%H%M%S_%f.csv")
    new_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename)
    new_logger.start()
    run_logger = new_logger
    run_logger.event("config", json.dumps({
        "schema_version": 2, "angle_protocol_version": 1, "sample_period_s": Ts,
        "pendulum_length_m": length, "center_of_mass_m": lval,
        "pendulum_inertia_kg_m2": Ival, "pendulum_mass_kg": m1val,
        "cart_mass_kg": mcartVal, "cart_drag": B_cart_dragVal,
        "pendulum_drag": T_dragVal, "gravity_m_s2": gval,
        "automatic_upright_correction_counts": float(automatic_correction),
        "manual_upright_offset_deg": UPRIGHT_OFFSET_DEG,
        "live_offset_step_deg": LIVE_OFFSET_STEP_DEG,
        "upright_correction_counts": float(correction),
        "controller_gain": np.asarray(Kd).tolist(),
        "estimator_gain": np.asarray(Ld).tolist(),
        "discrete_A": np.asarray(ssDisc.A).tolist(),
        "discrete_B": np.asarray(ssDisc.B).tolist(),
        "discrete_C": np.asarray(ssDisc.C).tolist(),
        # Swing-up config
        "swing_up_control": "resonant_bang_bang",
        "swing_up_v_max": v_max_swing,
        "swing_up_switch_threshold_rad_s": switch_threshold,
        "swing_up_startup_kick_v": startup_kick_v,
        "swing_up_theta_catch_rad": theta_catch,
        "swing_up_thetaDot_catch": thetaDot_catch,
        "swing_up_theta_fallback_rad": theta_fallback,
    }, separators=(",", ":")), 0.0)
    log("LOG", f"Recording every control cycle to {run_logger.path}")

# ---- CHANGED: swing-up starts with pendulum hanging, not held upright ----
log("READY", "SWING-UP MODE: Turn motor power ON with the pendulum hanging freely.")
log("READY", "The system will automatically swing up and catch the pendulum.")
log("READY", f"During balancing: Up/Right increases manual offset by {LIVE_OFFSET_STEP_DEG:.3f}deg; "
             "Down/Left decreases it. Focus this terminal; no Enter needed. Ctrl+C stops.")
input("Turn motor power ON (pendulum hanging freely), then press Enter: ")
balancing_output_active = True
run_started_at = time.monotonic()
if run_logger is not None:
    run_logger.event("start", f"upright_correction_counts={correction:.6f}", 0.0)

# Read initial angle from hanging position (no upright check needed for swing-up).
# Expect theta1 ≈ ±π if the pendulum is hanging straight down.
current_stage = "reading initial hanging angle from ESP32"
angle_reader.discard_pending(esp32)
initial_samples, missed_packets = collect_valid_samples(
    lambda: (angleRead(correction), latest_raw_angle), 10, max_total_misses=3)
initial_angles, initial_raw = zip(*initial_samples)
if missed_packets:
    log("CHECK", f"Skipped {missed_packets} unusable ESP32 packet(s)")
theta1 = float(np.median(initial_angles))
log("CHECK", f"initial raw first={initial_raw[0]}, last={initial_raw[-1]}, "
             f"distinct={len(set(initial_raw))}, "
             f"angle={np.rad2deg(theta1):+.2f} degrees relative to estimated upright "
             f"(expect ≈ ±180° if hanging)")
if run_logger is not None:
    run_logger.event("check", f"initial_angle_deg={np.rad2deg(theta1):+.3f}; "
                    f"raw_first={initial_raw[0]}; raw_last={initial_raw[-1]}",
                    time.monotonic() - run_started_at)
# No upright angle reject — swing-up starts from the hanging position.
# ---- END CHANGED ----

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

# Pair the last measured angle with its receiver timestamp for the derivative.
theta_prev = float(initial_angles[-1])
theta_prev_receiver_ms = latest_angle_packet.receiver_ms

loop = 0
next_status_log = time.monotonic()
last_status_time = next_status_log
last_status_cycle = 0
max_cycle_ms = 0.0
previous_cycle_started = None
consecutive_angle_misses = 0
log("RUN", "Control loop started in SWING-UP mode; status prints at most twice per second")

try:
    while True:
        cycle_count += 1
        cycle_started = time.monotonic()
        cycle_period_ms = ("" if previous_cycle_started is None else
                           (cycle_started - previous_cycle_started) * 1000)
        swing_dt = Ts if previous_cycle_started is None else cycle_period_ms / 1000.0
        previous_cycle_started = cycle_started
        if loop < 250:
            loop = loop + 1
        current_stage = "reading live manual offset keys"
        offset_steps = offset_keys.poll_steps()
        if offset_steps:
            offset_delta_deg = offset_steps * LIVE_OFFSET_STEP_DEG
            UPRIGHT_OFFSET_DEG += offset_delta_deg
            correction = ((automatic_correction + UPRIGHT_OFFSET_DEG *
                           ENCODER_COUNTS / 360.0) % ENCODER_COUNTS)
            Ylast[0, 0] -= np.deg2rad(offset_delta_deg)
            if theta_prev is not None:
                theta_prev = ((theta_prev - np.deg2rad(offset_delta_deg) + np.pi)
                              % (2 * np.pi) - np.pi)
            offset_message = (f"cycle={cycle_count}; manual_upright_offset_deg={UPRIGHT_OFFSET_DEG:+.6f}; "
                              f"delta_deg={offset_delta_deg:+.6f}; upright_correction_counts={correction:.6f}")
            if run_logger is not None:
                run_logger.event("manual_offset", offset_message,
                                 time.monotonic() - run_started_at)
        current_stage = "predicting controller state"
        previous_force_n = u.item()
        #calculate predicted states (Kalman filter part 1)
        Yest = ssDisc.A @ Ylast + ssDisc.B @ u

        current_stage = "reading ESP32 angle"
        sensor_read_started = time.monotonic()
        try:
            measured_theta = angleRead(correction)
        except MissingAnglePacketError:
            measured_theta = None
        sensor_read_ms = (time.monotonic() - sensor_read_started) * 1000

        # Receiver timestamps account for skipped frames and missed readings.
        update_theta_velocity(measured_theta, latest_angle_packet)

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

        # ---- Pendulum energy for swing-up control ----
        # E_pend = total mechanical energy; E_ref = energy at upright equilibrium.
        # (E_ref - E_pend) is positive when the pendulum needs more energy.
        E_pend = (0.5 * Ival_eff * theta_dot_est**2
                  + m1val * gval * lval * np.cos(theta1))
        E_err = E_pend - E_ref   # negative = below target (kept for logging)
        # ---- END ----

        # ---- Mode-switched control (swing-up or LQG) ----
        if mode == MODE_SWINGUP:
            current_stage = "swing-up resonant bang-bang"

            # --- Phase-synchronous resonant pumping ---
            # After the startup kick, the cart always moves in the direction the
            # pendulum is RETURNING toward the hang position.
            #
            # Direction switch logic:
            #   When theta_dot_est > 0 (pendulum moving CCW / rightward), it will
            #   soon reverse and return CW (leftward). We want the cart to move LEFT
            #   so its motion adds to the pendulum's return energy.
            #   LEFT cart movement = positive xDiv (motor sign: f = -(xDiv*6400)/0.638175).
            #   So: cart_direction = +sign(theta_dot_est).
            #
            #   Hysteresis: only update cart_direction when |theta_dot_est| > switch_threshold.
            #   This prevents noise near zero from chattering the relay direction.

            theta_used = measured_theta if measured_theta is not None else theta1

            if abs(theta_dot_est) < 0.05 and abs(abs(theta_used) - np.pi) < 0.20:
                # Startup: pendulum stationary near the bottom — give a small push.
                xDiv_swing = startup_kick_v
                swing_phase = "startup"

            else:
                # Update direction when the angular velocity is clear of noise threshold.
                # Sign convention:
                #   theta_dot_est > 0 → pendulum swinging CCW (right-ish)
                #   Pendulum will return CW (left-ish) → cart should move RIGHT → xDiv < 0
                #   So cart_direction = -sign(theta_dot_est).
                # (Inverted from naive expectation because positive xDiv → LEFT per motor wiring.)
                if abs(theta_dot_est) > switch_threshold:
                    cart_direction = -int(np.sign(theta_dot_est))
                # Command full speed in the chosen direction.
                xDiv_swing = cart_direction * v_max_swing
                swing_phase = "pumping"

            YfinalEst[3, 0] = xDiv_swing      # keep Kalman cart speed in sync
            Ylast = YfinalEst.copy()
            swing_message = (f"mode=SWING_UP; phase={swing_phase}; "
                             f"thetaDot_est={theta_dot_est:.3f}rad/s; "
                             f"cart_dir={cart_direction:+d}; "
                             f"energy_j={E_pend:.4f}/{E_ref:.4f}; "
                             f"cart_velocity_command_m_s={xDiv_swing:.3f}")
            angle_packet_message = "; ".join(
                message for message in (angle_packet_message, swing_message) if message)
            if (measured_theta is not None and not swing_above_horizontal
                    and abs(theta1) < np.pi / 2):
                swing_above_horizontal = True
                log("RUN", f"*** ABOVE HORIZONTAL *** theta={np.rad2deg(theta1):.1f}deg")
                if run_logger is not None:
                    run_logger.event("swing_above_horizontal", swing_message,
                                     time.monotonic() - run_started_at)

            # No force measurement available for velocity-driven swing-up.
            u = np.array([[0.0]])
            force_command_n = u.item()
            ramp_fraction   = 1.0  # no ramp during swing-up

            # Check catch condition: near upright and slow enough for LQG to hold.
            if (measured_theta is not None and abs(theta1) < theta_catch
                    and abs(theta_dot_est) < thetaDot_catch):
                mode       = MODE_LQG
                loop       = 0        # restart LQG ramp
                xDiv_swing = 0.0
                # Seed the Kalman θ̇ estimate with our measured derivative so the
                # controller has a realistic velocity on the very first LQG cycle.
                YfinalEst[1, 0] = theta_dot_est
                Ylast = YfinalEst.copy()
                log("RUN", f"*** CATCH *** switching to LQG: "
                            f"theta={np.rad2deg(theta1):.1f}deg  "
                            f"thetaDot={theta_dot_est:.2f}rad/s")
                if run_logger is not None:
                    run_logger.event("mode_switch",
                                     f"to=LQG; theta_deg={np.rad2deg(theta1):.3f}; "
                                     f"thetaDot_rad_s={theta_dot_est:.3f}; "
                                     f"E_pend={E_pend:.4f}; E_ref={E_ref:.4f}",
                                     time.monotonic() - run_started_at)

        elif mode == MODE_LQG:
            current_stage = "LQG balance control"
            u = -Kd @ YfinalEst * 2.0
            if loop < 200:  # ramp force up to full over the first 200 cycles
                u = u * (loop / 200.0)
            force_command_n = u.item()
            ramp_fraction   = min(loop / 200.0, 1.0)

            # Fall back to swing-up if balance is lost.
            if abs(theta1) > theta_fallback:
                mode       = MODE_SWINGUP
                xDiv_swing = 0.0   # start from rest; pump formula takes over immediately
                swing_above_horizontal = False
                log("RUN", f"*** LOST BALANCE *** returning to swing-up: "
                            f"theta={np.rad2deg(theta1):.1f}deg")
                if run_logger is not None:
                    run_logger.event("mode_switch",
                                     f"to=SWING_UP; theta_deg={np.rad2deg(theta1):.3f}; "
                                     f"thetaDot_rad_s={theta_dot_est:.3f}",
                                     time.monotonic() - run_started_at)
        # ---- END ----

        # ---- Select xDiv source based on active mode ----
        xDiv = xDiv_swing if mode == MODE_SWINGUP else YfinalEst[3].item()
        # ---- END ----

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
                esp32_packet_count, *latest_raw_angles, correction,
                theta1, theta_degrees, cart_used_m,
                latest_position_steps, x,
                *Yest.flat, *YfinalEst.flat,
                previous_force_n, force_command_n, ramp_fraction,
                f, timer_command, sensor_read_ms, position_read_ms,
                loop_ms, cycle_period_ms,
                *packet_csv_values(latest_angle_packet, angle_reader), angle_packet_message,
            ))
        max_cycle_ms = max(max_cycle_ms, loop_ms)
        if SHOW_BALANCING_LOGS and now >= next_status_log:
            if last_status_cycle == 0:
                rate_text = "starting"
            else:
                observed_hz = ((cycle_count - last_status_cycle) /
                               max(now - last_status_time, 1e-6))
                rate_text = f"{observed_hz:.1f}Hz"
            mode_tag = "SWING" if mode == MODE_SWINGUP else "LQG  "
            phase_tag = swing_phase if mode == MODE_SWINGUP else "balance"
            log("RUN", f"[{mode_tag}|{phase_tag}] cycle={cycle_count} raw={latest_raw_angle} "
                       f"tilt={theta_degrees:+.2f}deg "
                       f"offset={UPRIGHT_OFFSET_DEG:+.3f}deg "
                       f"thetaDot={theta_dot_est:+.2f}r/s dir={cart_direction:+d} "
                       f"E={E_pend:.4f}/{E_ref:.4f} "
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
