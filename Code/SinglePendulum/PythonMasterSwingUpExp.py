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
                         f"motor_command={last_motor_command}; {run_stop_reason}",
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
    # ESP32 sends three unsigned 16-bit angles, high byte first.
    try:
        packet = read_exact(esp32, 6)
    except TimeoutError as exc:
        log("SENSOR", f"ESP32_PACKET_TIMEOUT wall={datetime.now().astimezone().isoformat(timespec='milliseconds')} "
                      f"stage={current_stage} cycle={cycle_count} "
                      f"complete_packets_before_timeout={esp32_packet_count} "
                      f"last_valid_raw={latest_raw_angle} last_cart_steps={latest_position_steps} "
                      f"last_motor_cmd={last_motor_command}: {exc}")
        latest_raw_angles = (None, None, None)
        esp32.reset_input_buffer()  # discard a partial packet before the next cycle
        raise MissingAnglePacketError(str(exc)) from exc
    received_at = time.monotonic()
    esp32_packet_count += 1
    angles = struct.unpack('>HHH', packet)
    latest_raw_angles = angles
    esp32.reset_input_buffer()  # keep the newest ESP32 packet next time
    if any(value != 0xFFFF and value >= ENCODER_COUNTS for value in angles):
        log("SENSOR", f"ANGLE_PACKET_INVALID wall={datetime.now().astimezone().isoformat(timespec='milliseconds')} "
                      f"stage={current_stage} cycle={cycle_count} "
                      f"complete_packet={esp32_packet_count} bytes={packet.hex()} "
                      f"raw_angles={angles}")
        raise MissingAnglePacketError(f"Invalid ESP32 angle packet: {angles}")
    if angles[0] == 0xFFFF:
        age_ms = (None if last_good_angle_read_at is None else
                  (received_at - last_good_angle_read_at) * 1000)
        age_text = "none" if age_ms is None else f"{age_ms:.1f}"
        log("SENSOR", f"ANGLE1_MISSING wall={datetime.now().astimezone().isoformat(timespec='milliseconds')} "
                      f"stage={current_stage} cycle={cycle_count} "
                      f"complete_packet={esp32_packet_count} bytes={packet.hex()} "
                      f"raw_angles={angles} last_valid_raw={latest_raw_angle} "
                      f"ms_since_last_valid_Pi_read={age_text} "
                      f"last_cart_steps={latest_position_steps} last_motor_cmd={last_motor_command}")
        raise MissingAnglePacketError("First pendulum angle is missing (ESP32 sent 65535)")
    latest_raw_angle = angles[0]
    last_good_angle_read_at = received_at
    return angles[0]
 
 
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
# SWING-UP PARAMETERS
# Adjust k_energy, a_max_swing, theta_catch, and thetaDot_catch to tune.
# =====================================================================
Ival_eff       = Ival + m1val * lval**2   # effective pendulum rotational inertia
E_ref          = m1val * gval * lval       # target energy = upright equilibrium (θ=0, θ̇=0)
 
k_energy       = 10.0    # energy injection gain — start here, tune between 5 and 20
a_max_swing    = 2.0     # max cart acceleration during swing-up (m/s²)
v_max_swing    = 0.45    # max cart speed during swing-up (m/s)
k_center       = 1.5     # soft centering gain — prevents cart drifting to rail ends
 
theta_catch    = 0.35    # radians (~20°) from upright: attempt LQG catch inside this
thetaDot_catch = 5.0     # max |θ̇| (rad/s) allowed when switching to LQG
theta_fallback = 1.0     # if LQG loses it past this angle (rad), return to swing-up
 
MODE_SWINGUP   = 0
MODE_LQG       = 1
mode           = MODE_SWINGUP
 
# Finite-difference θ̇ estimator state
theta_prev     = None    # initialised after the first angle read below
theta_dot_est  = 0.0
alpha_td       = 0.6     # IIR coeff: 0 = raw derivative, 1 = completely frozen
 
# Swing-up motor velocity integrator
xDiv_swing     = 0.0
 
# Initialised here so the logger can reference them before the first cycle
E_pend         = 0.0
E_err          = 0.0
# =====================================================================
 
 
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
        "schema_version": 1, "sample_period_s": Ts,
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
        "swing_up_k_energy": k_energy,
        "swing_up_a_max": a_max_swing,
        "swing_up_v_max": v_max_swing,
        "swing_up_k_center": k_center,
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
 
# Initialise the finite-difference θ̇ estimator with the first angle reading.
theta_prev = theta1
 
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
 
        # ---- ADDED: θ̇ estimation via finite differences + IIR filter ----
        # Uses the raw measured angle (before Kalman correction) for best accuracy.
        # Handles the ±π wrap when the pendulum passes through the bottom.
        if measured_theta is not None and theta_prev is not None:
            d_theta = measured_theta - theta_prev
            if d_theta >  np.pi:  d_theta -= 2 * np.pi
            if d_theta < -np.pi:  d_theta += 2 * np.pi
            theta_dot_raw = d_theta / Ts
            theta_dot_est = alpha_td * theta_dot_est + (1 - alpha_td) * theta_dot_raw
        if measured_theta is not None:
            theta_prev = measured_theta
        # ---- END ADDED ----
 
        current_stage = "updating controller state"
        cart_used_m = x
        YfinalEst, theta1, consecutive_angle_misses = estimate_with_angle(
            Yest, measured_theta, x, ssDisc.C, Ld, consecutive_angle_misses)
        angle_packet_message = ""
        if measured_theta is None:
            angle_packet_message = "ESP32 angle missing; using predicted angle"
            log("SENSOR", f"{angle_packet_message} at cycle={cycle_count}")
            if run_logger is not None:
                run_logger.event("sensor_fallback", angle_packet_message,
                                 time.monotonic() - run_started_at)
        Ylast = YfinalEst.copy()
 
        # ---- ADDED: pendulum energy for swing-up control ----
        # E_pend = total mechanical energy; E_ref = energy at upright equilibrium.
        # E_err is negative when the pendulum needs more energy (i.e. below top).
        E_pend = (0.5 * Ival_eff * theta_dot_est**2
                  + m1val * gval * lval * np.cos(theta1))
        E_err = E_pend - E_ref
        # ---- END ADDED ----
 
        # ---- CHANGED: mode-switched control (swing-up or LQG) ----
        if mode == MODE_SWINGUP:
            current_stage = "swing-up energy control"
            # Åström-Furuta energy pump: drive cart acceleration proportional to
            # the energy error and the current swing state (θ̇·cos θ).
            a_swing  = k_energy * E_err * theta_dot_est * np.cos(theta1)
            # Soft centering keeps the cart from drifting to the rail ends.
            a_swing += k_center * (-x)
            a_swing  = np.clip(a_swing, -a_max_swing, a_max_swing)
 
            # Integrate to a target cart velocity (stepper is velocity-controlled).
            xDiv_swing += a_swing * Ts
            xDiv_swing  = np.clip(xDiv_swing, -v_max_swing, v_max_swing)
 
            # Store equivalent force in u so the Kalman prediction stays reasonable.
            u = np.array([[a_swing * (m1val + mcartVal)]])
            force_command_n = u.item()
            ramp_fraction   = 1.0  # no ramp during swing-up
 
            # Check catch condition: near upright and slow enough for LQG to hold.
            if abs(theta1) < theta_catch and abs(theta_dot_est) < thetaDot_catch:
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
                # Seed the velocity integrator from the current Kalman estimate
                # so the cart doesn't jump discontinuously.
                xDiv_swing = YfinalEst[3].item()
                log("RUN", f"*** LOST BALANCE *** returning to swing-up: "
                            f"theta={np.rad2deg(theta1):.1f}deg")
                if run_logger is not None:
                    run_logger.event("mode_switch",
                                     f"to=SWING_UP; theta_deg={np.rad2deg(theta1):.3f}; "
                                     f"thetaDot_rad_s={theta_dot_est:.3f}",
                                     time.monotonic() - run_started_at)
        # ---- END CHANGED ----
 
        # ---- CHANGED: select xDiv source based on active mode ----
        xDiv = xDiv_swing if mode == MODE_SWINGUP else YfinalEst[3].item()
        # ---- END CHANGED ----
 
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
            # ---- CHANGED: status log now shows mode, energy, and thetaDot ----
            mode_tag = "SWING" if mode == MODE_SWINGUP else "LQG  "
            log("RUN", f"[{mode_tag}] cycle={cycle_count} raw={latest_raw_angle} "
                       f"tilt={theta_degrees:+.2f}deg "
                       f"offset={UPRIGHT_OFFSET_DEG:+.3f}deg "
                       f"thetaDot={theta_dot_est:+.2f}r/s "
                       f"E={E_pend:.4f}/{E_ref:.4f} "
                       f"steps={latest_position_steps} cart={x:+.4f}m "
                       f"speed_est={xDiv:+.3f}m/s force_cmd={force_command_n:+.3f} "
                       f"step_hz={f:+.1f} timer_cmd={timer_command} "
                       f"rate={rate_text} max_cycle={max_cycle_ms:.1f}ms")
            # ---- END CHANGED ----
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
 