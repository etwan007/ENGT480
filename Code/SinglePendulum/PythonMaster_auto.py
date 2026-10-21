# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

CODE_DIRECTORY = Path(__file__).resolve().parents[1]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import atexit
import traceback
import json
from datetime import datetime

from Global.run_csv_logger import RunCsvLogger
from Global.angle_protocol import AnglePacketReader, AngleProtocolError, packet_csv_values
from Global.live_offset_keys import LiveOffsetKeys
from Global.angle_packet_fallback import (MissingAnglePacketError, collect_valid_samples,
                                   estimate_with_angle)

from Global.physical_parameters import (
    SINGLE_ARM, SINGLE_CART_MASS_KG, SINGLE_CART_DRAG, SINGLE_PIVOT_DRAG,
    GRAVITY_M_S2, SAMPLE_PERIOD_S, ENCODER_COUNTS, DISTANCE_PER_6400_STEPS_M,
)
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
    return (positionRaw * DISTANCE_PER_6400_STEPS_M) / 6400

#define symbols and symbol properties
t,g,l,m1,mcart,B_cart_drag = sp.symbols('t g l m1 mcart B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
x = sp.Function('x')(t) #define x as a function of t
I,H,V,F,T_drag = sp.symbols('I H V F T_drag', real = True)

#Sample Period
Ts = SAMPLE_PERIOD_S

#0.5N rolling friction

# Physical measurements are maintained in Code/Global/physical_parameters.py.
length = SINGLE_ARM.length_m
lval = SINGLE_ARM.center_of_mass_m
Ival = SINGLE_ARM.inertia_kg_m2
m1val = SINGLE_ARM.mass_kg
T_dragVal = SINGLE_PIVOT_DRAG
mcartVal = SINGLE_CART_MASS_KG
B_cart_dragVal = SINGLE_CART_DRAG
gval = GRAVITY_M_S2

#substitutions
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

#move everything to one side of the equation (set equal to 0)
Eq1 = EL_x.lhs - EL_x.rhs
Eq2 = EL_theta.lhs - EL_theta.rhs
#set up symbols for x2div and theta2div
x2div,theta2div = sp.symbols('x2div theta2div', real = True)
#substitute in new symbols in place of accelerations
Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})
Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})

#solve equations for accelerations
Subs = sp.solve([Eq1, Eq2], [x2div, theta2div], simplify = True)

#substitute in symbols for each state
Y1,Y2,Y3,Y4 = sp.symbols('Y1 Y2 Y3 Y4')
StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, x:Y3, sp.Derivative(x,t):Y4}
F1 = Subs[theta2div].subs(StateSubs)
F2 = Subs[x2div].subs(StateSubs)

#combine into a matrix (non-linear state space model), and create state matrix (Y)
NonLinMod = sp.Matrix([Y2, F1, Y4, F2])
Y = sp.Matrix([Y1, Y2, Y3, Y4])

#linearize model
A = NonLinMod.jacobian(Y)
B = NonLinMod.diff(F)
C = sp.Matrix([ #manual input
    [1, 0, 0, 0],
    [0, 0, 1, 0],])
D = sp.Matrix([0, 0]) #manual input

#Substitute in values
Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0} #all states are 0 at equilibrium
A = A.subs(vals).subs(Equilibrium)
B = B.subs(vals).subs(Equilibrium)

#convert to a ss system, and discretize
ssCont = ctrl.ss(A, B, C, D)
ssDisc = ctrl.c2d(ssCont, Ts)




#-----TUNING VALUES-----#
#calculate lqr and kalman filter values
#K, S, E = ctrl.lqr(A, B, sp.diag(10,2,1,1), 0.5)
# Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(10,1,6,0.5), 1)
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(5,5,5,5), 1)
#QN and RN are multiplied/divided by Ts to discretize them. MATLAB does this internally
# Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1), ssDisc.C, Ts*sp.diag(0.5,0.5,0.5,0.5), (0.01/Ts)*sp.diag(1,1))
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1), ssDisc.C, sp.diag(0.5,0.5,0.5,0.5), (0.01)*sp.diag(1,1))







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
#thetaDiv = 0
u = np.array([[0.0]]) #control force
Ymeas = np.array([[0], [0]]) #measured states (no thetaDiv)
Yest = np.array([[0], [0], [0], [0]]) #estimated states
Ylast = np.array([[0], [0], [0], [0]]) #previous state storage
YfinalEst = np.array([[0], [0], [0], [0]]) #previous state storage

# The upright correction is measured at the start of every run, then adjusted
# by UPRIGHT_OFFSET_DEG.

#initialize serial
current_stage = "opening ESP32 /dev/ttyUSB0"
log("START", current_stage)
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003) #initiate communication with the ESP32
log("START", "ESP32 port open at 921600 baud")
current_stage = "opening Arduino /dev/ttyACM0"
log("START", current_stage)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003,
                        write_timeout=0.2) #initiate communication with the arduino
log("START", "Arduino port open at 115200 baud")
motor_command_sent = False

def stop_and_close():
    # This runs on a normal exit or Python exception. A Pi crash/power loss
    # still needs a motor-off switch or an Arduino command watchdog.
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
current_stage = "waiting for Arduino startup"
log("START", "Waiting 3 seconds for the Arduino to restart")
time.sleep(3) #since code is restarted gives time for arduino
log("START", "Serial startup complete")
line = 0x00000000

time.sleep(0.1)

arduino.reset_input_buffer() #clears any old logs before reading data
esp32.reset_input_buffer()
log("START", "Cleared old serial input bytes")

current_stage = "waiting for hanging pendulum"
log("READY", "Keep motor power OFF. Center the cart; let pendulum 1 hang still")
input("Press Enter when it has stopped swinging: ")
current_stage = "calibrating hanging angle from ESP32"
angle_reader.discard_pending(esp32)
automatic_correction = calibrate_from_hanging()
correction = ((automatic_correction +
               UPRIGHT_OFFSET_DEG * ENCODER_COUNTS / 360.0) % ENCODER_COUNTS)
log("CAL", f"manual upright offset={UPRIGHT_OFFSET_DEG:+.3f} degrees; "
           f"effective upright reference={correction:.1f} counts")
current_stage = "waiting for upright pendulum and motor power"
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
    }, separators=(",", ":")), 0.0)
    log("LOG", f"Recording every control cycle to {run_logger.path}")
log("READY", f"During balancing: Up/Right increases the manual offset by {LIVE_OFFSET_STEP_DEG:.3f}deg; "
             "Down/Left decreases it. Focus this terminal; no Enter needed. Ctrl+C stops.")
log("READY", "Check the printed correction; then turn motor power ON and hold pendulum 1 upright")
input("Now turn motor power ON, hold the pendulum upright, then press Enter: ")
balancing_output_active = True
run_started_at = time.monotonic()
if run_logger is not None:
    run_logger.event("start", f"upright_correction_counts={correction:.6f}", 0.0)

# Check upright angle before the first motor command.
current_stage = "checking upright angle from ESP32"
angle_reader.discard_pending(esp32)
upright_samples, missed_packets = collect_valid_samples(
    lambda: (angleRead(correction), latest_raw_angle), 10, max_total_misses=3)
upright_angles, upright_raw = zip(*upright_samples)
if missed_packets:
    log("CHECK", f"Skipped {missed_packets} unusable ESP32 packet(s)")
theta1 = float(np.median(upright_angles))
log("CHECK", f"upright raw first={upright_raw[0]}, last={upright_raw[-1]}, "
             f"distinct={len(set(upright_raw))}, "
             f"angle={np.rad2deg(theta1):+.2f} degrees relative to estimated upright")
if run_logger is not None:
    run_logger.event("check", f"upright_angle_deg={np.rad2deg(theta1):+.3f}; "
                    f"raw_first={upright_raw[0]}; raw_last={upright_raw[-1]}",
                    time.monotonic() - run_started_at)
if abs(theta1) > np.deg2rad(12):
    raise RuntimeError(f"Upright check failed ({np.rad2deg(theta1):.1f} degrees). "
                       "Motor command was not sent.")


current_stage = "enabling live manual offset keys"
if not offset_keys.start():
    log("SENSOR", "Live offset keys unavailable: run in an interactive terminal to use arrow keys")

#send low frequency control value to trigger arduino response
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
current_stage = "sending initial Arduino motor command"
arduino.write(sendPulses) #send top value to Arduino
motor_command_sent = True
last_motor_command = int(pulses)
log("START", f"Sent initial motor command={pulses}; waiting for cart position")

#get position from arduino
current_stage = "waiting for initial Arduino cart position"
x = positionRead()
log("START", f"Arduino replied; initial step count={latest_position_steps}, "
             f"cart position={x:+.4f} m")
if run_logger is not None:
    run_logger.event("motor_start", f"initial_command={pulses}; "
                     f"initial_cart_steps={latest_position_steps}; initial_cart_m={x}",
                     time.monotonic() - run_started_at)

Ylast = np.array([[theta1], [0], [x], [0]])

loop = 0
next_status_log = time.monotonic()
last_status_time = next_status_log
last_status_cycle = 0
max_cycle_ms = 0.0
previous_cycle_started = None
consecutive_angle_misses = 0
log("RUN", "Control loop started; status prints at most twice per second")

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
            # Keep the estimator in the same angle coordinates as the sensor,
            # including when this cycle must fall back to a predicted angle.
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

        #---- ANGLE CORRECTION ----
#         theta1 = theta1 - np.clip(x/30, a_min=-0.05, a_max=0.05) #correct angle towards center
        #---- ANGLE CORRECTION ----

        current_stage = "updating controller state"
        #load measurements into matrix (using position from last loop)
        cart_used_m = x
        # On one missing packet, substitute the predicted angle while still
        # correcting with the available cart position. Two misses stop the run.
        YfinalEst, theta1, consecutive_angle_misses = estimate_with_angle(
            Yest, measured_theta, x, ssDisc.C, Ld, consecutive_angle_misses)
        angle_packet_message = ""
        if measured_theta is None:
            angle_packet_message = f"{latest_angle_message}; using predicted angle"
            log("SENSOR", f"{angle_packet_message} at cycle={cycle_count}")
            if run_logger is not None:
                run_logger.event("sensor_fallback", angle_packet_message,
                                 time.monotonic() - run_started_at)
        Ylast = YfinalEst.copy() #store values for next loop
        
        #calculate control force
        u = -Kd @ YfinalEst * 2.0
        if loop < 200: #ramp force up to full
            u = u*(loop/200.0)
        force_command_n = u.item()
        ramp_fraction = min(loop / 200.0, 1.0)
        #MAY NEED TO IMPLEMENT PID CORRECTIONS FOR DRIFT
        
        #convert force to velocity and frequency (u.item takes value from 1x1 array)
#         aCart = u.item() / mcartVal #calculate target cart acceleration
#         xDivLast = xDiv #store last speed
#         xDiv = xDivLast + aCart * Ts #calulate target cart speed
        
        #pull estimated cart velocity from matrix
        xDiv = YfinalEst[3].item() #pull velocity from kalman filter estimation
#         print(round(xDiv,2))
        
        f = -(xDiv * 6400) / DISTANCE_PER_6400_STEPS_M #conversion based on measured distance per pulse
        
        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535
            
        current_stage = "sending Arduino motor command"
        #arrange as a 32 bit signed integer for transmission
        timer_command = int(pulses)
        sendPulses = struct.pack('<i', timer_command)
        arduino.write(sendPulses) #send top value to Arduino
        last_motor_command = timer_command
        
        current_stage = "waiting for Arduino cart position"
        position_read_started = time.monotonic()
        x = positionRead() #read position from arduino
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
            log("RUN", f"cycle={cycle_count} raw={latest_raw_angle} "
                       f"tilt={theta_degrees:+.2f}deg "
                       f"offset={UPRIGHT_OFFSET_DEG:+.3f}deg "
                       f"steps={latest_position_steps} cart={x:+.4f}m "
                       f"speed_est={xDiv:+.3f}m/s force_cmd={force_command_n:+.3f} "
                       f"step_hz={f:+.1f} timer_cmd={timer_command} "
                       f"rate={rate_text} max_cycle={max_cycle_ms:.1f}ms")
            last_status_time = now
            last_status_cycle = cycle_count
            max_cycle_ms = 0.0
            next_status_log = now + LOG_INTERVAL_SECONDS
        
        #---- ANGLE TUNING ----
#         correction = correction + x/60 #correct angle slightly towards center each loop
#         print(round(correction, 1)) 
        #---- ANGLE TUNING ----

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    run_stop_reason = "Ctrl+C"
    log("STOP", f"Ctrl+C received after {cycle_count} cycles")
finally:
    offset_keys.close()
