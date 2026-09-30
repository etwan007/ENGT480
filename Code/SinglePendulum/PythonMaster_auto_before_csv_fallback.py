import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys
import atexit
import traceback
from datetime import datetime

ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
LOG_INTERVAL_SECONDS = 0.5
started_at = time.monotonic()
current_stage = "starting"
cycle_count = 0
latest_raw_angle = None
latest_position_steps = None
last_good_angle_read_at = None
esp32_packet_count = 0
last_motor_command = None

def log(tag, message):
    elapsed = time.monotonic() - started_at
    print(f"[{elapsed:7.2f}s] [{tag}] {message}", flush=True)


def report_uncaught(exc_type, exc, tb):
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
    global latest_raw_angle, last_good_angle_read_at, esp32_packet_count
    # ESP32 sends three unsigned 16-bit angles, high byte first.
    try:
        packet = read_exact(esp32, 6)
    except TimeoutError as exc:
        log("SENSOR", f"ESP32_PACKET_TIMEOUT wall={datetime.now().astimezone().isoformat(timespec='milliseconds')} "
                      f"stage={current_stage} cycle={cycle_count} "
                      f"complete_packets_before_timeout={esp32_packet_count} "
                      f"last_valid_raw={latest_raw_angle} last_cart_steps={latest_position_steps} "
                      f"last_motor_cmd={last_motor_command}: {exc}")
        raise
    received_at = time.monotonic()
    esp32_packet_count += 1
    angles = struct.unpack('>HHH', packet)
    esp32.reset_input_buffer()  # keep the newest ESP32 packet next time
    if any(value != 0xFFFF and value >= ENCODER_COUNTS for value in angles):
        raise ValueError(f"Invalid ESP32 angle packet: {angles}")
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
        raise RuntimeError("First pendulum angle is missing (ESP32 sent 65535)")
    latest_raw_angle = angles[0]
    last_good_angle_read_at = received_at
    return angles[0]


def calibrate_from_hanging(sample_count=50):
    """Use the hanging position to estimate the opposite, upright position."""
    log("CAL", f"Reading {sample_count} ESP32 angle packets with the motor off")
    sample_started = time.monotonic()
    readings = []
    for index in range(sample_count):
        readings.append(read_raw_angle1())
        if index < sample_count - 1:
            time.sleep(0.02)  # ESP32 sends at 100 Hz; use separated readings.
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

#0.5N rolling friction

#actual system values
length = 0.2
lval = 0.05 #(mweight*length+mrod*(length/2))/(mweight+mrod) #in meters (center of mass)  #FIX THIS YOU IDIOT
Ival = 0.003 #((mweight + (mrod/3))*lval**2) #rotational inertia
m1val = 0.260 #mweight + mrod #in kg

#actual system values
# length = 0.25
# mweight = 0.031 #weight of pendulum weight
# mrod = 0.072 #weight of pendulum arm
# lval = 0.16 #(mweight*length+mrod*(length/2))/(mweight+mrod) #in meters (center of mass)
T_dragVal = 0.0 #pendulum drag force
# Ival = ((mweight + (mrod/3))*length**2) #rotational inertia
mcartVal = 0.969 #in kg
B_cart_dragVal = 0.5 #drag coefficient
gval = 9.81 #gravitational constant
# m1val = mweight + mrod #in kg
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

# The upright correction is measured at the start of every run below.

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
correction = calibrate_from_hanging()
current_stage = "waiting for upright pendulum and motor power"
log("READY", "Check the printed correction; then turn motor power ON and hold pendulum 1 upright")
input("Now turn motor power ON, hold the pendulum upright, then press Enter: ")

# Check upright angle before the first motor command.
current_stage = "checking upright angle from ESP32"
upright_angles = []
upright_raw = []
for index in range(10):
    upright_angles.append(angleRead(correction))
    upright_raw.append(latest_raw_angle)
    if index < 9:
        time.sleep(0.02)
theta1 = float(np.median(upright_angles))
log("CHECK", f"upright raw first={upright_raw[0]}, last={upright_raw[-1]}, "
             f"distinct={len(set(upright_raw))}, "
             f"angle={np.rad2deg(theta1):+.2f} degrees relative to estimated upright")
if abs(theta1) > np.deg2rad(12):
    raise RuntimeError(f"Upright check failed ({np.rad2deg(theta1):.1f} degrees). "
                       "Motor command was not sent.")


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

Ylast = np.array([[theta1], [0], [x], [0]])

loop = 0
next_status_log = time.monotonic()
last_status_time = next_status_log
last_status_cycle = 0
max_cycle_ms = 0.0
log("RUN", "Control loop started; status prints at most twice per second")

try:
    while True:
        cycle_count += 1
        cycle_started = time.monotonic()
        if loop < 250:
            loop = loop + 1
        current_stage = "predicting controller state"
        #calculate predicted states (Kalman filter part 1)
        Yest = ssDisc.A @ Ylast + ssDisc.B @ u 


        current_stage = "reading ESP32 angle"
        theta1 = angleRead(correction)#read angle

        #---- ANGLE CORRECTION ----
#         theta1 = theta1 - np.clip(x/30, a_min=-0.05, a_max=0.05) #correct angle towards center
        #---- ANGLE CORRECTION ----

        current_stage = "updating controller state"
        #load measurements into matrix (using position from last loop)
        Ymeas = np.array([[theta1], [x]])
        
        #Factor in measured states(Kalman filter part 2) (@ for matrix multiplication)
        YfinalEst = Yest + Ld @ (Ymeas - ssDisc.C @ Yest)
        Ylast = YfinalEst.copy() #store values for next loop
        
        #calculate control force
        u = -Kd @ YfinalEst * 2.0
        if loop < 200: #ramp force up to full
            u = u*(loop/200.0)
        #MAY NEED TO IMPLEMENT PID CORRECTIONS FOR DRIFT
        
        #convert force to velocity and frequency (u.item takes value from 1x1 array)
#         aCart = u.item() / mcartVal #calculate target cart acceleration
#         xDivLast = xDiv #store last speed
#         xDiv = xDivLast + aCart * Ts #calulate target cart speed
        
        #pull estimated cart velocity from matrix
        xDiv = YfinalEst[3].item() #pull velocity from kalman filter estimation
#         print(round(xDiv,2))
        
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
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
        last_motor_command = int(pulses)
        
        current_stage = "waiting for Arduino cart position"
        x = positionRead() #read position from arduino

        now = time.monotonic()
        loop_ms = (now - cycle_started) * 1000
        max_cycle_ms = max(max_cycle_ms, loop_ms)
        if now >= next_status_log:
            if last_status_cycle == 0:
                rate_text = "starting"
            else:
                observed_hz = ((cycle_count - last_status_cycle) /
                               max(now - last_status_time, 1e-6))
                rate_text = f"{observed_hz:.1f}Hz"
            log("RUN", f"cycle={cycle_count} raw={latest_raw_angle} "
                       f"tilt={np.rad2deg(theta1):+.2f}deg "
                       f"steps={latest_position_steps} cart={x:+.4f}m "
                       f"speed_est={xDiv:+.3f}m/s force_cmd={u.item():+.3f} "
                       f"step_hz={f:+.1f} timer_cmd={int(pulses)} "
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
    log("STOP", f"Ctrl+C received after {cycle_count} cycles")
