"""Cart velocity <-> Arduino timer command, identical to PythonMaster2_auto.py.

The Arduino (Code/ArduinoSlave/ArduinoSlave.ino) runs Timer3 at 250 kHz in CTC
mode with ICR3 = |command| and toggles the step pin on every compare match,
so the step rate is 125000 / |command| Hz. Commands of +/-65535 (about 1.9 Hz)
are the controllers' "stopped" value; 0xFFFFFFFF (-1 as int32) disables the
motor. Pure functions so the digital twin can reuse them without pyserial.
"""

import struct

try:  # shared value, single source of truth
    from Global.physical_parameters import DISTANCE_PER_6400_STEPS_M
except ImportError:  # pragma: no cover - only when Code/ is not on sys.path
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from Global.physical_parameters import DISTANCE_PER_6400_STEPS_M

METRES_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_TICK_HZ = 250000
STOPPED_COMMAND = 65535
STOP_MOTOR_COMMAND = -1  # 0xFFFFFFFF: Arduino disables the driver


def timer_command_for_velocity(cart_velocity):
    """Return (step_hz, timer_command) exactly like PythonMaster2_auto.py."""
    step_hz = -(cart_velocity * 6400) / DISTANCE_PER_6400_STEPS_M
    if abs(step_hz) > 2:
        timer_command = int((0.5 / step_hz) * TIMER_TICK_HZ)
    else:
        timer_command = STOPPED_COMMAND if step_hz >= 0 else -STOPPED_COMMAND
    return step_hz, timer_command


def velocity_for_timer_command(timer_command):
    """Cart velocity the Arduino actually produces for a timer command (m/s)."""
    if timer_command == 0:
        return 0.0
    ticks = min(abs(int(timer_command)), 65535)
    step_hz = 0.5 * TIMER_TICK_HZ / ticks
    # Positive commands set direction=-1 in the Arduino step counter (cart -x).
    sign = -1.0 if timer_command > 0 else 1.0
    return sign * step_hz * METRES_PER_STEP


def pack_command(timer_command):
    return struct.pack("<i", int(timer_command))


def unpack_position(reply):
    """Arduino reply: big-endian int16 step count -> (steps, metres)."""
    steps = struct.unpack(">h", reply)[0]
    return steps, steps * METRES_PER_STEP
