"""Balance the double pendulum using encoder references measured at startup.

Run on the Raspberry Pi with the ESP32 receiver and Arduino motor controller.
Both links must hang motionless for calibration. This script does not swing them up.
"""

import re
import struct
import time
import traceback
import json
from datetime import datetime
from pathlib import Path

import numpy as np
import serial

from run_csv_logger import DOUBLE_COLUMNS, RunCsvLogger


ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
SAMPLE_PERIOD_S = 0.01
HANGING_SAMPLES = 50
UPRIGHT_SAMPLES = 10
SAMPLE_SPACING_S = 0.02
MAX_HANGING_DEVIATION_COUNTS = 80
MAX_START_ANGLE_DEG = 12
MAX_RUNNING_ANGLE_RAD = 1.0
STATUS_INTERVAL_S = 0.5
DISTANCE_PER_6400_STEPS_M = 0.638175
# Set False to hide routine terminal output after the motor-on Enter prompt.
SHOW_BALANCING_LOGS = True
# CSV writes happen on a separate thread; the control loop never waits for disk.
SAVE_RUN_CSV = True
RUN_LOG_DIRECTORY = Path(__file__).resolve().parent / "run_logs"

started_at = time.monotonic()
balancing_output_active = False
esp32_packet_count = 0
latest_raw_angles = None


def log(tag, message):
    if balancing_output_active and not SHOW_BALANCING_LOGS and tag not in ("FAULT", "SENSOR"):
        return
    print(f"[{time.monotonic() - started_at:7.2f}s] [{tag}] {message}", flush=True)


def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(
            f"{port.port}: received {len(data)}/{count} bytes in {timeout_s:.2f}s"
        )
    return bytes(data)


def read_angles(esp32):
    """Read the receiver's three big-endian 16-bit channels; require links 1 and 2."""
    global esp32_packet_count, latest_raw_angles
    packet = read_exact(esp32, 6)
    raw = struct.unpack(">HHH", packet)
    esp32_packet_count += 1
    latest_raw_angles = raw
    esp32.reset_input_buffer()  # Discard stale complete packets before the next read.
    if any(value != 0xFFFF and value >= ENCODER_COUNTS for value in raw):
        raise ValueError(f"Invalid ESP32 angle packet {raw} ({packet.hex()})")
    missing = [str(index + 1) for index in (0, 1) if raw[index] == 0xFFFF]
    if missing:
        raise RuntimeError(
            f"Pendulum angle channel(s) {', '.join(missing)} missing "
            f"(ESP32 packet {raw}, bytes {packet.hex()})"
        )
    return raw


def signed_count_delta(raw, reference):
    return (raw - reference + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def circular_mean_counts(readings):
    phase = np.asarray(readings, dtype=float) * (2 * np.pi / ENCODER_COUNTS)
    mean_phase = np.angle(np.mean(np.exp(1j * phase))) % (2 * np.pi)
    return mean_phase * ENCODER_COUNTS / (2 * np.pi)


def calibrate_hanging(esp32):
    """Return arm 1 upright and arm 2 aligned references in encoder counts."""
    log("CAL", f"Reading {HANGING_SAMPLES} packets with both arms hanging")
    readings = []
    for index in range(HANGING_SAMPLES):
        readings.append(read_angles(esp32)[:2])
        if index < HANGING_SAMPLES - 1:
            time.sleep(SAMPLE_SPACING_S)
    readings = np.asarray(readings, dtype=float)
    down1 = circular_mean_counts(readings[:, 0])
    aligned2 = circular_mean_counts(readings[:, 1])
    for channel, reference in ((0, down1), (1, aligned2)):
        deviations = signed_count_delta(readings[:, channel], reference)
        largest = float(np.max(np.abs(deviations)))
        log(
            "CAL",
            f"angle{channel + 1} first={int(readings[0, channel])} "
            f"last={int(readings[-1, channel])} "
            f"distinct={len(np.unique(readings[:, channel]))} "
            f"largest_deviation={largest:.1f} counts",
        )
        if largest > MAX_HANGING_DEVIATION_COUNTS:
            raise RuntimeError(
                f"Pendulum {channel + 1} moved during hanging calibration "
                f"({largest:.1f} counts); let both arms settle and restart"
            )
    upright1 = (down1 + HALF_TURN) % ENCODER_COUNTS
    log(
        "CAL",
        f"angle1 hanging={down1:.1f}, estimated upright={upright1:.1f}; "
        f"angle2 aligned reference={aligned2:.1f} counts",
    )
    return upright1, aligned2


def model_angles(raw1, raw2, upright1, aligned2):
    """Return both absolute angles and the second joint angle, in radians.

    The original controller negates both encoder directions. Angle 2's encoder
    is relative to arm 1, while the model's angle 2 is relative to vertical.
    """
    scale = -(2 * np.pi / ENCODER_COUNTS)
    theta1 = signed_count_delta(raw1, upright1) * scale
    joint2 = signed_count_delta(raw2, aligned2) * scale
    return theta1, theta1 + joint2, joint2


def check_upright(esp32, upright1, aligned2):
    samples = []
    raw_first = None
    raw_last = None
    for index in range(UPRIGHT_SAMPLES):
        raw = read_angles(esp32)
        raw_first = raw if raw_first is None else raw_first
        raw_last = raw
        samples.append(model_angles(raw[0], raw[1], upright1, aligned2))
        if index < UPRIGHT_SAMPLES - 1:
            time.sleep(SAMPLE_SPACING_S)
    theta1, theta2, joint2 = np.median(np.asarray(samples), axis=0)
    log(
        "CHECK",
        f"raw first={raw_first[:2]} last={raw_last[:2]}; "
        f"arm1={np.degrees(theta1):+.2f}deg "
        f"arm2={np.degrees(theta2):+.2f}deg "
        f"joint2={np.degrees(joint2):+.2f}deg",
    )
    limit = np.deg2rad(MAX_START_ANGLE_DEG)
    if max(abs(theta1), abs(theta2), abs(joint2)) > limit:
        raise RuntimeError(
            "Upright check failed: hold both arms upright and restart. "
            "No motor command was sent."
        )
    return float(theta1), float(theta2)


def load_control_data(path):
    content = Path(path).read_text()
    sections = {
        "A": (r"--- Matrix A \(Linearized\) ---\s*(.*?)(?=---|\Z)", (6, 6)),
        "B": (r"--- Matrix B ---\s*(.*?)(?=---|\Z)", (6, 1)),
        "C": (r"--- Matrix C ---\s*(.*?)(?=---|\Z)", (3, 6)),
        "Kd": (r"--- LQR Gain \(Kd\) ---\s*(.*?)(?=---|\Z)", (1, 6)),
        "Ld": (r"--- Kalman Gain \(Ld\) ---\s*(.*?)(?=---|\Z)", (6, 3)),
    }
    matrices = {}
    for name, (pattern, shape) in sections.items():
        match = re.search(pattern, content, re.DOTALL)
        if match is None:
            raise ValueError(f"Missing {name} in {path}")
        values = np.fromstring(match.group(1).replace("[", " ").replace("]", " "), sep=" ")
        if values.size != np.prod(shape) or not np.isfinite(values).all():
            raise ValueError(f"Invalid {name} matrix in {path}")
        matrices[name] = values.reshape(shape)
    return matrices


def read_position(arduino):
    steps = struct.unpack(">h", read_exact(arduino, 2))[0]
    arduino.reset_input_buffer()
    return steps, steps * DISTANCE_PER_6400_STEPS_M / 6400


def send_motor_command(arduino, timer_command):
    payload = struct.pack("<i", int(timer_command))
    if arduino.write(payload) != len(payload):
        raise OSError(f"Arduino accepted fewer than {len(payload)} command bytes")


def timer_command_for_velocity(cart_velocity):
    step_hz = -(cart_velocity * 6400) / DISTANCE_PER_6400_STEPS_M
    if abs(step_hz) > 2:
        timer_command = int((0.5 / step_hz) * 250000)
    else:
        timer_command = 65535 if step_hz >= 0 else -65535
    return step_hz, timer_command


def main():
    global balancing_output_active
    matrices_path = Path(__file__).with_name("control_matrices2.txt")
    matrices = load_control_data(matrices_path)
    log("START", f"Loaded control matrices from {matrices_path}")

    esp32 = None
    arduino = None
    motor_command_sent = False
    stage = "opening serial ports"
    cycle = 0
    run_logger = None
    run_started_at = None
    run_stop_reason = "program exit"
    steps = None
    last_command = None
    try:
        esp32 = serial.Serial("/dev/ttyUSB0", 921600, timeout=0.003)
        arduino = serial.Serial(
            "/dev/ttyACM0", 115200, timeout=0.003, write_timeout=0.2
        )
        log("START", "ESP32 and Arduino serial ports open; waiting 3s for Arduino")
        time.sleep(3)
        arduino.reset_input_buffer()
        esp32.reset_input_buffer()

        stage = "hanging calibration"
        log("READY", "Motor power OFF; center the cart and let BOTH arms hang still")
        input("Press Enter when both arms have stopped swinging: ")
        upright1, aligned2 = calibrate_hanging(esp32)

        stage = "upright check"
        if SAVE_RUN_CSV:
            filename = datetime.now().astimezone().strftime("balance_double_%Y%m%d_%H%M%S_%f.csv")
            new_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=DOUBLE_COLUMNS)
            new_logger.start()
            run_logger = new_logger
            run_logger.event("config", json.dumps({
                "schema_version": 1, "pendulum_count": 2,
                "sample_period_s": SAMPLE_PERIOD_S,
                "distance_per_6400_steps_m": DISTANCE_PER_6400_STEPS_M,
                "max_running_angle_rad": MAX_RUNNING_ANGLE_RAD,
                "reference_upright_1_counts": float(upright1),
                "reference_aligned_2_counts": float(aligned2),
                "discrete_A": matrices["A"].tolist(),
                "discrete_B": matrices["B"].tolist(),
                "discrete_C": matrices["C"].tolist(),
                "controller_gain": matrices["Kd"].tolist(),
                "estimator_gain": matrices["Ld"].tolist(),
            }, separators=(",", ":")), 0.0)
            log("LOG", f"Recording every control cycle to {run_logger.path}")
        log("READY", "Turn motor power ON and hold BOTH arms upright")
        input("Press Enter when both arms are upright: ")
        balancing_output_active = True
        run_started_at = time.monotonic()
        if run_logger is not None:
            run_logger.event("start", "Double pendulum balancing requested", 0.0)
        theta1, theta2 = check_upright(esp32, upright1, aligned2)
        if run_logger is not None:
            run_logger.event("check", f"arm1_deg={np.degrees(theta1):+.3f}; "
                             f"arm2_deg={np.degrees(theta2):+.3f}; "
                             f"joint2_deg={np.degrees(theta2 - theta1):+.3f}",
                             time.monotonic() - run_started_at)

        stage = "initial Arduino command"
        motor_command_sent = True
        send_motor_command(arduino, -65535)
        last_command = -65535
        steps, cart_m = read_position(arduino)
        log("START", f"Initial cart position={cart_m:+.4f}m ({steps} steps)")
        if run_logger is not None:
            run_logger.event("motor_start", f"initial_command={last_command}; "
                             f"initial_cart_steps={steps}; initial_cart_m={cart_m}",
                             time.monotonic() - run_started_at)

        state = np.array([[theta1], [0.0], [theta2], [0.0], [cart_m], [0.0]])
        force = np.array([[0.0]])
        next_status = time.monotonic()
        last_status_at = next_status
        last_status_cycle = 0
        previous_cycle_started = None
        log("RUN", "Double pendulum control loop started")

        while True:
            stage = "control loop"
            cycle += 1
            cycle_started = time.monotonic()
            cycle_period_ms = ("" if previous_cycle_started is None else
                               (cycle_started - previous_cycle_started) * 1000)
            previous_cycle_started = cycle_started
            previous_force_n = float(force[0, 0])
            predicted = matrices["A"] @ state + matrices["B"] @ force
            sensor_read_started = time.monotonic()
            raw1, raw2, raw3 = read_angles(esp32)
            sensor_read_ms = (time.monotonic() - sensor_read_started) * 1000
            reference_used = upright1
            theta1, theta2, joint2 = model_angles(raw1, raw2, reference_used, aligned2)
            if abs(theta1) > MAX_RUNNING_ANGLE_RAD or abs(theta2) > MAX_RUNNING_ANGLE_RAD:
                raise RuntimeError(
                    f"Angle limit exceeded: arm1={np.degrees(theta1):+.1f}deg, "
                    f"arm2={np.degrees(theta2):+.1f}deg"
                )

            cart_used_m = cart_m
            measured = np.array([[theta1], [theta2], [cart_m]])
            state = predicted + matrices["Ld"] @ (measured - matrices["C"] @ predicted)
            force = np.clip(-matrices["Kd"] @ state * 1.5, -5, 5)
            force_command_n = float(force[0, 0])

            # Preserve the existing double controller's motor mapping: its
            # estimated cart velocity, not the force value, sets step frequency.
            cart_velocity = float(state[5, 0])
            if cycle < 20:
                cart_velocity *= cycle / 20.0
            ramp_fraction = min(cycle / 20.0, 1.0)
            cart_velocity = float(np.clip(cart_velocity, -5, 5))
            step_hz, timer_command = timer_command_for_velocity(cart_velocity)

            stage = "sending motor command"
            send_motor_command(arduino, timer_command)
            last_command = timer_command
            stage = "reading cart position"
            position_read_started = time.monotonic()
            steps, cart_m = read_position(arduino)
            position_read_ms = (time.monotonic() - position_read_started) * 1000

            # Keep the existing controller's slow centering adjustment.
            upright1 += cart_m / 60 + cart_velocity / 30

            now = time.monotonic()
            loop_ms = (now - cycle_started) * 1000
            arm1_deg = float(np.degrees(theta1))
            arm2_deg = float(np.degrees(theta2))
            joint2_deg = float(np.degrees(joint2))
            if run_logger is not None:
                run_logger.sample((
                    "sample", cycle_started - run_started_at, cycle,
                    esp32_packet_count, raw1, raw2, raw3,
                    reference_used, aligned2,
                    theta1, arm1_deg, theta2, arm2_deg,
                    joint2, joint2_deg, cart_used_m,
                    steps, cart_m,
                    *predicted.flat, *state.flat,
                    previous_force_n, force_command_n,
                    cart_velocity, ramp_fraction, step_hz, timer_command,
                    upright1, sensor_read_ms, position_read_ms,
                    loop_ms, cycle_period_ms, "",
                ))
            if SHOW_BALANCING_LOGS and now >= next_status:
                rate = (cycle - last_status_cycle) / max(now - last_status_at, 1e-6)
                log(
                    "RUN",
                    f"cycle={cycle} raw=({raw1},{raw2}) "
                    f"arm1={arm1_deg:+.2f}deg "
                    f"arm2={arm2_deg:+.2f}deg "
                    f"cart={cart_m:+.4f}m steps={steps} "
                    f"speed_est={cart_velocity:+.3f}m/s "
                    f"force_calc={force_command_n:+.3f} "
                    f"step_hz={step_hz:+.1f} timer={last_command} rate={rate:.1f}Hz",
                )
                next_status = now + STATUS_INTERVAL_S
                last_status_at = now
                last_status_cycle = cycle

    except KeyboardInterrupt:
        run_stop_reason = "Ctrl+C"
        log("STOP", f"Ctrl+C at {stage} after {cycle} cycles")
    except Exception as exc:
        run_stop_reason = f"{type(exc).__name__}: {exc}"
        if run_logger is not None:
            run_logger.event("fault", f"stage={stage}; cycle={cycle}; "
                             f"raw_angles={latest_raw_angles}; cart_steps={steps}; "
                             f"motor_command={last_command}; {run_stop_reason}",
                             "" if run_started_at is None else time.monotonic() - run_started_at)
        log("FAULT", f"stage={stage}, cycle={cycle}: {type(exc).__name__}: {exc}")
        traceback.print_exc()
        raise
    finally:
        try:
            if arduino is not None and arduino.is_open:
                if motor_command_sent:
                    try:
                        send_motor_command(arduino, -1)  # 0xFFFFFFFF stop command
                        arduino.flush()
                        log("STOP", "Sent Arduino stop command")
                    except (serial.SerialException, OSError) as exc:
                        log("FAULT", f"Could not send stop command: {exc}")
                arduino.close()
            if esp32 is not None and esp32.is_open:
                esp32.close()
            log("STOP", "Serial ports closed")
        finally:
            if run_logger is not None:
                warning = run_logger.stop(run_stop_reason)
                if warning:
                    log("FAULT", warning)
                else:
                    log("STOP", f"CSV saved: {run_logger.path}")


if __name__ == "__main__":
    main()
