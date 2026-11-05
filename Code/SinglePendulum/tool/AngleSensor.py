"""Show all three ESP32 angle channels without opening or commanding the Arduino.

Run with motor power off. Move one pendulum at a time and watch which channel
changes. Press Ctrl+C to stop.
"""

import struct
import time

import serial


PORT = "/dev/ttyUSB0"
BAUD = 921600
ENCODER_COUNTS = 16384
PRINT_INTERVAL = 0.2


def read_exact(port, count, timeout_seconds=0.25):
    deadline = time.monotonic() + timeout_seconds
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    return bytes(data)


def format_channel(value, baseline):
    if value == 0xFFFF:
        return "MISSING"
    if baseline is None:
        return f"{value:5d}"
    change = (value - baseline + 8192) % ENCODER_COUNTS - 8192
    return f"{value:5d} (change {change:+6d} counts, {change * 360 / ENCODER_COUNTS:+6.1f} deg)"


def main():
    print("Motor power OFF. This reads only the ESP32; it does not open the Arduino.", flush=True)
    print("Let the pendulum hang, then move pendulum 1 slowly. Watch which channel changes.", flush=True)
    baseline = [None, None, None]
    started = time.monotonic()
    next_print = started
    last_timeout_print = started - 1
    packet_count = 0
    missing_count = [0, 0, 0]
    angle1_missing_streak = 0
    angle1_dropout_count = 0
    longest_angle1_streak = 0

    with serial.Serial(PORT, BAUD, timeout=0.01) as esp32:
        esp32.reset_input_buffer()
        print(f"Opened {PORT} at {BAUD} baud. Press Ctrl+C to stop.", flush=True)
        while True:
            packet = read_exact(esp32, 6)
            now = time.monotonic()
            if len(packet) != 6:
                if now - last_timeout_print >= 1:
                    print(f"[{now-started:6.1f}s] NO COMPLETE ESP32 PACKET "
                          f"({len(packet)}/6 bytes)", flush=True)
                    last_timeout_print = now
                esp32.reset_input_buffer()
                continue

            angles = struct.unpack(">HHH", packet)
            if any(value != 0xFFFF and value >= ENCODER_COUNTS for value in angles):
                if now >= next_print:
                    print(f"[{now-started:6.1f}s] INVALID PACKET {angles}; retrying", flush=True)
                    next_print = now + PRINT_INTERVAL
                esp32.reset_input_buffer()
                continue

            packet_count += 1
            for index, value in enumerate(angles):
                if value == 0xFFFF:
                    missing_count[index] += 1

            if angles[0] == 0xFFFF:
                angle1_missing_streak += 1
                longest_angle1_streak = max(longest_angle1_streak, angle1_missing_streak)
                if angle1_missing_streak == 1:
                    angle1_dropout_count += 1
                    print(f"[{now-started:6.1f}s] ANGLE 1 DROPOUT STARTED "
                          f"(episode {angle1_dropout_count})", flush=True)
            elif angle1_missing_streak:
                print(f"[{now-started:6.1f}s] ANGLE 1 RECOVERED after "
                      f"{angle1_missing_streak} missing packet(s)", flush=True)
                angle1_missing_streak = 0

            for index, value in enumerate(angles):
                if baseline[index] is None and value != 0xFFFF:
                    baseline[index] = value

            if now >= next_print:
                fields = [f"angle{index+1}={format_channel(value, baseline[index])}"
                          for index, value in enumerate(angles)]
                print(f"[{now-started:6.1f}s] " + " | ".join(fields) +
                      f" | angle1 missing {missing_count[0]}/{packet_count} packets "
                      f"in {angle1_dropout_count} episode(s), longest "
                      f"{longest_angle1_streak} packets", flush=True)
                next_print = now + PRINT_INTERVAL


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
