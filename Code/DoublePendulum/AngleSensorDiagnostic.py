"""Inspect framed ESP32 angles and dropout reasons with motor power OFF."""

import time
import serial
from angle_protocol import AnglePacketReader, AngleProtocolError, VALID, UNKNOWN_AGE

PORT = '/dev/ttyUSB0'
BAUD = 921600
ENCODER_COUNTS = 16384
PRINT_INTERVAL = 0.2


def format_channel(packet, index, baseline):
    value = packet.angles[index]
    detail = packet.channel_message(index)
    if packet.statuses[index] != VALID:
        return f'{detail} MISSING'
    change = 0 if baseline is None else (value - baseline + 8192) % 16384 - 8192
    return f'{detail} raw={value} change={change * 360 / ENCODER_COUNTS:+.2f}deg'


def main():
    print('Motor power OFF. This reads only the ESP32; it does not open the Arduino.', flush=True)
    print('Requires receiver protocol v1. Move one pendulum at a time.', flush=True)
    reader = AnglePacketReader()
    baseline = [None] * 3
    missing_count = [0] * 3
    started = time.monotonic()
    next_print = started
    last_timeout_print = started - 1
    packet_count = 0
    streak = 0
    episodes = 0
    longest_streak = 0
    with serial.Serial(PORT, BAUD, timeout=0.01) as esp32:
        reader.discard_pending(esp32)
        while True:
            try:
                packet = reader.read_latest(esp32, timeout_s=0.25)
            except AngleProtocolError as exc:
                now = time.monotonic()
                if now - last_timeout_print >= 1:
                    print(f'[{now-started:6.1f}s] SERIAL: {exc}', flush=True)
                    last_timeout_print = now
                continue
            now = time.monotonic()
            packet_count += 1
            for i in range(3):
                if packet.statuses[i] != VALID:
                    missing_count[i] += 1
                elif baseline[i] is None:
                    baseline[i] = packet.angles[i]
            if packet.statuses[0] != VALID:
                streak += 1
                longest_streak = max(longest_streak, streak)
                if streak == 1:
                    episodes += 1
                    print(f'[{now-started:6.1f}s] ANGLE 1 DROPOUT STARTED '
                          f'sequence={packet.sequence} {packet.channel_message(0)}', flush=True)
            elif streak:
                print(f'[{now-started:6.1f}s] ANGLE 1 RECOVERED after {streak} observed missing frame(s)', flush=True)
                streak = 0
            if now >= next_print:
                fields = [format_channel(packet, i, baseline[i]) for i in range(3)]
                print(f'[{now-started:6.1f}s] sequence={packet.sequence} ' + ' | '.join(fields) +
                      f' | angle1 missing={missing_count[0]}/{packet_count} episodes={episodes} '
                      f'longest={longest_streak} crc_errors={reader.crc_errors} '
                      f'sequence_gaps={reader.sequence_gaps} superseded={reader.superseded_frames} '
                      f'discarded_bytes={reader.discarded_bytes} receiver_restarts={reader.receiver_restarts}', flush=True)
                next_print = now + PRINT_INTERVAL


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
