"""Bounded streaming reader for receiver protocol v1 (see ANGLE_PROTOCOL.md)."""

from dataclasses import dataclass
import binascii
import struct
import time

MAGIC = b'\xa5\x5a\xc3\x3c'
VERSION = 1
PAYLOAD_SIZE = 29
FRAME_SIZE = 37
VALID, ENCODER_FAILED, WIRELESS_TIMEOUT, NEVER_RECEIVED = range(4)
STATUS_NAMES = ('valid', 'encoder_read_failed', 'wireless_timeout', 'never_received')
UNKNOWN_AGE = 0xFFFFFFFF
PRESENT_TIMEOUT_MS = 50
TELEMETRY_COLUMNS = (
    'esp32_sequence', 'receiver_uptime_ms',
    'angle_status_1', 'angle_status_2', 'angle_status_3',
    'radio_age_1_ms', 'radio_age_2_ms', 'radio_age_3_ms',
    'serial_crc_errors', 'serial_discarded_bytes', 'serial_sequence_gaps',
    'serial_superseded_frames',
)


class AngleProtocolError(TimeoutError):
    """No usable framed serial message arrived before the deadline."""


@dataclass(frozen=True)
class AnglePacket:
    sequence: int
    receiver_ms: int
    angles: tuple
    statuses: tuple
    ages_ms: tuple

    def channel_message(self, index):
        age = self.ages_ms[index]
        age_text = 'unknown' if age == UNKNOWN_AGE else str(age)
        return (f'angle{index + 1} reason={STATUS_NAMES[self.statuses[index]]} '
                f'radio_age_ms={age_text}')

    def missing_message(self, channels=(0, 1, 2)):
        return '; '.join(self.channel_message(i) for i in channels
                         if self.statuses[i] != VALID)


def decode_frame(frame):
    """Validate framing, CRC, and channel/status consistency before using angles."""
    if len(frame) != FRAME_SIZE or frame[:4] != MAGIC or frame[4:6] != bytes((VERSION, PAYLOAD_SIZE)):
        raise ValueError('unsupported frame header')
    if binascii.crc_hqx(frame[4:-2], 0xFFFF) != int.from_bytes(frame[-2:], 'big'):
        raise ValueError('CRC mismatch')
    sequence, receiver_ms = struct.unpack_from('>II', frame, 6)
    angles, statuses, ages = [], [], []
    for i in range(3):
        angle, status, age = struct.unpack_from('>HBI', frame, 14 + 7 * i)
        if status not in range(4):
            raise ValueError('unknown channel status')
        if status == VALID:
            valid = angle < 16384 and age <= PRESENT_TIMEOUT_MS
        elif status == ENCODER_FAILED:
            valid = angle == 0xFFFF and age <= PRESENT_TIMEOUT_MS
        elif status == WIRELESS_TIMEOUT:
            valid = angle == 0xFFFF and age > PRESENT_TIMEOUT_MS
        else:
            valid = angle == 0xFFFF and age == UNKNOWN_AGE
        if not valid:
            raise ValueError('inconsistent angle/status/age')
        angles.append(angle)
        statuses.append(status)
        ages.append(age)
    return AnglePacket(sequence, receiver_ms, tuple(angles), tuple(statuses), tuple(ages))


class AnglePacketReader:
    """Keep partial frames; scan at most 4096 serial bytes per control-cycle read.

    Read all currently queued complete frames and return the newest one. A
    trailing partial frame remains buffered. CRC failures discard one byte and
    search for the next header; no per-cycle serial-buffer clearing is used.
    Serial ports must have a finite read timeout (controllers use 3 ms).
    """
    MAX_READ_BYTES = 4096
    CHUNK_SIZE = 512

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.sequence_gaps = 0
        self.superseded_frames = 0
        self.duplicate_frames = 0
        self.receiver_restarts = 0
        self.last_packet = None

    def _discard(self, count):
        del self.buffer[:count]
        self.discarded_bytes += count

    def _parse(self):
        latest = None
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                # Retain a possible split header, bounded to three bytes.
                self._discard(max(0, len(self.buffer) - len(MAGIC) + 1))
                break
            if start:
                self._discard(start)
            if len(self.buffer) < 6:
                break
            if self.buffer[4:6] != bytes((VERSION, PAYLOAD_SIZE)):
                self._discard(1)
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            try:
                packet = decode_frame(frame)
            except ValueError as exc:
                if str(exc) == 'CRC mismatch':
                    self.crc_errors += 1
                self._discard(1)
                continue
            del self.buffer[:FRAME_SIZE]
            previous = self.last_packet
            if previous is not None:
                delta = (packet.sequence - previous.sequence) & 0xFFFFFFFF
                time_delta = (packet.receiver_ms - previous.receiver_ms) & 0xFFFFFFFF
                if time_delta >= 0x80000000:
                    # Receiver reboot: its uptime moved backwards, not through wrap.
                    self.receiver_restarts += 1
                elif delta == 0 or delta >= 0x80000000:
                    self.duplicate_frames += 1
                    continue
                elif delta > 1:
                    self.sequence_gaps += delta - 1
            self.last_packet = packet
            if latest is not None:
                self.superseded_frames += 1
            latest = packet
        return latest

    def read_latest(self, port, timeout_s=0.1):
        deadline = time.monotonic() + timeout_s
        total = 0
        latest = self._parse()
        while time.monotonic() < deadline:
            waiting = max(0, int(port.in_waiting))
            if latest is not None and waiting == 0:
                return latest
            if total >= self.MAX_READ_BYTES:
                raise AngleProtocolError('serial_backlog: more than 4096 bytes queued; no angle used')
            count = min(waiting or 1, self.CHUNK_SIZE, self.MAX_READ_BYTES - total)
            chunk = port.read(count)
            total += len(chunk)
            self.buffer.extend(chunk)
            parsed = self._parse()
            if parsed is not None:
                if latest is not None:
                    self.superseded_frames += 1
                latest = parsed
        # Never use a sample from a backlog that was not fully drained.
        if latest is not None and int(port.in_waiting) == 0:
            return latest
        raise AngleProtocolError(
            f'no_valid_frame: expected receiver protocol v1; crc_errors={self.crc_errors} '
            f'discarded_bytes={self.discarded_bytes}; check receiver firmware')

    def discard_pending(self, port, timeout_s=1.0):
        """At a user prompt boundary, drain old bytes without flushing fragments.

        Calibration then waits for a new complete frame. This is intentionally
        outside the motor control loop and never commands hardware.
        """
        deadline = time.monotonic() + timeout_s
        while int(port.in_waiting):
            if time.monotonic() >= deadline:
                raise AngleProtocolError('serial_backlog: could not drain input before calibration')
            chunk = port.read(min(int(port.in_waiting), self.CHUNK_SIZE))
            self.buffer.extend(chunk)
            self._parse()
        # Keep any partial frame; the next read completes it or resynchronizes.


def packet_csv_values(packet, reader):
    values = ('',) * 8 if packet is None else (
        packet.sequence, packet.receiver_ms,
        *(STATUS_NAMES[s] for s in packet.statuses),
        *('' if age == UNKNOWN_AGE else age for age in packet.ages_ms),
    )
    return (*values, reader.crc_errors, reader.discarded_bytes,
            reader.sequence_gaps, reader.superseded_frames)
