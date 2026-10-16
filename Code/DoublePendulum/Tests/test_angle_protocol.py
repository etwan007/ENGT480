"""Transport regressions, including a fixture emitted by the real C++ encoder."""
import binascii
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from angle_protocol import (AnglePacketReader, AngleProtocolError, decode_frame,
                            packet_csv_values, TELEMETRY_COLUMNS, FRAME_SIZE, MAGIC)


def frame(sequence=1, receiver_ms=1000, channels=None):
    if channels is None:
        channels = ((9054, 0, 2), (65535, 3, 0xFFFFFFFF), (65535, 3, 0xFFFFFFFF))
    body = bytes((1, 29)) + struct.pack('>II', sequence, receiver_ms)
    body += b''.join(struct.pack('>HBI', *channel) for channel in channels)
    return MAGIC + body + struct.pack('>H', binascii.crc_hqx(body, 0xFFFF))


class Port:
    def __init__(self, data=b'', chunk_limit=512):
        self.data = bytearray(data)
        self.chunk_limit = chunk_limit
    @property
    def in_waiting(self):
        return len(self.data)
    def read(self, count):
        count = min(count, self.chunk_limit)
        result = bytes(self.data[:count])
        del self.data[:count]
        return result
    def reset_input_buffer(self):
        raise AssertionError('reader must preserve partial frames')


class AngleProtocolTests(unittest.TestCase):
    def test_every_possible_split_and_one_byte_reads(self):
        wire = frame()
        for split in range(1, FRAME_SIZE):
            with self.subTest(split=split):
                reader = AnglePacketReader()
                port = Port(wire[:split])
                with self.assertRaises(AngleProtocolError):
                    reader.read_latest(port, timeout_s=0.001)
                port.data.extend(wire[split:])
                self.assertEqual(reader.read_latest(port).angles, (9054, 65535, 65535))
        self.assertEqual(AnglePacketReader().read_latest(Port(wire, 1)).sequence, 1)

    def test_corruption_deletion_insertion_and_bad_header_resynchronize(self):
        original = frame()
        mutations = [b'boot chatter\r\n' + original,
                     original[:9] + b'\x99' + original[9:],
                     original[:9] + original[10:],
                     original[:20] + bytes((original[20] ^ 1,)) + original[21:],
                     MAGIC + b'\x02\x1d' + original[6:],
                     MAGIC + b'\x01\xff' + original[6:]]
        for damaged in mutations:
            with self.subTest(data=damaged.hex()):
                reader = AnglePacketReader()
                packet = reader.read_latest(Port(damaged + frame(2, 1010)))
                self.assertEqual(packet.sequence, 2)
                self.assertEqual(packet.angles[0], 9054)
                self.assertGreater(reader.discarded_bytes, 0)

    def test_crc_error_and_sequence_gap_are_recorded(self):
        reader = AnglePacketReader()
        reader.read_latest(Port(frame(1)))
        bad = bytearray(frame(2, 1010)); bad[14] ^= 1
        packet = reader.read_latest(Port(bytes(bad) + frame(3, 1020)))
        self.assertEqual(packet.sequence, 3)
        self.assertEqual(reader.crc_errors, 1)
        self.assertEqual(reader.sequence_gaps, 1)

    def test_newest_complete_frame_and_trailing_fragment(self):
        reader = AnglePacketReader()
        third = frame(3, 1020)
        port = Port(frame(1) + frame(2, 1010) + third[:17])
        self.assertEqual(reader.read_latest(port).sequence, 2)
        self.assertEqual(reader.superseded_frames, 1)
        port.data.extend(third[17:])
        self.assertEqual(reader.read_latest(port).sequence, 3)

    def test_all_missing_reasons_and_csv_mapping(self):
        packet = decode_frame(frame(channels=((65535, 1, 2), (65535, 2, 57), (65535, 3, 0xFFFFFFFF))))
        self.assertIn('encoder_read_failed radio_age_ms=2', packet.channel_message(0))
        self.assertIn('wireless_timeout radio_age_ms=57', packet.channel_message(1))
        self.assertIn('never_received radio_age_ms=unknown', packet.channel_message(2))
        values = dict(zip(TELEMETRY_COLUMNS, packet_csv_values(packet, AnglePacketReader())))
        self.assertEqual(values['angle_status_1'], 'encoder_read_failed')
        self.assertEqual(values['radio_age_2_ms'], 57)
        self.assertEqual(values['radio_age_3_ms'], '')
        self.assertEqual(len(packet_csv_values(None, AnglePacketReader())), len(TELEMETRY_COLUMNS))

    def test_invalid_crc_consistent_and_inconsistent_fields(self):
        for channel in [(40000, 0, 1), (65535, 0, 1), (1234, 1, 1),
                        (65535, 7, 1), (1234, 0, 51), (65535, 1, 51),
                        (65535, 2, 50), (65535, 3, 0)]:
            with self.subTest(channel=channel), self.assertRaises(ValueError):
                decode_frame(frame(channels=(channel, (65535, 3, 0xFFFFFFFF), (65535, 3, 0xFFFFFFFF))))
        corrupt = bytearray(frame()); corrupt[-1] ^= 1
        with self.assertRaisesRegex(ValueError, 'CRC'):
            decode_frame(bytes(corrupt))

    def test_duplicate_out_of_order_and_sequence_wrap(self):
        reader = AnglePacketReader()
        reader.read_latest(Port(frame(0xFFFFFFFF, 1000)))
        self.assertEqual(reader.read_latest(Port(frame(0, 1010))).sequence, 0)
        self.assertEqual(reader.sequence_gaps, 0)
        reader.read_latest(Port(frame(0, 1010) + frame(0xFFFFFFFF, 1010) + frame(1, 1020)))
        self.assertEqual(reader.duplicate_frames, 2)
        self.assertEqual(reader.sequence_gaps, 0)

    def test_uptime_wrap_and_receiver_reboot(self):
        reader = AnglePacketReader()
        reader.read_latest(Port(frame(10, 0xFFFFFFFA)))
        reader.read_latest(Port(frame(11, 4)))
        self.assertEqual(reader.receiver_restarts, 0)
        reader.read_latest(Port(frame(12, 10000)))
        packet = reader.read_latest(Port(frame(0, 200)))
        self.assertEqual(packet.sequence, 0)
        self.assertEqual(reader.receiver_restarts, 1)

    def test_backlog_is_bounded_and_not_returned_as_current_angle(self):
        reader = AnglePacketReader()
        data = b''.join(frame(i, i * 10) for i in range(200))
        port = Port(data)
        with self.assertRaisesRegex(AngleProtocolError, 'serial_backlog'):
            reader.read_latest(port)
        self.assertLessEqual(len(reader.buffer), FRAME_SIZE - 1)
        self.assertEqual(reader.read_latest(port).sequence, 199)

    def test_garbage_memory_bound_and_legacy_protocol_rejection(self):
        reader = AnglePacketReader()
        with self.assertRaises(AngleProtocolError):
            reader.read_latest(Port(b'\xff' * 10000))
        self.assertLessEqual(len(reader.buffer), 3)
        with self.assertRaisesRegex(AngleProtocolError, 'protocol v1'):
            AnglePacketReader().read_latest(Port(struct.pack('>HHH', 9054, 65535, 65535)), timeout_s=0.001)

    def test_prompt_drain_preserves_partial_next_frame(self):
        reader = AnglePacketReader()
        second = frame(2, 1010)
        port = Port(frame() + second[:8])
        reader.discard_pending(port)
        port.data.extend(second[8:])
        self.assertEqual(reader.read_latest(port).sequence, 2)

    def test_real_cpp_encoder_and_receiver_timing_regression(self):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if compiler is None:
            self.skipTest('C++ compiler unavailable for receiver host test')
        tests = Path(__file__).resolve().parents[2] / 'Wireless' / 'tests'
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'receiver_test'
            subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(tests / 'mocks'), str(tests / 'receiver_protocol_test.cpp'),
                            '-o', str(binary)], check=True, capture_output=True, text=True)
            emitted = subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout.strip()
        packet = decode_frame(bytes.fromhex(emitted))
        self.assertEqual(packet.sequence, 0x01020304)
        self.assertEqual(packet.angles, (9054, 65535, 65535))
        self.assertEqual(packet.statuses, (0, 1, 3))
        self.assertEqual(packet.ages_ms, (50, 1, 0xFFFFFFFF))


if __name__ == '__main__':
    unittest.main()
