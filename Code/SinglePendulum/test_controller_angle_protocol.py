"""Exercise actual single-controller reader and cleanup functions without hardware."""

# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

CODE_DIRECTORY = Path(__file__).resolve().parents[1]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

import ast
from datetime import datetime
import struct
import time
import unittest
from unittest.mock import Mock

from Global.angle_protocol import AnglePacketReader, packet_csv_values, TELEMETRY_COLUMNS
from Global.angle_packet_fallback import MissingAnglePacketError
from test_angle_protocol import Port, frame


class ControllerAngleProtocolTests(unittest.TestCase):
    def environment(self, data):
        env = dict(time=time, datetime=datetime, struct=struct,
                   angle_reader=AnglePacketReader(), esp32=Port(data),
                   latest_raw_angle=None, latest_raw_angles=None,
                   latest_angle_packet=None, latest_angle_message='',
                   last_good_angle_read_at=None, esp32_packet_count=0,
                   current_stage='test', cycle_count=1, latest_position_steps=0,
                   last_motor_command=0, MissingAnglePacketError=MissingAnglePacketError,
                   log=Mock())
        from Global.angle_protocol import AngleProtocolError
        env['AngleProtocolError'] = AngleProtocolError
        tree = ast.parse(Path(__file__).with_name('PythonMaster_auto.py').read_text())
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in ('read_raw_angle1', 'stop_and_close')]
        exec(compile(ast.Module(body=functions, type_ignores=[]), 'single-reader', 'exec'), env)
        return env

    def test_valid_sensor1_with_two_inactive_channels(self):
        env = self.environment(frame())
        self.assertEqual(env['read_raw_angle1'](), 9054)
        self.assertEqual(env['latest_angle_message'], '')
        self.assertEqual(env['latest_angle_packet'].statuses, (0, 3, 3))
        self.assertEqual(env['esp32_packet_count'], 1)

    def test_terminal_reason_and_csv_age_match_receiver(self):
        for status, age, reason in [(1, 2, 'encoder_read_failed'),
                                    (2, 57, 'wireless_timeout'),
                                    (3, 0xFFFFFFFF, 'never_received')]:
            with self.subTest(reason=reason):
                env = self.environment(frame(channels=((65535, status, age),
                        (65535, 3, 0xFFFFFFFF), (65535, 3, 0xFFFFFFFF))))
                with self.assertRaisesRegex(MissingAnglePacketError, reason):
                    env['read_raw_angle1']()
                self.assertIn(reason, env['log'].call_args.args[1])
                row = dict(zip(TELEMETRY_COLUMNS, packet_csv_values(
                    env['latest_angle_packet'], env['angle_reader'])))
                self.assertEqual(row['angle_status_1'], reason)
                self.assertEqual(row['radio_age_1_ms'], '' if status == 3 else age)

    def test_corrupt_frame_recovers_then_timeout_clears_previous_metadata(self):
        corrupt = bytearray(frame()); corrupt[-1] ^= 1
        env = self.environment(bytes(corrupt) + frame(2, 1010))
        self.assertEqual(env['read_raw_angle1'](), 9054)
        self.assertEqual(env['angle_reader'].crc_errors, 1)
        with self.assertRaisesRegex(MissingAnglePacketError, 'no_valid_frame'):
            env['read_raw_angle1']()
        self.assertIsNone(env['latest_angle_packet'])
        self.assertEqual(env['latest_raw_angles'], (None, None, None))

    def test_cleanup_still_stops_motor_and_closes_both_ports(self):
        env = self.environment(b'')
        arduino, esp32, keys = Mock(), Mock(), Mock()
        env.update(arduino=arduino, esp32=esp32, offset_keys=keys,
                   motor_command_sent=True, run_logger=None)
        env['stop_and_close']()
        arduino.write.assert_called_once_with(b'\xff\xff\xff\xff')
        arduino.close.assert_called_once()
        esp32.close.assert_called_once()
        keys.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
