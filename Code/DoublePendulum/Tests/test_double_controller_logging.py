"""Short hardware-free exercise of the double controller's CSV path."""

# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

CODE_DIRECTORY = Path(__file__).resolve().parents[2]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))


import csv
import io
import json
import struct
import tempfile
import types
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest.mock import patch

import numpy as np

try:
    import serial
except ModuleNotFoundError:
    serial = types.ModuleType("serial")
    serial.SerialException = OSError
    sys.modules["serial"] = serial

import PythonMaster2_auto as controller
from Global.angle_packet_fallback import MissingAnglePacketError
from Global.run_csv_logger import DOUBLE_COLUMNS
from Global.angle_protocol import AnglePacket, AnglePacketReader
from Tests.test_angle_protocol import Port, frame


class FakePort:
    def __init__(self):
        self.is_open = True
        self.in_waiting = 0

    def reset_input_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.is_open = False


class DoubleControllerLoggingTests(unittest.TestCase):
    def test_framed_reader_preserves_required_channel_reasons(self):
        controller.angle_reader = AnglePacketReader()
        packet = frame(channels=((1000, 0, 1), (65535, 2, 57), (65535, 3, 0xFFFFFFFF)))
        self.assertEqual(controller.read_angles(Port(packet)), (1000, 65535, 65535))
        self.assertIn('angle2 reason=wireless_timeout radio_age_ms=57', controller.latest_angle_message)
        self.assertNotIn('angle3', controller.latest_angle_message)
        controller.angle_reader = AnglePacketReader()
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(MissingAnglePacketError, 'wireless_timeout'):
            controller.read_required_angles(Port(packet))

    def test_both_setup_stages_use_configurable_miss_limits(self):
        for stage in ("hanging", "upright"):
            with (
                self.subTest(stage=stage),
                patch.object(controller, "MAX_SETUP_CONSECUTIVE_MISSES", 2),
                patch.object(controller, "read_required_angles", side_effect=[
                    MissingAnglePacketError("missing"), MissingAnglePacketError("missing"),
                    *[(1000, 2000, 0xFFFF)] * 50]),
                patch.object(controller.time, "sleep"),
                redirect_stdout(io.StringIO()),
            ):
                if stage == "hanging":
                    np.testing.assert_allclose(controller.calibrate_hanging(None), [9192, 2000])
                else:
                    np.testing.assert_allclose(controller.check_upright(None, 1000, 2000), [0, 0])

    def test_manual_offsets_sign_wraparound_and_prediction(self):
        with (
            patch.object(controller, "ARM1_OFFSET_DEG", 0.3),
            patch.object(controller, "JOINT2_OFFSET_DEG", -0.1),
        ):
            upright1, aligned2 = controller.apply_manual_offsets(0, 16383)
        self.assertTrue(0 <= upright1 < 16384)
        self.assertTrue(0 <= aligned2 < 16384)
        angles = controller.model_angles(0, 16383, upright1, aligned2)
        np.testing.assert_allclose(np.degrees(angles), [-0.3, -0.2, 0.1], atol=1e-10)
        predicted = np.array([[-0.02], [0], [0.04], [0], [0], [0]])
        theta1, theta2, _, missing = controller.model_angles_with_prediction(
            (None, None, None), predicted, upright1, aligned2)
        self.assertAlmostEqual(theta1, -0.02)
        self.assertAlmostEqual(theta2, 0.04)
        self.assertEqual(missing, [1, 2])

    def test_malformed_packet_is_an_unusable_reading(self):
        class PacketPort(FakePort):
            port = "/dev/test-esp32"

            def __init__(self, packet):
                super().__init__()
                self.packet = packet

            def read(self, count):
                data, self.packet = self.packet[:count], self.packet[count:]
                return data

        port = PacketPort(struct.pack(">HHH", 65507, 0xFFFF, 0xFFFF))
        with redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(MissingAnglePacketError, "expected receiver protocol v1"):
                controller.read_angles(port)

    def test_missing_channels_use_predictions_individually(self):
        predicted = np.array([[0.12], [0], [0.17], [0], [0], [0]])
        theta1, theta2, joint2, missing = controller.model_angles_with_prediction(
            (0xFFFF, 2000, 0xFFFF), predicted, 1000, 2000)
        self.assertAlmostEqual(theta1, 0.12)
        self.assertAlmostEqual(theta2, 0.12)
        self.assertAlmostEqual(joint2, 0)
        self.assertEqual(missing, [1])

        theta1, theta2, joint2, missing = controller.model_angles_with_prediction(
            (1000, 0xFFFF, 0xFFFF), predicted, 1000, 2000)
        self.assertAlmostEqual(theta1, 0)
        self.assertAlmostEqual(joint2, 0.05)
        self.assertAlmostEqual(theta2, 0.05)
        self.assertEqual(missing, [2])

        theta1, theta2, joint2, missing = controller.model_angles_with_prediction(
            (None, None, None), predicted, 1000, 2000)
        self.assertAlmostEqual(theta1, 0.12)
        self.assertAlmostEqual(theta2, 0.17)
        self.assertAlmostEqual(joint2, 0.05)
        self.assertEqual(missing, [1, 2])

    def test_two_consecutive_incomplete_packets_stop_motor(self):
        for allowed in (0, 1, 3):
            with self.subTest(allowed=allowed):
                self.exercise_packet_stop_limit(allowed)

    def exercise_packet_stop_limit(self, allowed):
        matrices = {
            "A": np.eye(6), "B": np.zeros((6, 1)),
            "C": np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]]),
            "Kd": np.zeros((1, 6)), "Ld": np.zeros((6, 3)),
        }
        esp32, arduino = FakePort(), FakePort()
        commands = []
        packets = iter([(1000, 2000, 0xFFFF)] + [
            (0xFFFF, 2000, 0xFFFF) if index % 2 == 0 else (1000, 0xFFFF, 0xFFFF)
            for index in range(allowed + 1)])
        controller.balancing_output_active = False
        controller.esp32_packet_count = 0
        controller.latest_raw_angles = None
        with (
            patch.object(controller, "MAX_RUNNING_CONSECUTIVE_MISSES", allowed),
            patch.object(controller, "SAVE_RUN_CSV", False),
            patch.object(controller, "SHOW_BALANCING_LOGS", False),
            patch.object(controller, "load_control_data", return_value=matrices),
            patch.object(controller.serial, "Serial", side_effect=[esp32, arduino], create=True),
            patch.object(controller, "calibrate_hanging", return_value=(1000.0, 2000.0)),
            patch.object(controller, "check_upright", return_value=(0.0, 0.0)),
            patch.object(controller, "read_angles", side_effect=lambda _port: next(packets)),
            patch.object(controller, "read_position", return_value=(0, 0.0)),
            patch.object(controller, "send_motor_command", side_effect=lambda _port, value: commands.append(value)),
            patch.object(controller.time, "sleep"),
            patch("builtins.input", return_value=""),
            redirect_stdout(io.StringIO()),
            redirect_stderr(io.StringIO()),
        ):
            with self.assertRaisesRegex(RuntimeError, f"{allowed + 1} consecutive ESP32 angle packets"):
                controller.main()
        self.assertEqual(commands[0], -65535)
        self.assertEqual(commands[-1], -1)
        self.assertEqual(len(commands), allowed + 3)  # Initial, valid, allowed fallbacks, stop.
        self.assertFalse(esp32.is_open)
        self.assertFalse(arduino.is_open)

    def test_valid_packet_resets_fallback_streak(self):
        matrices = {
            "A": np.eye(6), "B": np.zeros((6, 1)),
            "C": np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]]),
            "Kd": np.zeros((1, 6)), "Ld": np.zeros((6, 3)),
        }
        esp32, arduino = FakePort(), FakePort()
        commands = []
        packets = iter([(0xFFFF, 2000, 0xFFFF),
                        (1000, 2000, 0xFFFF),
                        (1000, 0xFFFF, 0xFFFF), None])

        def read_angles(_port):
            packet = next(packets)
            if packet is None:
                raise KeyboardInterrupt
            return packet

        controller.balancing_output_active = False
        with (
            patch.object(controller, "SAVE_RUN_CSV", False),
            patch.object(controller, "SHOW_BALANCING_LOGS", False),
            patch.object(controller, "load_control_data", return_value=matrices),
            patch.object(controller.serial, "Serial", side_effect=[esp32, arduino], create=True),
            patch.object(controller, "calibrate_hanging", return_value=(1000.0, 2000.0)),
            patch.object(controller, "check_upright", return_value=(0.0, 0.0)),
            patch.object(controller, "read_angles", side_effect=read_angles),
            patch.object(controller, "read_position", return_value=(0, 0.0)),
            patch.object(controller, "send_motor_command", side_effect=lambda _port, value: commands.append(value)),
            patch.object(controller.time, "sleep"),
            patch("builtins.input", return_value=""),
            redirect_stdout(io.StringIO()),
        ):
            controller.main()
        self.assertEqual(len(commands), 5)  # Initial, three cycles, stop.
        self.assertEqual(commands[-1], -1)
        self.assertFalse(esp32.is_open)
        self.assertFalse(arduino.is_open)

    def test_five_cycles_recorded_and_motor_stopped(self):
        matrices = {
            "A": np.eye(6), "B": np.zeros((6, 1)),
            "C": np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]]),
            "Kd": np.zeros((1, 6)), "Ld": np.zeros((6, 3)),
        }
        esp32, arduino = FakePort(), FakePort()
        commands = []
        reads = 0

        def read_angles(_port):
            nonlocal reads
            reads += 1
            if reads > 5:
                raise KeyboardInterrupt
            controller.esp32_packet_count += 1
            controller.latest_raw_angles = (1000, 2000, 3000)
            controller.latest_angle_packet = AnglePacket(reads, reads * 10,
                controller.latest_raw_angles, (0, 0, 0), (1, 2, 3))
            return controller.latest_raw_angles

        controller.balancing_output_active = False
        controller.esp32_packet_count = 0
        controller.latest_raw_angles = None
        with tempfile.TemporaryDirectory() as folder:
            with (
                patch.object(controller, "RUN_LOG_DIRECTORY", Path(folder)),
                patch.object(controller, "ARM1_OFFSET_DEG", 0.3),
                patch.object(controller, "JOINT2_OFFSET_DEG", -0.1),
                patch.object(controller, "SHOW_BALANCING_LOGS", False),
                patch.object(controller, "load_control_data", return_value=matrices),
                patch.object(controller.serial, "Serial", side_effect=[esp32, arduino], create=True),
                patch.object(controller, "calibrate_hanging", return_value=(1000.0, 2000.0)),
                patch.object(controller, "check_upright", return_value=(0.0, 0.0)),
                patch.object(controller, "read_angles", side_effect=read_angles),
                patch.object(controller, "read_position", return_value=(600, 0.06)),
                patch.object(controller, "send_motor_command", side_effect=lambda _port, value: commands.append(value)),
                patch.object(controller.time, "sleep"),
                patch("builtins.input", return_value=""),
                redirect_stdout(io.StringIO()),
            ):
                controller.main()
            files = list(Path(folder).glob("balance_double_*.csv"))
            self.assertEqual(len(files), 1)
            with files[0].open(newline="", encoding="utf-8") as file:
                reader = csv.DictReader(file)
                self.assertEqual(tuple(reader.fieldnames), DOUBLE_COLUMNS)
                rows = list(reader)
        samples = [row for row in rows if row["record_type"] == "sample"]
        self.assertEqual(len(samples), 5)
        self.assertEqual([int(row["cycle"]) for row in samples], [1, 2, 3, 4, 5])
        self.assertAlmostEqual(float(samples[0]["measured_angle_2_deg"]), -0.2)
        self.assertEqual(samples[0]["esp32_sequence"], "1")
        self.assertEqual(samples[0]["angle_status_2"], "valid")
        self.assertEqual(samples[0]["radio_age_2_ms"], "2")
        first_reference = 1000 - 0.3 * 16384 / 360
        for index, sample in enumerate(samples):
            self.assertAlmostEqual(float(sample["reference_upright_1_counts"]),
                                   first_reference + index * 0.001)
            self.assertAlmostEqual(float(sample["reference_upright_1_next_counts"]),
                                   first_reference + (index + 1) * 0.001)
        config = json.loads(next(row["message"] for row in rows if row["record_type"] == "config"))
        self.assertEqual(config["manual_arm1_offset_deg"], 0.3)
        self.assertEqual(config["manual_joint2_offset_deg"], -0.1)
        self.assertEqual(config["max_running_consecutive_misses"], 1)
        self.assertEqual(rows[-1]["record_type"], "stop")
        self.assertIn("dropped_samples=0", rows[-1]["message"])
        self.assertEqual(commands[0], -65535)
        self.assertEqual(commands[-1], -1)
        self.assertFalse(esp32.is_open)
        self.assertFalse(arduino.is_open)


if __name__ == "__main__":
    unittest.main()
