"""Exercise real framed serial input through double-controller startup and stop."""

import io
import runpy
import struct
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

import numpy as np

# Reuse the serial dependency stub when pyserial is absent on the test host.
from Tests.test_double_controller_logging import controller
from Tests.test_angle_protocol import frame


class StreamPort:
    """Deliver one scheduled frame per read, with optional byte fragmentation."""
    def __init__(self, frames=(), chunk_limit=512):
        self.frames = iter(frames)
        self.data = bytearray()
        self.chunk_limit = chunk_limit
        self.is_open = True
        self.port = "/dev/simulated"
        self.commands = []

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        if not self.data:
            item = next(self.frames, b"")
            if item is None:
                raise KeyboardInterrupt
            self.data.extend(item)
        count = min(count, self.chunk_limit)
        result = bytes(self.data[:count])
        del self.data[:count]
        return result

    def write(self, data):
        self.commands.append(struct.unpack("<i", data)[0])
        return len(data)

    def reset_input_buffer(self):
        self.data.clear()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def wire(sequence, first=1000, second=2000, status1=0, status2=0):
    def channel(raw, status):
        age = {0: 2, 1: 2, 2: 57, 3: 0xFFFFFFFF}[status]
        return (raw if status == 0 else 65535, status, age)
    return frame(sequence, sequence * 10, (
        channel(first, status1), channel(second, status2),
        (65535, 3, 0xFFFFFFFF)))


class DoubleProtocolIntegrationTests(unittest.TestCase):
    def exercise(self, frames, expected_error=None, chunk_limit=512):
        esp32 = StreamPort(frames, chunk_limit)
        arduino = StreamPort()
        matrices = {
            "A": np.eye(6), "B": np.zeros((6, 1)),
            "C": np.eye(6)[[0, 2, 4]],
            "Kd": np.zeros((1, 6)), "Ld": np.zeros((6, 3)),
        }
        with (
            patch.object(controller, "HANGING_SAMPLES", 1),
            patch.object(controller, "UPRIGHT_SAMPLES", 1),
            patch.object(controller, "SAMPLE_SPACING_S", 0),
            patch.object(controller, "SAVE_RUN_CSV", False),
            patch.object(controller, "balancing_output_active", False),
            patch.object(controller, "load_control_data", return_value=matrices),
            patch.object(controller.serial, "Serial", side_effect=[esp32, arduino], create=True),
            patch.object(controller, "read_position", return_value=(0, 0.0)),
            patch.object(controller.time, "sleep"),
            patch("builtins.input", return_value=""),
            redirect_stdout(io.StringIO()) as output,
            redirect_stderr(io.StringIO()),
        ):
            if expected_error:
                with self.assertRaisesRegex(Exception, expected_error):
                    controller.main()
            else:
                controller.main()
        self.assertFalse(esp32.is_open)
        self.assertFalse(arduino.is_open)
        return arduino.commands, output.getvalue()

    def test_standard_entry_point_starts_shared_controller(self):
        path = Path(__file__).resolve().parents[1] / "PythonMasterControl2.py"
        with patch.object(controller, "main") as start:
            runpy.run_path(str(path), run_name="__main__")
        start.assert_called_once_with()

    def test_split_frames_start_with_absent_third_channel_and_stop_on_interrupt(self):
        commands, _ = self.exercise([
            wire(1, first=9192), wire(2), wire(3), None,
        ], chunk_limit=1)
        self.assertEqual(commands, [-65535, 65535, -1])

    def test_old_six_byte_input_fails_calibration_before_motor_command(self):
        commands, output = self.exercise(
            [struct.pack(">HHH", 9192, 2000, 65535)] * 2,
            "2 consecutive ESP32 angle packets were unusable during setup")
        self.assertEqual(commands, [])
        self.assertIn("expected receiver protocol v1", output)

    def test_alternating_required_channel_failures_stop_after_one_prediction(self):
        commands, output = self.exercise([
            wire(1, first=9192), wire(2),
            wire(3, status1=1), wire(4, status2=2),
        ], "2 consecutive ESP32 angle packets")
        self.assertEqual(commands, [-65535, 65535, -1])
        self.assertIn("angle1 reason=encoder_read_failed", output)
        self.assertIn("angle2 reason=wireless_timeout", output)

    def test_corrupt_frame_resynchronizes_to_next_valid_frame(self):
        damaged = bytearray(wire(3))
        damaged[-1] ^= 1
        commands, _ = self.exercise([
            wire(1, first=9192), wire(2), bytes(damaged) + wire(4), None,
        ])
        self.assertEqual(commands, [-65535, 65535, -1])
        self.assertEqual(controller.angle_reader.crc_errors, 1)
        self.assertEqual(controller.angle_reader.sequence_gaps, 1)


if __name__ == "__main__":
    unittest.main()
