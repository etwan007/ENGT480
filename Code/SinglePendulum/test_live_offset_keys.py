"""Exercise terminal input and the actual controller loop without motor hardware."""

import ast
import io
import os
from pathlib import Path
import pty
import struct
import termios
import time
import types
import unittest
from unittest.mock import Mock

import numpy as np

from angle_packet_fallback import MissingAnglePacketError, estimate_with_angle
from live_offset_keys import LiveOffsetKeys
from run_csv_logger import COLUMNS
from angle_protocol import AnglePacketReader, packet_csv_values


class LiveOffsetTests(unittest.TestCase):
    def test_split_sequences_repeats_and_unrelated_keys(self):
        keys = LiveOffsetKeys()
        self.assertEqual(keys._decode(b"\x1b"), 0)
        self.assertEqual(keys._decode(b"["), 0)
        self.assertEqual(keys._decode(b"A\x1b[C\x1bOD\x1b[B"), 0)
        self.assertEqual(keys._decode(b"\x1b[C" * 5), 5)
        self.assertEqual(keys._decode(b"abcABCD\r\x1b[3~\x1b[H"), 0)
        self.assertEqual(keys._decode(b"\x1b[1;5D"), -1)

    def test_noninteractive_input_is_optional(self):
        keys = LiveOffsetKeys(io.StringIO())
        self.assertFalse(keys.start())
        self.assertEqual(keys.poll_steps(), 0)
        keys.close()

    def test_terminal_immediate_input_ctrl_c_mode_and_restoration(self):
        master, slave = pty.openpty()
        try:
            with os.fdopen(os.dup(slave), "r") as stream:
                before = termios.tcgetattr(slave)
                keys = LiveOffsetKeys(stream)
                try:
                    self.assertTrue(keys.start())
                    mode = termios.tcgetattr(slave)
                    self.assertFalse(mode[3] & (termios.ICANON | termios.ECHO))
                    self.assertTrue(mode[3] & termios.ISIG)
                    self.assertEqual(keys.poll_steps(), 0)
                    os.write(master, b"\x1b[C")
                    steps = 0
                    deadline = time.monotonic() + 1
                    while not steps and time.monotonic() < deadline:
                        steps += keys.poll_steps()
                    self.assertEqual(steps, 1)
                finally:
                    keys.close()
                restored = termios.tcgetattr(slave)
                # macOS sets this transient kernel flag when canonical mode resumes.
                restored[3] &= ~getattr(termios, "PENDIN", 0)
                before[3] &= ~getattr(termios, "PENDIN", 0)
                self.assertEqual(restored, before)
                keys.close()
        finally:
            os.close(master)
            os.close(slave)

    def test_controller_applies_keys_before_measurement_and_missing_angle_prediction(self):
        # Execute the actual loop with fake serial devices and deterministic matrices.
        # Loading the whole script would open hardware and derive the model.
        source = Path(__file__).with_name("PythonMaster_autocalibrate.py").read_text()
        tree = ast.parse(source)
        loop = next(node for node in tree.body if isinstance(node, ast.Try)
                    and any(isinstance(child, ast.While) for child in node.body))
        code = compile(ast.Module(body=[loop], type_ignores=[]), "controller-loop", "exec")
        for missing in (False, True):
            with self.subTest(missing=missing):
                keys = Mock()
                keys.poll_steps.side_effect = [1, -2, KeyboardInterrupt]
                logger = Mock()
                references = []

                def read_angle(correction):
                    references.append(correction)
                    if missing and len(references) == 1:
                        raise MissingAnglePacketError("missing")
                    return ((16383 - correction + 8192) % 16384 - 8192) * 2 * np.pi / 16384

                env = dict(np=np, time=time, struct=struct,
                           offset_keys=keys, run_logger=logger, log=Mock(),
                           angle_reader=AnglePacketReader(), latest_angle_packet=None,
                           latest_angle_message="test missing reading", packet_csv_values=packet_csv_values,
                           estimate_with_angle=estimate_with_angle,
                           MissingAnglePacketError=MissingAnglePacketError,
                           automatic_correction=16383.0, correction=16383.0,
                           UPRIGHT_OFFSET_DEG=0.0, LIVE_OFFSET_STEP_DEG=0.05,
                           ENCODER_COUNTS=16384, cycle_count=0, loop=0,
                           previous_cycle_started=None, run_started_at=time.monotonic(),
                           u=np.zeros((1, 1)), Ylast=np.zeros((4, 1)),
                           ssDisc=types.SimpleNamespace(A=np.eye(4), B=np.zeros((4, 1)),
                               C=np.array([[1, 0, 0, 0], [0, 0, 1, 0]])),
                           Ld=np.zeros((4, 2)), Kd=np.zeros((1, 4)),
                           x=0.0, consecutive_angle_misses=0, arduino=Mock(),
                           angleRead=read_angle, positionRead=lambda: 0.0,
                           latest_position_steps=0, latest_raw_angles=(16383, 65535, 65535),
                           esp32_packet_count=0, max_cycle_ms=0, SHOW_BALANCING_LOGS=False)
                exec(code, env)
                samples = [dict(zip(COLUMNS, call.args[0])) for call in logger.sample.call_args_list]
                self.assertEqual(len(samples), 2)
                for sample, offset in zip(samples, [0.05, -0.05]):
                    self.assertAlmostEqual(sample["upright_correction_counts"],
                                           (16383 + offset * 16384 / 360) % 16384)
                    self.assertAlmostEqual(sample["pred_angle_rad"], -np.deg2rad(offset))
                    self.assertAlmostEqual(sample["measured_angle_deg"], -offset)
                self.assertAlmostEqual(env["UPRIGHT_OFFSET_DEG"], -0.05)
                changes = [call for call in logger.event.call_args_list if call.args[0] == "manual_offset"]
                self.assertEqual(len(changes), 2)
                self.assertIn("manual_upright_offset_deg=-0.050000", changes[1].args[1])
                self.assertEqual(env["run_stop_reason"], "Ctrl+C")
                keys.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
