"""Exercise swing-up protocol integration without opening ports or commanding hardware."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np
import test_controller_angle_protocol as base_tests
from angle_packet_fallback import estimate_with_angle, MissingAnglePacketError
from angle_protocol import decode_frame, packet_csv_values
from run_csv_logger import COLUMNS
from test_angle_protocol import frame


SCRIPT = Path(__file__).with_name('PythonMasterSwingUpExp.py')


def execute(nodes, env):
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SCRIPT), 'exec'), env)


class SwingUpAngleProtocolTests(base_tests.ControllerAngleProtocolTests):
    def environment(self, data=b''):
        env = super().environment(data)
        tree = ast.parse(SCRIPT.read_text())
        execute([node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in ('read_raw_angle1', 'stop_and_close', 'update_theta_velocity')], env)
        env.update(np=np, theta_prev=None, theta_prev_receiver_ms=None,
                   theta_dot_est=0.0, alpha_td=0.6)
        return env

    def packet(self, sequence, ms):
        return decode_frame(frame(sequence, ms))

    def test_newest_packet_and_partial_frame_are_preserved(self):
        third = frame(3, 1020, ((10000, 0, 2), (65535, 3, 0xFFFFFFFF), (65535, 3, 0xFFFFFFFF)))
        env = self.environment(frame(1) + frame(2, 1010) + third[:17])
        self.assertEqual(env['read_raw_angle1'](), 9054)
        self.assertEqual(env['latest_angle_packet'].sequence, 2)
        env['esp32'].data.extend(third[17:])
        self.assertEqual(env['read_raw_angle1'](), 10000)

    def test_missing_channel_and_transport_failure_keep_one_miss_then_stop(self):
        for data in (b'', frame(channels=((65535, 2, 70),
                                         (65535, 3, 0xFFFFFFFF), (65535, 3, 0xFFFFFFFF)))):
            with self.subTest(data=data):
                env = self.environment(data)
                predicted = np.array([[0.2], [0.3], [0.0], [0.0]])
                misses = 0
                for cycle in range(2):
                    with self.assertRaises(MissingAnglePacketError):
                        env['read_raw_angle1']()
                    if cycle == 0:
                        _, angle, misses = estimate_with_angle(
                            predicted, None, 0.0, np.eye(2, 4), np.zeros((4, 2)), misses)
                        self.assertEqual(angle, 0.2)
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'Two consecutive'):
                            estimate_with_angle(predicted, None, 0.0, np.eye(2, 4),
                                                np.zeros((4, 2)), misses)

    def test_velocity_uses_elapsed_time_across_skipped_frames_and_missing_read(self):
        env = self.environment()
        update = env['update_theta_velocity']
        update(0.1, self.packet(1, 1000))
        update(None, None)
        update(0.16, self.packet(4, 1030))
        # 0.06 rad / 0.03 s = 2 rad/s, filtered by the existing 0.6 coefficient.
        self.assertAlmostEqual(env['theta_dot_est'], 0.8)
        update(0.18, self.packet(5, 1040))
        self.assertAlmostEqual(env['theta_dot_est'], 1.28)

    def test_velocity_wraps_angle_at_bottom(self):
        env = self.environment()
        env['update_theta_velocity'](np.pi - 0.01, self.packet(1, 1000))
        env['update_theta_velocity'](-np.pi + 0.01, self.packet(2, 1010))
        self.assertAlmostEqual(env['theta_dot_est'], 0.8)

    def test_velocity_handles_uptime_wrap_and_reseeds_after_restart(self):
        env = self.environment()
        update = env['update_theta_velocity']
        update(0.1, self.packet(1, 0xFFFFFFFA))
        update(0.12, self.packet(2, 4))
        self.assertAlmostEqual(env['theta_dot_est'], 0.8)
        update(0.14, self.packet(3, 10000))
        update(0.2, self.packet(0, 0))
        self.assertEqual(env['theta_dot_est'], 0.0)
        update(0.22, self.packet(1, 10))
        self.assertAlmostEqual(env['theta_dot_est'], 0.8)

    def test_zero_timestamp_interval_reseeds_without_division(self):
        env = self.environment()
        env['update_theta_velocity'](0.1, self.packet(1, 1000))
        env['theta_dot_est'] = 1.0
        env['update_theta_velocity'](0.2, self.packet(2, 1000))
        self.assertEqual(env['theta_dot_est'], 0.0)
        self.assertEqual(env['theta_prev'], 0.2)

    def test_initial_velocity_seed_uses_paired_last_sample_and_timestamp(self):
        env = self.environment()
        env.update(initial_angles=(3.1, -3.1), theta1=0.0,
                   latest_angle_packet=self.packet(10, 1200))
        tree = ast.parse(SCRIPT.read_text())
        seeds = [node for node in tree.body if isinstance(node, ast.Assign)
                 and isinstance(node.targets[0], ast.Name)
                 and node.targets[0].id in ('theta_prev', 'theta_prev_receiver_ms')
                 and not isinstance(node.value, ast.Constant)]
        execute(seeds, env)
        env['update_theta_velocity'](-3.08, self.packet(11, 1210))
        self.assertAlmostEqual(env['theta_dot_est'], 0.8)

    def test_live_offset_does_not_create_velocity(self):
        env = self.environment()
        env['update_theta_velocity'](np.pi - 0.001, self.packet(1, 1000))
        env.update(offset_steps=-1, LIVE_OFFSET_STEP_DEG=0.5,
                   UPRIGHT_OFFSET_DEG=0.0, automatic_correction=9000.0,
                   ENCODER_COUNTS=16384, Ylast=np.zeros((4, 1)), run_logger=None)
        tree = ast.parse(SCRIPT.read_text())
        offset = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                      and isinstance(node.test, ast.Name) and node.test.id == 'offset_steps')
        execute([offset], env)
        shifted_angle = (np.pi - 0.001 + np.deg2rad(0.5) + np.pi) % (2 * np.pi) - np.pi
        env['update_theta_velocity'](shifted_angle, self.packet(2, 1010))
        self.assertAlmostEqual(env['theta_dot_est'], 0.0)

    def test_setup_drains_old_complete_readings_before_both_measurements(self):
        tree = ast.parse(SCRIPT.read_text())
        for measurement in ('automatic_correction', 'initial_samples'):
            with self.subTest(measurement=measurement):
                index = next(i for i, node in enumerate(tree.body) if isinstance(node, ast.Assign)
                             and measurement in {name.id for target in node.targets
                                                 for name in ast.walk(target) if isinstance(name, ast.Name)})
                env = self.environment(frame(1))
                execute([tree.body[index - 1]], env)
                env['esp32'].data.extend(frame(2, 1010))
                env['read_raw_angle1']()
                self.assertEqual(env['latest_angle_packet'].sequence, 2)
                self.assertEqual(env['esp32_packet_count'], 1)

    def test_actual_sample_row_matches_current_csv_header(self):
        tree = ast.parse(SCRIPT.read_text())
        sample = next(node for node in ast.walk(tree) if isinstance(node, ast.Expr)
                      and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Attribute)
                      and node.value.func.attr == 'sample')
        for packet in (self.packet(12, 1400), None):
            with self.subTest(packet=packet):
                logger = Mock()
                env = dict(run_logger=logger, cycle_started=10.0, run_started_at=9.0,
                           cycle_count=1, esp32_packet_count=2,
                           latest_raw_angles=(9054, 65535, 65535), correction=8500,
                           theta1=0.1, theta_degrees=np.rad2deg(0.1), cart_used_m=0.0,
                           latest_position_steps=12, x=0.01,
                           Yest=np.zeros((4, 1)), YfinalEst=np.zeros((4, 1)),
                           previous_force_n=0.0, force_command_n=0.1, ramp_fraction=1.0,
                           f=123.0, timer_command=1000, sensor_read_ms=2.0,
                           position_read_ms=1.0, loop_ms=10.0, cycle_period_ms=10.0,
                           packet_csv_values=packet_csv_values, latest_angle_packet=packet,
                           angle_reader=self.environment()['angle_reader'], angle_packet_message='example')
                execute([sample], env)
                row = logger.sample.call_args.args[0]
                self.assertEqual(len(row), len(COLUMNS))
                values = dict(zip(COLUMNS, row))
                self.assertEqual(values['message'], 'example')
                self.assertEqual(values['esp32_sequence'], 12 if packet else '')
                self.assertEqual(values['angle_status_1'], 'valid' if packet else '')
                self.assertEqual(values['radio_age_1_ms'], 2 if packet else '')


if __name__ == '__main__':
    unittest.main()
