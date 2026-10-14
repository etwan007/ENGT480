"""Hardware-free phase, limit, and nonlinear-model checks for energy swing-up."""
import ast
import math
from pathlib import Path
import unittest

from swingup_control import SwingUpController


SCRIPT = Path(__file__).with_name('PythonMasterSwingUpExp.py')


class SwingUpControlTests(unittest.TestCase):
    def controller(self):
        # Execute the actual script's tuning block without ports, prompts, or motors.
        names = {'Ival_eff', 'E_ref', 'k_energy', 'a_max_swing', 'v_max_swing',
                 'k_center', 'hesitation_s', 'reversal_speed', 'startup_kick_s',
                 'swing_controller'}
        nodes = [node for node in ast.parse(SCRIPT.read_text()).body
                 if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                 and node.targets[0].id in names]
        env = dict(Ival=0.003, m1val=0.260, lval=0.05, gval=9.81,
                   SwingUpController=SwingUpController)
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SCRIPT), 'exec'), env)
        return env['swing_controller']

    def test_still_hanging_gets_a_bounded_startup_kick(self):
        control = self.controller()
        self.assertAlmostEqual(control.update(-math.pi, 0.0, 0.0, 0.01), 0.02)
        control.reset(startup=False)
        self.assertEqual(control.update(-math.pi, 0.0, 0.0, 0.01), 0.0)

    def test_acceleration_adds_energy_on_both_sides_of_bottom_and_horizontal(self):
        # For the script's coordinate convention: E_dot = m*l*a*omega*cos(theta).
        for theta in (-2.5, -1.2, 1.2, 2.5):
            for omega in (-0.8, 0.8):
                with self.subTest(theta=theta, omega=omega):
                    control = self.controller()
                    control.reset(startup=False)
                    acceleration = control.update(theta, omega, 0.0, 0.01) / 0.01
                    self.assertGreater(acceleration * omega * math.cos(theta), 0)

    def test_excess_energy_is_removed(self):
        control = self.controller()
        control.reset(startup=False)
        theta, omega = 0.5, 6.0
        acceleration = control.update(theta, omega, 0.0, 0.01) / 0.01
        self.assertLess(acceleration * omega * math.cos(theta), 0)

    def test_reversal_brakes_then_holds_then_resumes_with_limits(self):
        control = self.controller()
        control.reset(0.12, startup=False)
        control.update(2.3, 0.8, 0.0, 0.01)
        previous = control.velocity
        velocity = control.update(2.3, -0.8, 0.0, 0.01)
        self.assertEqual(control.phase, 'brake')
        self.assertAlmostEqual(velocity, previous - 0.02)
        for _ in range(20):
            if control.phase != 'brake':
                break
            previous = control.velocity
            velocity = control.update(2.3, -0.8, 0.0, 0.01)
            self.assertLessEqual(abs(velocity - previous), 0.02 + 1e-12)
        self.assertEqual(control.phase, 'hesitate')
        hold_step = control.hesitation_s / 10
        for _ in range(9):
            # Centering is deliberately suspended for the configured hold time.
            self.assertEqual(control.update(2.3, -0.8, 0.1, hold_step), 0.0)
            self.assertEqual(control.phase, 'hesitate')
        for _ in range(3):
            control.update(2.3, -0.8, 0.0, hold_step)
        self.assertEqual(control.phase, 'pump')
        self.assertGreater(control.velocity, 0.0)

    def test_small_velocity_noise_and_upright_pass_do_not_trigger_a_pause(self):
        control = self.controller()
        control.reset(startup=False)
        control.update(2.3, 0.8, 0.0, 0.01)
        for omega in (0.1, -0.1, 0.2, -0.2):
            control.update(2.3, omega, 0.0, 0.01)
            self.assertEqual(control.phase, 'pump')
        control.update(0.2, -0.8, 0.0, 0.01)
        self.assertEqual(control.phase, 'pump')

    def test_missing_measurement_freezes_phase_and_velocity(self):
        control = self.controller()
        self.assertEqual(control.update(-math.pi, 0.0, 0.0, 0.01,
                                        valid_measurement=False), 0.0)
        self.assertEqual(control.kick_remaining, control.kick_s)
        control.reset(0.2, startup=False)
        control.update(2.3, 0.8, 0.0, 0.01)
        previous = (control.velocity, control.direction, control.phase)
        control.update(2.3, -0.8, 0.0, 0.01, valid_measurement=False)
        self.assertEqual((control.velocity, control.direction, control.phase), previous)

    def test_speed_acceleration_and_delayed_loop_limits(self):
        control = self.controller()
        control.reset(startup=False)
        for dt in (0.008, 0.013, 0.01, 0.20) * 200:
            previous = control.velocity
            velocity = control.update(2.5, -4.0, 0.0, dt)
            self.assertLessEqual(abs(velocity), 0.40)
            self.assertLessEqual(abs(velocity - previous), 2 * min(dt, 0.03) + 1e-12)
        for dt in (0, -0.01, float('nan'), float('inf')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                control.update(2.5, -4.0, 0.0, dt)

    def test_fallback_clears_old_hesitation_without_a_new_kick(self):
        control = self.controller()
        control.phase = 'hesitate'
        control.hold_remaining = 0.10
        control.direction = 1
        control.reset(-0.2, startup=False)
        self.assertEqual(control.phase, 'pump')
        self.assertEqual(control.hold_remaining, 0.0)
        self.assertEqual(control.direction, 0)
        self.assertEqual(control.kick_remaining, 0.0)
        self.assertEqual(control.velocity, -0.2)

    def test_script_requires_a_measured_angle_for_catch(self):
        tree = ast.parse(SCRIPT.read_text())
        catch = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                     and any(isinstance(item, ast.Assign)
                             and isinstance(item.value, ast.Name)
                             and item.value.id == 'MODE_LQG' for item in node.body))
        expression = compile(ast.Expression(catch.test), str(SCRIPT), 'eval')
        environment = dict(theta1=0.1, theta_dot_est=0.0, theta_catch=0.35,
                           thetaDot_catch=5.0, measured_theta=None)
        self.assertFalse(eval(expression, environment))
        environment['measured_theta'] = 0.1
        self.assertTrue(eval(expression, environment))

    def test_damped_nonlinear_model_from_rest_rises_above_horizontal(self):
        # Model assumption: the stepper tracks velocity perfectly, with the
        # angle/acceleration sign from the existing script. This is NOT a rig test.
        # Include encoder quantization, the script's finite-difference IIR,
        # small pivot friction, and variable control-cycle periods.
        for friction in (0.0, 0.0005, 0.001):
            with self.subTest(friction=friction):
                control = self.controller()
                theta, omega, cart, velocity, filtered = math.pi, 0.0, 0.0, 0.0, 0.0
                previous_angle = math.pi
                previous_period = 0.01
                pauses = 0
                previous_phase = 'pump'
                crossed = False
                elapsed = 0.0
                for cycle in range(1800):
                    dt = (0.008, 0.013, 0.01)[cycle % 3]
                    angle = round(theta * 16384 / (2 * math.pi)) * (2 * math.pi / 16384)
                    angle = (angle + math.pi) % (2 * math.pi) - math.pi
                    delta = (angle - previous_angle + math.pi) % (2 * math.pi) - math.pi
                    filtered = 0.6 * filtered + 0.4 * delta / previous_period
                    previous_angle = angle
                    new_velocity = control.update(angle, filtered, cart, previous_period)
                    acceleration = (new_velocity - velocity) / dt
                    # RK4 integrates pendulum response to each held acceleration.
                    def derivative(state):
                        position, speed = state
                        return (speed, (0.013 * acceleration * math.cos(position)
                                + 0.12753 * math.sin(position) - friction * speed) / 0.00365)
                    for _ in range(5):
                        h = dt / 5
                        q = (theta, omega)
                        k1 = derivative(q)
                        k2 = derivative(tuple(q[j] + h * k1[j] / 2 for j in range(2)))
                        k3 = derivative(tuple(q[j] + h * k2[j] / 2 for j in range(2)))
                        k4 = derivative(tuple(q[j] + h * k3[j] for j in range(2)))
                        theta, omega = tuple(q[j] + h / 6 * (k1[j] + 2 * k2[j]
                                             + 2 * k3[j] + k4[j]) for j in range(2))
                    cart += (new_velocity + velocity) * dt / 2
                    velocity = new_velocity
                    elapsed += dt
                    previous_period = dt
                    if control.phase == 'hesitate' and previous_phase != 'hesitate':
                        pauses += 1
                    previous_phase = control.phase
                    if abs((theta + math.pi) % (2 * math.pi) - math.pi) < math.pi / 2:
                        crossed = True
                        break
                self.assertTrue(crossed, f'friction={friction}; time={elapsed:.2f}s')
                self.assertGreater(pauses, 0)
                self.assertLess(elapsed, 15.0)


if __name__ == '__main__':
    unittest.main()
