"""Digital twin of the rig: plant + stepper + encoders + radio + Pi loop timing.

A controller object is stepped exactly like the hardware loop in
PythonMaster2_auto.py:

    1. read the newest angle frame (age = sensor delay + TX sample age),
    2. compute a cart velocity command from it and the PREVIOUS Arduino
       position reply,
    3. send the timer command (applied after the serial/actuator delay),
    4. read the new step-count position reply.

Controllers implement ``update(obs) -> ControlOutput`` (see Observation /
ControlOutput below); both the improved controller and the port of the
team's existing controller (legacy_controller.py) do.

What is modelled
  * nonlinear link dynamics with viscous + Coulomb joint friction (twin_model)
  * closed-loop stepper following the pulse train, accel ceiling, timer
    quantisation of the step rate (stepper_mapping, same as the Pi scripts)
  * limit switches at +/- rail_limit_m: the Arduino disables the motor for good
  * 14-bit encoder quantisation + noise, TX sample age (200 Hz), radio and USB
    delay, random lost frames, calibration offsets
  * Pi loop period and jitter
"""

from dataclasses import dataclass, field
import csv
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CODE_DIRECTORY = HERE.parent
for path in (CODE_DIRECTORY, CODE_DIRECTORY / "DoublePendulum" / "Improved"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from twin_model import (  # noqa: E402
    TwinParameters, load_parameters, rk4_step, free_cart_step, wrap_angle,
    energy, upright_energy,
)
from stepper_mapping import (  # noqa: E402
    timer_command_for_velocity, velocity_for_timer_command, METRES_PER_STEP,
)

TX_PERIOD_S = 1 / 200      # TX1/TX2 SEND_HZ
STOP_DECEL_M_S2 = 15.0     # cart coasting to rest with the driver disabled (guess)
USB_DELAY_S = 0.001        # part of the sensor delay the receiver's age field cannot see


@dataclass
class Observation:
    t: float                 # controller clock, s
    dt: float                # time since the previous cycle, s
    cycle: int
    theta1: object           # absolute model angle, rad (None = channel missing)
    theta2: object
    cart_m: float            # step-count position from the previous Arduino reply
    raw1: object = None      # encoder counts (for CSV)
    raw2: object = None
    age_s: object = None     # sample age as the receiver reports it (radio_age_ms), s
    truth: object = None     # true plant state now (debug/tests only; real rig has none)
    sampled_truth: object = None  # true state at the encoder sample time (debug only)


@dataclass
class ControlOutput:
    velocity: float          # commanded cart velocity, m/s
    stop: bool = False       # True -> send the motor-disable command
    mode: str = ""
    accel: float = float("nan")
    reason: str = ""
    extra: dict = field(default_factory=dict)


@dataclass
class SimConfig:
    duration_s: float = 20.0
    dt_sim: float = 0.001
    seed: int = 0
    offset1_rad: float = 0.0       # calibration error of arm 1 (true - reported)
    offset_joint_rad: float = 0.0  # calibration error of the joint encoder
    disturbance_std: float = 0.0   # random torque-ish angular accel kicks, rad/s^2
    record_every: int = 1


class RigSimulator:
    def __init__(self, params=None, config=None):
        self.p = params if params is not None else load_parameters()
        self.cfg = config if config is not None else SimConfig()
        self.rng = np.random.default_rng(self.cfg.seed)

    # ------------------------------------------------------------------
    def _history_state(self, hist_t, hist_x, t_query):
        """Linear interpolation of the true state at an earlier time."""
        i = int(np.searchsorted(hist_t, t_query))
        i = min(max(i, 1), len(hist_t) - 1)
        t0, t1 = hist_t[i - 1], hist_t[i]
        w = 0.0 if t1 == t0 else (t_query - t0) / (t1 - t0)
        w = min(max(w, 0.0), 1.0)
        return hist_x[i - 1] * (1 - w) + hist_x[i] * w

    def _encode(self, th1, th2):
        p = self.p
        counts = p.encoder_counts
        scale = counts / (2 * math.pi)
        n1 = (th1 - self.cfg.offset1_rad) * scale + self.rng.normal(0, p.encoder_noise_counts)
        nj = (wrap_angle(th2 - th1) - self.cfg.offset_joint_rad) * scale \
            + self.rng.normal(0, p.encoder_noise_counts)
        q1 = round(n1)
        qj = round(nj)
        theta1 = wrap_angle(q1 / scale)
        joint = wrap_angle(qj / scale)
        return theta1, wrap_angle(theta1 + joint), int(q1 % counts), int(qj % counts)

    # ------------------------------------------------------------------
    def run(self, controller, initial_state, duration_s=None, on_sample=None):
        """Simulate; returns a dict of numpy arrays (one row per control cycle)."""
        p, cfg = self.p, self.cfg
        duration = cfg.duration_s if duration_s is None else duration_s
        dt = cfg.dt_sim
        x = np.array(initial_state, dtype=float)
        x_cmd = x[4]                 # pulse position (what the Arduino counts)
        v_q = x[5]                   # commanded velocity in effect (starts at the cart speed)
        pending = []                 # (apply_time, v_q, stop)
        motor_enabled = True
        limit_hit = False
        stopped_by_controller = False
        t = 0.0
        hist_len = int(0.2 / dt) + 4
        hist_t = np.zeros(hist_len)
        hist_x = np.zeros((hist_len, 6))
        hist_n = 0

        def push_hist(tt, xx):
            nonlocal hist_n, hist_t, hist_x
            if hist_n == hist_len:
                hist_t[:-1] = hist_t[1:]
                hist_x[:-1] = hist_x[1:]
                hist_n -= 1
            hist_t[hist_n] = tt
            hist_x[hist_n] = xx
            hist_n += 1

        push_hist(t, x)
        next_ctrl = 0.0
        last_ctrl = None
        reply_x = x_cmd
        cycle = 0
        rows = []
        stop_reason = ""
        tx_phase = self.rng.uniform(0, TX_PERIOD_S)
        while t < duration - 1e-12:
            # ---- control cycle -------------------------------------------
            if t >= next_ctrl - 1e-12:
                cycle += 1
                ctrl_dt = p.control_period_s if last_ctrl is None else t - last_ctrl
                last_ctrl = t
                lost = self.rng.random() < p.packet_loss
                # newest TX sample before (t - transport delay)
                t_avail = t - p.sensor_delay_s
                t_sample = math.floor((t_avail - tx_phase) / TX_PERIOD_S) * TX_PERIOD_S + tx_phase
                xs = self._history_state(hist_t[:hist_n], hist_x[:hist_n], max(t_sample, 0.0))
                th1_m, th2_m, raw1, raw2 = self._encode(xs[0], xs[2])
                # The receiver reports each channel's radio age in whole ms; the
                # USB/serial part (~1 ms) is not included in that number.
                age_s = max(0.0, round((t - t_sample - USB_DELAY_S) * 1000) / 1000)
                if lost:
                    th1_m = th2_m = None
                    raw1 = raw2 = 0xFFFF
                    age_s = None
                obs = Observation(t=t, dt=ctrl_dt, cycle=cycle, theta1=th1_m, theta2=th2_m,
                                  cart_m=reply_x, raw1=raw1, raw2=raw2, age_s=age_s, truth=x.copy(),
                                  sampled_truth=xs.copy())
                out = controller.update(obs)
                if out.stop and motor_enabled:
                    stop_reason = out.reason or "controller stop"
                    stopped_by_controller = True
                    pending.append((t + p.actuator_delay_s, 0.0, True))
                elif motor_enabled:
                    step_hz, timer = timer_command_for_velocity(out.velocity)
                    pending.append((t + p.actuator_delay_s,
                                    velocity_for_timer_command(timer), False))
                else:
                    step_hz, timer = 0.0, 65535
                if out.stop:
                    step_hz, timer = 0.0, -1
                # the reply is the pulse count when the command lands
                reply_x = x_cmd + v_q * p.actuator_delay_s if motor_enabled else x_cmd
                reply_x = round(reply_x / METRES_PER_STEP) * METRES_PER_STEP
                if cycle % cfg.record_every == 0:
                    rows.append((t, *x, x_cmd, out.velocity, v_q, out.accel,
                                 np.nan if th1_m is None else th1_m,
                                 np.nan if th2_m is None else th2_m,
                                 reply_x, out.mode, step_hz, timer, raw1, raw2,
                                 energy(p, x[0], x[1], x[2], x[3])))
                    if on_sample is not None:
                        on_sample(rows[-1])
                period = p.control_period_s + self.rng.normal(0, p.control_jitter_s)
                next_ctrl = t + max(0.5 * p.control_period_s, period)
            # ---- apply due commands ------------------------------------
            while pending and pending[0][0] <= t + 1e-12:
                _, v_new, stop = pending.pop(0)
                if stop:
                    motor_enabled = False
                if motor_enabled:
                    v_q = v_new
            # ---- integrate ---------------------------------------------
            if motor_enabled:
                x = rk4_step(p, x, x_cmd, v_q, dt)
                x_cmd += v_q * dt
            else:
                x = free_cart_step(p, x, STOP_DECEL_M_S2, dt)
                x_cmd = x[4]
                v_q = 0.0
            if cfg.disturbance_std > 0:
                x[1] += self.rng.normal(0, cfg.disturbance_std) * math.sqrt(dt)
                x[3] += self.rng.normal(0, cfg.disturbance_std) * math.sqrt(dt)
            t += dt
            push_hist(t, x)
            if motor_enabled and abs(x[4]) >= p.rail_limit_m:
                motor_enabled = False
                limit_hit = True
                stop_reason = stop_reason or f"limit switch at x={x[4]:+.3f} m"
            if stopped_by_controller and abs(x[5]) < 1e-6 and t > (rows[-1][0] if rows else 0) + 0.5:
                break
        names = ("t", "th1", "w1", "th2", "w2", "x", "v", "x_cmd", "v_req", "v_q", "a_cmd",
                 "th1_meas", "th2_meas", "cart_reply", "mode", "step_hz", "timer", "raw1", "raw2",
                 "energy")
        cols = list(zip(*rows)) if rows else [[] for _ in names]
        result = {n: (np.array(c, dtype=object) if n == "mode" else np.array(c, dtype=float))
                  for n, c in zip(names, cols)}
        result.update(limit_hit=limit_hit, stop_reason=stop_reason,
                      stopped_by_controller=stopped_by_controller,
                      upright_energy=upright_energy(p), params=p)
        return result


# ----------------------------------------------------------------------
def hanging_state(x=0.0, th1=math.pi, th2=math.pi):
    return np.array([th1, 0.0, th2, 0.0, x, 0.0])


def write_run_csv(result, path, controller_name="improved"):
    """Write a CSV with Code/Global/run_csv_logger.py's DOUBLE_COLUMNS header so
    Code/RunReview/balancing_visualizer.html can replay the simulated run."""
    from Global.run_csv_logger import DOUBLE_COLUMNS
    import json
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    idx = {c: i for i, c in enumerate(DOUBLE_COLUMNS)}
    blank = [""] * len(DOUBLE_COLUMNS)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(DOUBLE_COLUMNS)
        row = list(blank)
        row[0], row[1] = "config", 0.0
        row[-1] = json.dumps({"schema_version": 2, "pendulum_count": 2, "source": "digital twin",
                              "controller": controller_name, "SIMULATED": True},
                             separators=(",", ":"))
        w.writerow(row)
        row = list(blank)
        row[0], row[1], row[-1] = "start", 0.0, "SIMULATED run from Code/DigitalTwin"
        w.writerow(row)
        n = len(result["t"])
        for k in range(n):
            row = list(blank)
            th1 = result["th1_meas"][k]
            th2 = result["th2_meas"][k]
            th1 = result["th1"][k] if not np.isfinite(th1) else th1
            th2 = result["th2"][k] if not np.isfinite(th2) else th2
            th1, th2 = wrap_angle(th1), wrap_angle(th2)
            j = wrap_angle(th2 - th1)
            vals = {
                "record_type": "sample", "elapsed_s": f"{result['t'][k]:.4f}", "cycle": k + 1,
                "raw_angle_1": int(result["raw1"][k]), "raw_angle_2": int(result["raw2"][k]),
                "measured_angle_1_rad": f"{th1:.6f}", "measured_angle_1_deg": f"{math.degrees(th1):.4f}",
                "measured_angle_2_rad": f"{th2:.6f}", "measured_angle_2_deg": f"{math.degrees(th2):.4f}",
                "joint_angle_2_rad": f"{j:.6f}", "joint_angle_2_deg": f"{math.degrees(j):.4f}",
                "cart_used_m": f"{result['cart_reply'][k]:.5f}",
                "cart_steps": int(round(result["cart_reply"][k] / METRES_PER_STEP)),
                "cart_m": f"{result['cart_reply'][k]:.5f}",
                "est_cart_velocity_m_s": f"{result['v'][k]:.4f}",
                "force_command_n": "" if not np.isfinite(result["a_cmd"][k]) else f"{result['a_cmd'][k]:.4f}",
                "cart_velocity_command_m_s": f"{result['v_req'][k]:.4f}",
                "step_frequency_hz": f"{result['step_hz'][k]:.1f}",
                "motor_timer_command": int(result["timer"][k]),
                "loop_ms": "0.5",
                "message": str(result["mode"][k]),
            }
            for key, val in vals.items():
                row[idx[key]] = val
            w.writerow(row)
        row = list(blank)
        row[0] = "stop"
        row[-1] = f"SIMULATED; {result['stop_reason'] or 'end of simulation'}; dropped_samples=0"
        w.writerow(row)
    return path
