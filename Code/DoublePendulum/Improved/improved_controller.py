"""Improved double-pendulum controller: energy swing-up -> LQR catch -> balance.

Pure NumPy, no serial: the same object runs on the Pi
(PythonMaster2_improved.py) and against the digital twin (--sim).

Angles follow PythonMaster2_auto.py: theta1, theta2 absolute from upright
(rad), positive tips toward +x. The output is a cart VELOCITY command per
cycle (the Arduino turns it into a step rate), plus a stop flag.

Modes
  WAIT      cart held at centre until the arms hang calm, then SWING
  SWING     swing_mode "trajectory" (default): follow the offline plan in
            swingup_plan.json (swingup_trajectory.py) with time-varying LQR
            feedback and a model-based observer, abort to RECOVER if an arm
            gets track_abort_deg off plan, hand off to BALANCE at the end.
            swing_mode "energy": energy pumping toward the upright energy (cart acceleration law
            a = -k (E* - E) sat(y), with y = d/dt of the arms' horizontal
            mass moment), plus damping of the relative (joint) motion and cart
            centring. A braking-distance governor keeps the cart inside the
            soft rail limits at all times.
  BALANCE   discrete LQR (design_lqr.py) + Kalman filter on the link states +
            integral action on cart position. Entered when the arms are near
            upright AND a short closed-loop prediction with the real
            accel/speed/rail limits says the catch will hold. The first
            catch_hold_s use the stiffer "catch" gains, then the hold profile
            ("conservative" by default). Left (hysteresis) when an arm passes
            drop_angle_deg for drop_hold_s.
  RECOVER   after a drop or a swing timeout: brake the swing (energy removal),
            re-centre, then WAIT and SWING again, up to max_attempts.
  STOPPING  e-stop (rail, missing frames, a drop in balance-only mode, too many
            attempts): the cart is braked to rest with the motor first,
  STOPPED   then the stop flag is raised and the caller sends the
            motor-disable command. A stalled loop or non-finite data skips the
            braking. The controller never restarts by itself.
"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

GAINS_PATH = Path(__file__).with_name("improved_gains.json")
PLAN_PATH = Path(__file__).with_name("swingup_plan.json")

try:  # the planned swing-up shares its one-cycle model with the offline planner
    from plan_model import cycle as plan_cycle
except ImportError:  # pragma: no cover
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent))
    from plan_model import cycle as plan_cycle


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(v, lim):
    return max(-lim, min(lim, v))


@dataclass
class ControllerConfig:
    # --- cart-frame energy constants (overwritten from the gains file) ---
    J1: float = 0.0112
    J2: float = 0.00434
    c: float = 0.00445
    h1: float = 0.0600
    h2: float = 0.0223
    g: float = 9.81
    # --- actuator limits (conservative for first hardware tests) ---
    max_speed: float = 1.0           # m/s  (same as the team's swing script; the plan uses <= 0.8)
    max_accel_swing: float = 8.0     # m/s^2
    max_accel_balance: float = 15.0  # m/s^2 (catch; the hold gains rarely use > 3)
    max_accel_track: float = 12.0    # m/s^2 while following the swing-up plan (plan itself <= 7)
    # --- rail ---
    soft_limit_m: float = 0.28       # governor: cart can always stop before this
    brake_accel: float = 6.0         # m/s^2 assumed for the stopping distance
    estop_limit_m: float = 0.36      # beyond this: stop the motor (switches ~0.45 m)
    # --- swing-up ---
    swing_mode: str = "trajectory"   # "trajectory" (planned + TVLQR) or "energy"
    track_abort_deg: float = 30.0    # trajectory mode: give up if an arm is this far off plan
    track_L_angle: float = 0.5       # trajectory-mode observer gains (angle, rate)
    track_L_rate: float = 12.0
    early_handoff_s: float = 0.0     # allow the LQR catch this long before the plan ends
    retime_samples: bool = True      # remove the 0-5 ms TX sample-age jitter using the
                                     # receiver's radio_age field (protocol v1)
    energy_gain: float = 30.0        # m/s^2 per J of energy error
    energy_margin: float = 0.0       # fraction above the upright energy to aim for
    pump_sat: float = 0.15           # |y| (kg m^2/s) giving full pumping authority
    sync_gain: float = 0.02          # damping of the joint (mode-2) motion while pumping
    swing_center_kp: float = 6.0     # 1/s^2
    swing_center_kd: float = 3.0     # 1/s
    kick_accel: float = 3.0          # breaks the still-hanging equilibrium
    kick_s: float = 0.25
    swing_timeout_s: float = 30.0
    # --- catch / balance ---
    catch_angle_deg: float = 25.0    # both arms within this of upright to consider a catch
    catch_horizon_s: float = 0.8     # closed-loop prediction used to accept a catch
    catch_ok_angle_deg: float = 20.0  # predicted arm angles must stay inside this
    catch_hold_s: float = 1.5        # use the stiff catch gains this long, then hold gains
    drop_angle_deg: float = 35.0
    drop_hold_s: float = 0.05
    # --- recovery ---
    recover_damp_gain: float = 25.0
    calm_energy_j: float = 0.02
    calm_time_s: float = 0.8
    recover_timeout_s: float = 30.0
    max_attempts: int = 5
    # --- safety ---
    max_missing_frames: int = 3      # consecutive unusable frames allowed (stop on the next)
    max_cycle_s: float = 0.05        # a stalled loop stops the motor
    stop_brake_accel: float = 8.0    # m/s^2 used to bring the cart to rest before disabling
    balance_only: bool = False       # start upright and only balance (like PythonMaster2_auto)
    alpha: float = 0.6               # alpha-beta filter gains for the swing-mode rates
    beta: float = 0.25


@dataclass
class ControlOutput:
    velocity: float
    stop: bool = False
    mode: str = ""
    accel: float = float("nan")
    reason: str = ""
    extra: dict = field(default_factory=dict)


class AlphaBeta:
    """Unwrapped angle + rate tracker that tolerates missing samples."""

    def __init__(self, alpha, beta):
        self.alpha, self.beta = alpha, beta
        self.angle = None
        self.rate = 0.0

    def reset(self, angle, rate=0.0):
        self.angle, self.rate = angle, rate

    def update(self, measured, dt):
        if self.angle is None:
            if measured is not None:
                self.angle = measured
            return
        pred = self.angle + self.rate * dt
        if measured is None:
            self.angle = pred
            return
        err = wrap(measured - pred)
        self.angle = pred + self.alpha * err
        self.rate += self.beta * err / dt


class GainSet:
    """One LQR/Kalman design from improved_gains.json."""

    def __init__(self, g):
        self.name = g["profile"]
        self.Ts = g["Ts"]
        self.K = np.array(g["K"]).reshape(-1)
        self.P = np.array(g["P"])
        Ad = np.array(g["Ad"])
        B1 = np.array(g["B1"]).reshape(-1)
        B2 = np.array(g["B2"]).reshape(-1)
        self.Ad4, self.B14, self.B24 = Ad[:4, :4], B1[:4], B2[:4]
        self.L = np.array(g["L"])
        self.Bq = np.array(g["Bq"])
        self.params = g.get("params")
        # augmented model z = [s(5), v_prev, xi], input dv (for catch prediction)
        Az = np.zeros((7, 7))
        Bz = np.zeros(7)
        Az[:5, :5] = Ad
        Az[:5, 5] = B1 + B2
        Bz[:5] = B2
        Az[5, 5] = 1.0
        Bz[5] = 1.0
        Az[6, 4] = self.Ts
        Az[6, 6] = 1.0
        self.Az, self.Bz = Az, Bz


def load_gains(path=GAINS_PATH, profile=None):
    data = json.loads(Path(path).read_text())
    name = profile or data.get("default_profile", "conservative")
    return data["profiles"][name]


class ImprovedController:
    def __init__(self, gains=None, config=None, profile=None, catch_gains=None,
                 gains_path=GAINS_PATH, plan=None, plan_path=PLAN_PATH):
        hold = gains if gains is not None else load_gains(gains_path, profile)
        catch = catch_gains if catch_gains is not None else load_gains(gains_path, "catch")
        self.hold = GainSet(hold)
        self.catch = GainSet(catch)
        cfg = config if config is not None else ControllerConfig()
        prm = self.hold.params
        if prm:  # energy constants from the same model the gains were designed on
            cfg.J1, cfg.J2, cfg.c = prm["J1"], prm["J2"], prm["c"]
            cfg.h1, cfg.h2 = prm["h1"], prm["h2"]
        self.cfg = cfg
        self.Bq = self.hold.Bq
        self.Ts = self.hold.Ts
        self.plan = None
        if cfg.swing_mode == "trajectory" and not cfg.balance_only:
            if plan is None and Path(plan_path).exists():
                plan = json.loads(Path(plan_path).read_text())
            if plan is None:
                raise FileNotFoundError(f"{plan_path} missing: run swingup_trajectory.py "
                                        "or use swing_mode='energy'")
            self.plan = plan
            self.plan_Z = np.array(plan["z_nominal"])
            self.plan_V = np.concatenate([self.plan_Z[1:, 5], [plan["z_final"][5]]])
            self.plan_K = np.array(plan["K"])
            self.plan_K_final = np.array(plan["K_final"])
            self.plan_k = {kk: float(v) for kk, v in plan["constants"].items()}
            self.plan_Ts = float(plan["Ts"])
            self.plan_delay = float(plan["delay"])
            self.plan_T = len(self.plan_V) * self.plan_Ts
        self.reset()

    # ------------------------------------------------------------------
    @property
    def upright_energy(self):
        return 2 * self.cfg.g * (self.cfg.h1 + self.cfg.h2)

    def energy(self, th1, w1, th2, w2):
        c = self.cfg
        return (0.5 * c.J1 * w1 ** 2 + 0.5 * c.J2 * w2 ** 2 + c.c * math.cos(th1 - th2) * w1 * w2
                + c.g * (c.h1 * (math.cos(th1) + 1) + c.h2 * (math.cos(th2) + 1)))

    def reset(self, mode=None):
        c = self.cfg
        self.mode = mode or ("BALANCE" if c.balance_only else "WAIT")
        self.f1 = AlphaBeta(c.alpha, c.beta)
        self.f2 = AlphaBeta(c.alpha, c.beta)
        self.v = 0.0                 # velocity commanded last cycle (in effect now)
        self.v_prev = 0.0            # the one before (for the latency model)
        self.xi = 0.0
        self.s = None                # Kalman link state [th1, w~1, th2, w~2]
        self.active = self.hold if c.balance_only else self.catch
        self.mode_time = 0.0
        self.time = 0.0
        self.missing = 0
        self.attempts = 0
        self.calm_time = 0.0
        self.drop_time = 0.0
        self.kick_left = c.kick_s
        self.stop_reason = ""
        self.balance_time = 0.0
        self.catches = 0
        self.rejected_catches = 0
        self.track = None            # trajectory-mode observer state [th1, w1, th2, w2]
        self.track_x0 = 0.0
        self.track_error_deg = 0.0
        self.age_mean = None

    def _set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.mode_time = 0.0
        if mode == "BALANCE":
            self.catches += 1
            self.xi = 0.0
            self.s = None
            self.active = self.catch
        elif mode == "SWING":
            self.attempts += 1
            self.kick_left = self.cfg.kick_s
            self.track = None
        elif mode in ("WAIT", "RECOVER"):
            self.calm_time = 0.0

    def _stop(self, reason, brake=True):
        """Stop: brake the cart to rest with the motor, then disable it.

        Disabling the driver at speed would let the cart coast into the end
        stops, so unless the loop itself is untrustworthy (stall, non-finite
        data) the controller first ramps the velocity command to zero.
        """
        self.stop_reason = reason
        if brake and abs(self.v) > 0.02:
            self.mode = "STOPPING"
            return self._braking(self.Ts)
        self.mode = "STOPPED"
        return ControlOutput(0.0, True, "STOPPED", 0.0, reason)

    def _braking(self, dt):
        dv = min(abs(self.v), self.cfg.stop_brake_accel * dt)
        self.v_prev, self.v = self.v, self.v - math.copysign(dv, self.v)
        if abs(self.v) < 1e-3:
            self.mode = "STOPPED"
            return ControlOutput(0.0, True, "STOPPED", 0.0, self.stop_reason)
        return ControlOutput(self.v, False, "STOPPING", -math.copysign(dv / dt, self.v_prev),
                             self.stop_reason)

    # ------------------------------------------------------------------
    def lqr_state(self, th1, w1, th2, w2, x, v=None):
        v = self.v if v is None else v
        return np.array([wrap(th1), w1 - self.Bq[0] * v, wrap(th2), w2 - self.Bq[1] * v,
                         x, v, self.xi])

    def catch_feasible(self, z):
        """Predict the linear closed loop with the real limits for catch_horizon_s."""
        c = self.cfg
        gs = self.catch
        lim_dv = c.max_accel_balance * gs.Ts
        th_ok = math.radians(c.catch_ok_angle_deg)
        z = z.copy()
        for _ in range(int(c.catch_horizon_s / gs.Ts)):
            dv = clip(-float(gs.K @ z), lim_dv)
            v_new = clip(z[5] + dv, c.max_speed)
            dv = v_new - z[5]
            z = gs.Az @ z + gs.Bz * dv
            if abs(z[0]) > th_ok or abs(z[2]) > th_ok or abs(z[4]) > c.soft_limit_m:
                return False
        return abs(z[0]) < 0.5 * th_ok and abs(z[2]) < 0.5 * th_ok

    def _governor(self, a, x, dt, accel_limit):
        """Limit accel so the cart can always stop inside the soft limits."""
        c = self.cfg
        a = clip(a, accel_limit)
        v_next = self.v + a * dt
        for side in (1.0, -1.0):
            dist = c.soft_limit_m - side * x
            v_allow = math.sqrt(2 * c.brake_accel * max(dist, 0.0))
            if side * v_next > v_allow:
                a = (side * v_allow - self.v) / dt
        a = clip(a, max(accel_limit, c.brake_accel * 1.5))
        v_next = clip(self.v + a * dt, c.max_speed)
        return (v_next - self.v) / dt

    # ------------------------------------------------------------------
    def update(self, obs):
        c = self.cfg
        dt = obs.dt if obs.dt and obs.dt > 0 else self.Ts
        if self.mode == "STOPPED":
            return ControlOutput(0.0, True, "STOPPED", 0.0, self.stop_reason)
        if dt > c.max_cycle_s and self.time > 0:
            return self._stop(f"control loop stalled ({dt * 1000:.0f} ms)", brake=False)
        if self.mode == "STOPPING":
            return self._braking(min(dt, 0.02))
        self.time += dt
        self.mode_time += dt
        valid = obs.theta1 is not None and obs.theta2 is not None
        if valid and not (math.isfinite(obs.theta1) and math.isfinite(obs.theta2)):
            valid = False
        self.missing = 0 if valid else self.missing + 1
        if self.missing > c.max_missing_frames:
            return self._stop(f"{self.missing} consecutive missing angle frames")
        self.f1.update(obs.theta1 if valid else None, dt)
        self.f2.update(obs.theta2 if valid else None, dt)
        if self.f1.angle is None:
            return ControlOutput(0.0, False, self.mode, 0.0, "waiting for first frame")
        th1, w1 = self.f1.angle, self.f1.rate
        th2, w2 = self.f2.angle, self.f2.rate
        # cart position now = Arduino's previous reply + motion since then
        x = obs.cart_m + self.v * dt
        if not math.isfinite(x):
            return self._stop("non-finite cart position", brake=False)
        if abs(x) > c.estop_limit_m:
            return self._stop(f"cart at {x:+.3f} m passed the e-stop limit {c.estop_limit_m} m")
        E = self.energy(th1, w1, th2, w2)
        up = max(abs(wrap(th1)), abs(wrap(th2)))

        if self.mode == "BALANCE":
            self.drop_time = self.drop_time + dt if up > math.radians(c.drop_angle_deg) else 0.0
            if self.drop_time > c.drop_hold_s:
                if c.balance_only:
                    return self._stop(f"balance lost (arms {math.degrees(wrap(th1)):+.1f}, "
                                      f"{math.degrees(wrap(th2)):+.1f} deg)")
                self._set_mode("RECOVER")
            else:
                if (self.active is self.catch and self.mode_time > c.catch_hold_s
                        and up < math.radians(5)):
                    self.active = self.hold
                a = self._balance(obs, valid, x, dt)
                return self._command(a, x, dt, c.max_accel_balance, E)

        if self.mode == "SWING" and self.plan is not None:
            return self._track(obs, valid, th1, th2, x, dt, E)

        if self.mode == "SWING" and up < math.radians(c.catch_angle_deg):
            z = self.lqr_state(th1, w1, th2, w2, x)
            z[6] = 0.0
            if self.catch_feasible(z):
                self._set_mode("BALANCE")
                a = self._balance(obs, valid, x, dt)
                return self._command(a, x, dt, c.max_accel_balance, E)
            self.rejected_catches += 1

        if self.mode == "WAIT":
            a = self._center(x)
            self.calm_time = self.calm_time + dt if E < c.calm_energy_j else 0.0
            if self.calm_time > c.calm_time_s:
                if self.attempts >= c.max_attempts:
                    return self._stop(f"no catch after {self.attempts} swing-up attempts")
                self._set_mode("SWING")
            return self._command(a, x, dt, c.max_accel_swing, E)

        if self.mode == "SWING":
            if self.mode_time > c.swing_timeout_s:
                self._set_mode("RECOVER")
            else:
                a = self._swing(th1, w1, th2, w2, x, E, dt)
                return self._command(a, x, dt, c.max_accel_swing, E)

        if self.mode == "RECOVER":
            if self.mode_time > c.recover_timeout_s:
                return self._stop("recovery timed out (arms did not settle)")
            y = c.h1 * math.cos(th1) * w1 + c.h2 * math.cos(th2) * w2
            # dE/dt = -a*y, so a = +k*y removes energy
            a = c.recover_damp_gain * clip(y, 0.3) + self._center(x)
            calm = E < c.calm_energy_j and abs(x) < 0.03
            self.calm_time = self.calm_time + dt if calm else 0.0
            if self.calm_time > c.calm_time_s:
                self._set_mode("WAIT")
            return self._command(a, x, dt, c.max_accel_swing, E)
        return self._stop(f"unknown mode {self.mode}")

    # ------------------------------------------------------------------
    def _retime(self, obs, w1=None, w2=None):
        """Shift a measurement from its reported age to the running-mean age.

        TX1/TX2 sample at 200 Hz, unsynchronised with the receiver's 10 ms
        slots, so a frame's angles are 0-5 ms old at random. During a fast
        swing (5+ rad/s) that is ~1.5 deg of timing noise; the receiver's
        radio_age field lets us remove the fluctuation. The mean delay stays
        as designed (it is part of the plan/LQR latency model).
        """
        age = getattr(obs, "age_s", None)
        if age is None or not self.cfg.retime_samples:
            return obs.theta1, obs.theta2
        self.age_mean = age if self.age_mean is None else self.age_mean + 0.02 * (age - self.age_mean)
        shift = age - self.age_mean
        w1 = self.f1.rate if w1 is None else w1
        w2 = self.f2.rate if w2 is None else w2
        return obs.theta1 + w1 * shift, obs.theta2 + w2 * shift

    def _track(self, obs, valid, th1_ab, th2_ab, x, dt, E):
        """Follow the planned swing-up with TVLQR feedback."""
        c = self.cfg
        if self.track is None:
            self.track = np.array([th1_ab, 0.0, th2_ab, 0.0])
            self.track_x0 = x
            self.v_prev = self.v
        else:
            s = plan_cycle(self.plan_k, (self.track[0], self.track[1], self.track[2], self.track[3], 0.0),
                           self.v_prev, self.v, dt, min(self.plan_delay, dt))
            self.track = np.array(s[:4])
            if valid:
                m1, m2 = self._retime(obs, self.track[1], self.track[3])
                e1 = wrap(m1 - self.track[0])
                e2 = wrap(m2 - self.track[2])
                self.track += np.array([c.track_L_angle * e1, c.track_L_rate * e1,
                                        c.track_L_angle * e2, c.track_L_rate * e2])
        tau = self.mode_time
        k = min(int(tau / self.plan_Ts), len(self.plan_V) - 1)
        if tau >= self.plan_T:
            # Plan finished near upright. Keep the same observer and switch to the
            # time-invariant LQR that terminates the TVLQR (same model, same
            # state), so the hand-over is seamless; once settled, BALANCE takes
            # over with the Kalman-filter LQR and integral action.
            dz = np.array([wrap(self.track[0]), self.track[1], wrap(self.track[2]), self.track[3],
                           x, self.v])
            up_t = max(abs(dz[0]), abs(dz[2]))
            if up_t > math.radians(c.drop_angle_deg):
                self._set_mode("RECOVER")
                return self._command(self._center(x), x, dt, c.max_accel_swing, E)
            held = tau - self.plan_T
            if held >= c.catch_hold_s and up_t < math.radians(5):
                self._set_mode("BALANCE")
                self.active = self.hold
                self.s = np.array([dz[0], dz[1] - self.Bq[0] * self.v, dz[2], dz[3] - self.Bq[1] * self.v])
                a = self._balance(obs, valid, x, dt)
                return self._command(a, x, dt, c.max_accel_track, E)
            v_target = -float(self.plan_K_final @ dz)
            return self._command((v_target - self.v) / dt, x, dt, c.max_accel_track, E)
        # interpolate the plan in time: the Pi loop period jitters (8-12 ms on
        # the rig), and stepping through the plan by cycle count or by
        # truncated index gives velocity kinks at the most sensitive moments
        u = tau / self.plan_Ts
        k = min(int(u), len(self.plan_V) - 2)
        f = min(max(u - k, 0.0), 1.0)
        zn = (1 - f) * self.plan_Z[k] + f * self.plan_Z[k + 1]
        dz = np.array([wrap(self.track[0] - zn[0]), self.track[1] - zn[1],
                       wrap(self.track[2] - zn[2]), self.track[3] - zn[3],
                       x - self.track_x0 - zn[4], self.v - zn[5]])
        self.track_error_deg = math.degrees(max(abs(dz[0]), abs(dz[2])))
        if self.track_error_deg > c.track_abort_deg:
            self._set_mode("RECOVER")
            return self._command(self._center(x), x, dt, c.max_accel_swing, E)
        v_nom = (1 - f) * self.plan_V[k] + f * self.plan_V[k + 1]
        v_target = v_nom - float(self.plan_K[k] @ dz)
        a = (v_target - self.v) / dt
        return self._command(a, x, dt, c.max_accel_track, E)

    def _center(self, x):
        c = self.cfg
        return -c.swing_center_kp * x - c.swing_center_kd * self.v

    def _swing(self, th1, w1, th2, w2, x, E, dt):
        c = self.cfg
        y = c.h1 * math.cos(th1) * w1 + c.h2 * math.cos(th2) * w2
        target = self.upright_energy * (1 + c.energy_margin)
        # dE/dt = -a*y  ->  a = -k (E* - E) sat(y) drives E toward E*
        a = -c.energy_gain * (target - E) * clip(y / c.pump_sat, 1.0)
        if c.sync_gain:
            # joint response to cart accel: phi_dd = ... - g_phi * a; a ~ +g_phi*phi_dot damps it
            m12 = c.c * math.cos(th1 - th2)
            det = c.J1 * c.J2 - m12 * m12
            h1c, h2c = c.h1 * math.cos(th1), c.h2 * math.cos(th2)
            b1 = (c.J2 * h1c - m12 * h2c) / det
            b2 = (c.J1 * h2c - m12 * h1c) / det
            a += c.sync_gain * (b2 - b1) * (w2 - w1)
        if self.kick_left > 0:
            if E < 0.02:
                a = c.kick_accel if self.kick_left > c.kick_s / 2 else -c.kick_accel
                self.kick_left -= dt
            else:
                self.kick_left = 0.0
        return a + self._center(x)

    def _balance(self, obs, valid, x, dt):
        """Kalman update + LQR; returns cart accel (dv/dt)."""
        gs = self.active
        if self.s is None:
            self.s = np.array([wrap(self.f1.angle), self.f1.rate - self.Bq[0] * self.v,
                               wrap(self.f2.angle), self.f2.rate - self.Bq[1] * self.v])
        else:
            self.s = gs.Ad4 @ self.s + gs.B14 * self.v_prev + gs.B24 * self.v
            if valid:
                m1, m2 = self._retime(obs)
                innov = np.array([wrap(m1 - self.s[0]), wrap(m2 - self.s[2])])
                self.s = self.s + gs.L @ innov
        self.xi = clip(self.xi + dt * x, 0.3)
        z = np.concatenate([self.s, [x, self.v, self.xi]])
        dv = -float(gs.K @ z)
        self.balance_time += dt
        return dv / gs.Ts

    def _command(self, a, x, dt, accel_limit, E):
        a = self._governor(a, x, dt, accel_limit)
        v_new = clip(self.v + a * dt, self.cfg.max_speed)
        self.v_prev, self.v = self.v, v_new
        return ControlOutput(v_new, False, self.mode, a, "", {"E": E})
