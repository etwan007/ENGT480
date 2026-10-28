"""Improved double-pendulum controller: hardware entry point (and --sim).

    engt run experiment/claude-controller-twin Code/DoublePendulum/Improved/PythonMaster2_improved.py
    python3 PythonMaster2_improved.py --sim                 # same controller, digital twin
    python3 PythonMaster2_improved.py --balance-only        # hand-held upright start, no swing-up

Talks to the rig exactly like PythonMaster2_auto.py (it imports that script's
serial helpers): ESP32 receiver protocol v1 on /dev/ttyUSB0 @ 921600,
Arduino on /dev/ttyACM0 @ 115200, 4-byte little-endian timer command per
cycle, 2-byte big-endian step-count reply, 0xFFFFFFFF disables the motor.

Startup (swing-up, default):
  1. Motor power OFF, cart centred, both arms hanging still -> Enter.
     Hanging calibration (same as PythonMaster2_auto.py).
  2. Motor power ON, hands clear -> type GO + Enter. The controller waits for
     the arms to be calm, then swings up, catches and balances.
Startup (--balance-only): as PythonMaster2_auto.py (hold both arms upright).

Ctrl+C, any fault, or a controller e-stop sends the motor-disable command.
Every cycle is logged to Code/DoublePendulum/run_logs/improved_double_*.csv in the standard
DOUBLE_COLUMNS format (Code/RunReview/balancing_visualizer.html can open it).
"""

import argparse
import json
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
CODE_DIRECTORY = HERE.parents[1]
for path in (CODE_DIRECTORY, HERE.parent, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np  # noqa: E402

from improved_controller import ImprovedController, ControllerConfig  # noqa: E402
from stepper_mapping import timer_command_for_velocity, STOP_MOTOR_COMMAND  # noqa: E402

# ---- run settings (first hardware tests: keep these conservative) ----------
GAIN_PROFILE = "conservative"     # hold gains after a catch: "conservative" or "nominal"
SWING_MODE = "trajectory"         # "trajectory" (planned + TVLQR) or "energy"
MAX_SPEED_M_S = 1.0
SOFT_LIMIT_M = 0.28
ESTOP_LIMIT_M = 0.36
MAX_ATTEMPTS = 3
RUN_SECONDS = 120.0               # the script stops the motor after this long
SHOW_STATUS = True
SAVE_RUN_CSV = True
# Same folder as the other double-pendulum controllers: ignored by git and
# backed up to the logs branch by Code/Global/upload_csv_logs.py.
RUN_LOG_DIRECTORY = HERE.parent / "run_logs"
STATUS_INTERVAL_S = 0.5


class Obs:
    __slots__ = ("t", "dt", "cycle", "theta1", "theta2", "cart_m", "raw1", "raw2", "age_s")

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def make_controller(balance_only=False, profile=GAIN_PROFILE, swing_mode=SWING_MODE):
    cfg = ControllerConfig(balance_only=balance_only, swing_mode=swing_mode,
                           max_speed=MAX_SPEED_M_S, soft_limit_m=SOFT_LIMIT_M,
                           estop_limit_m=ESTOP_LIMIT_M, max_attempts=MAX_ATTEMPTS)
    return ImprovedController(config=cfg, profile=profile)


# ---------------------------------------------------------------------------
def run_sim(args):
    sys.path.insert(0, str(CODE_DIRECTORY / "DigitalTwin"))
    from replay import simulate_and_report
    controller = make_controller(args.balance_only, args.profile, args.swing_mode)
    out = simulate_and_report(controller, duration_s=args.duration, seed=args.seed,
                              start="upright" if args.balance_only else "hanging",
                              out_dir=Path(args.out), name=args.name,
                              animate=not args.no_animation)
    print(json.dumps(out["summary"], indent=2))
    return 0


# ---------------------------------------------------------------------------
def run_hardware(args):
    import serial
    import PythonMaster2_auto as base
    from Global.run_csv_logger import DOUBLE_COLUMNS, RunCsvLogger
    from Global.angle_protocol import AnglePacketReader, packet_csv_values
    from Global.angle_packet_fallback import MissingAnglePacketError

    controller = make_controller(args.balance_only, args.profile, args.swing_mode)
    base.angle_reader = AnglePacketReader()
    esp32 = arduino = None
    logger = None
    motor_on = False
    stop_reason = "program exit"
    started = time.monotonic()

    def log(tag, msg):
        print(f"[{time.monotonic() - started:7.2f}s] [{tag}] {msg}", flush=True)

    try:
        esp32 = serial.Serial("/dev/ttyUSB0", 921600, timeout=0.003)
        arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=0.003, write_timeout=0.2)
        log("START", "serial ports open; waiting 3 s for the Arduino")
        time.sleep(3)
        arduino.reset_input_buffer()
        esp32.reset_input_buffer()

        log("READY", "Motor power OFF; centre the cart and let BOTH arms hang still")
        input("Press Enter when both arms have stopped swinging: ")
        base.angle_reader.discard_pending(esp32)
        auto_up1, auto_al2 = base.calibrate_hanging(esp32)
        upright1, aligned2 = base.apply_manual_offsets(auto_up1, auto_al2)

        if SAVE_RUN_CSV:
            name = datetime.now().astimezone().strftime("improved_double_%Y%m%d_%H%M%S_%f.csv")
            logger = RunCsvLogger(RUN_LOG_DIRECTORY / name, columns=DOUBLE_COLUMNS)
            logger.start()
            logger.event("config", json.dumps({
                "schema_version": 2, "angle_protocol_version": 1, "pendulum_count": 2,
                "script": "Improved/PythonMaster2_improved.py",
                "force_columns": "cart acceleration command, m/s^2",
                "controller_config": controller.cfg.__dict__,
                "gain_profile": args.profile, "swing_mode": args.swing_mode,
                "reference_upright_1_counts": float(upright1),
                "reference_aligned_2_counts": float(aligned2),
            }, default=str, separators=(",", ":")), 0.0)
            log("LOG", f"recording to {logger.path}")

        if args.balance_only:
            log("READY", "Turn motor power ON and hold BOTH arms upright")
            input("Press Enter when both arms are upright: ")
            base.angle_reader.discard_pending(esp32)
            base.check_upright(esp32, upright1, aligned2)
        else:
            log("READY", "Turn motor power ON. Keep hands and objects clear of the rail.")
            if input("Type GO and press Enter to start the swing-up: ").strip().upper() != "GO":
                raise RuntimeError("start not confirmed; no motor command was sent")
            base.angle_reader.discard_pending(esp32)

        motor_on = True
        base.send_motor_command(arduino, -65535)        # enables the driver (stopped)
        steps, cart_m = base.read_position(arduino)
        x0 = cart_m                                      # controller works relative to start
        run_t0 = time.monotonic()
        last = None
        cycle = 0
        next_status = run_t0
        if logger:
            logger.event("motor_start", f"initial_cart_m={cart_m}", 0.0)
        while True:
            cycle += 1
            try:
                raw1, raw2, raw3 = base.read_angles(esp32)
            except MissingAnglePacketError:
                raw1 = raw2 = raw3 = None
            now = time.monotonic()
            dt = controller.Ts if last is None else now - last
            last = now
            th1 = th2 = None
            age_s = None
            if raw1 not in (None, 0xFFFF) and raw2 not in (None, 0xFFFF):
                th1, th2, _ = base.model_angles(raw1, raw2, upright1, aligned2)
                packet = base.latest_angle_packet
                if packet is not None:
                    # receiver radio age per channel (ms); used to re-time the sample
                    age_s = 0.5 * (packet.ages_ms[0] + packet.ages_ms[1]) / 1000.0
            obs = Obs(t=now - run_t0, dt=dt, cycle=cycle, theta1=th1, theta2=th2,
                      cart_m=cart_m - x0, raw1=raw1, raw2=raw2, age_s=age_s)
            out = controller.update(obs)
            if out.stop:
                stop_reason = f"controller stop: {out.reason}"
                break
            if now - run_t0 > RUN_SECONDS:
                stop_reason = f"run time limit {RUN_SECONDS} s"
                break
            step_hz, timer = timer_command_for_velocity(out.velocity)
            base.send_motor_command(arduino, timer)
            steps, cart_m = base.read_position(arduino)
            if logger:
                j = None if th1 is None else float(np.angle(np.exp(1j * (th2 - th1))))
                row = [""] * len(DOUBLE_COLUMNS)
                vals = {"record_type": "sample", "elapsed_s": now - run_t0, "cycle": cycle,
                        "esp32_packet": base.esp32_packet_count,
                        "raw_angle_1": raw1, "raw_angle_2": raw2, "raw_angle_3": raw3,
                        "reference_upright_1_counts": upright1, "reference_aligned_2_counts": aligned2,
                        "measured_angle_1_rad": th1, "measured_angle_2_rad": th2,
                        "measured_angle_1_deg": None if th1 is None else np.degrees(th1),
                        "measured_angle_2_deg": None if th2 is None else np.degrees(th2),
                        "joint_angle_2_rad": j, "joint_angle_2_deg": None if j is None else np.degrees(j),
                        "cart_used_m": obs.cart_m, "cart_steps": steps, "cart_m": cart_m - x0,
                        "force_command_n": out.accel, "cart_velocity_command_m_s": out.velocity,
                        "step_frequency_hz": step_hz, "motor_timer_command": timer,
                        "loop_ms": (time.monotonic() - now) * 1000, "cycle_period_ms": dt * 1000,
                        "message": out.mode}
                idx = {c: i for i, c in enumerate(DOUBLE_COLUMNS)}
                for k, v in vals.items():
                    row[idx[k]] = "" if v is None else v
                tel = packet_csv_values(base.latest_angle_packet, base.angle_reader)
                start = idx["esp32_sequence"]
                row[start:start + len(tel)] = tel
                logger.sample(tuple(row))
            if SHOW_STATUS and now >= next_status:
                log("RUN", f"{out.mode:8s} th1={np.degrees(th1 or 0):+7.1f} th2={np.degrees(th2 or 0):+7.1f} "
                           f"x={cart_m - x0:+.3f} v={out.velocity:+.3f} attempts={controller.attempts}")
                next_status = now + STATUS_INTERVAL_S
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
    except Exception as exc:  # noqa: BLE001 - always stop the motor, then report
        stop_reason = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        if arduino is not None and arduino.is_open:
            if motor_on:
                try:
                    base.send_motor_command(arduino, STOP_MOTOR_COMMAND)
                    arduino.flush()
                    log("STOP", "sent motor-disable command")
                except Exception as exc:  # noqa: BLE001
                    log("FAULT", f"could not send stop command: {exc}")
            arduino.close()
        if esp32 is not None and esp32.is_open:
            esp32.close()
        log("STOP", stop_reason)
        if logger:
            warning = logger.stop(stop_reason)
            log("STOP", warning or f"CSV saved: {logger.path}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="run against the digital twin instead of the rig")
    ap.add_argument("--balance-only", action="store_true", help="hand-held upright start, no swing-up")
    ap.add_argument("--profile", default=GAIN_PROFILE, choices=("conservative", "nominal", "catch"))
    ap.add_argument("--swing-mode", default=SWING_MODE, choices=("trajectory", "energy"))
    ap.add_argument("--duration", type=float, default=20.0, help="--sim: seconds to simulate")
    ap.add_argument("--seed", type=int, default=0, help="--sim: noise/timing seed")
    ap.add_argument("--out", default=str(CODE_DIRECTORY / "DigitalTwin" / "run_logs"),
                    help="--sim: output folder for plots, CSV and animation")
    ap.add_argument("--name", default=None, help="--sim: output file prefix")
    ap.add_argument("--no-animation", action="store_true")
    args = ap.parse_args(argv)
    return run_sim(args) if args.sim else run_hardware(args)


if __name__ == "__main__":
    sys.exit(main())
