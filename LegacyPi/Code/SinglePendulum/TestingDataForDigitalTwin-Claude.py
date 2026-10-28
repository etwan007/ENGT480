#!/usr/bin/env python3
"""PendulumSysID.py - system identification runs for the ENGT480 cart-pendulum rig.

Drives the cart through a randomized set of tests and records everything the Pi can see,
so a simulation can be matched to the real rig. Every run gets its own folder:

    run_logs/sysid_<date>_<time>_<plan>_seed<seed>/

with one file per kind of data (see columns.txt in each folder). The tests' frequencies,
amplitudes, speeds and order come from a random seed that is saved with the run, so each
run covers different conditions and any run can be repeated exactly with --seed.

Tests (the plan decides which, and how many):
  still      arm(s) hanging, motor power off, then on: sensor noise, hanging zero, timing
  moves      point-to-point cart moves at random distances, speeds and accelerations
  pump/decay the cart swings the arm up to a random amplitude, stops, and records the free decay
             (natural frequency vs amplitude, viscous / dry / air friction)
  chirp      a log frequency sweep of the cart
  dwell      steady sine shaking at random frequencies (frequency response points)
  multisine  several frequencies at once with random phases
  random     smooth random cart motion (broadband)
  mode decay (double) shaking at each natural frequency in turn, then the free decay
Between tests the cart damps the swing and returns toward the centre.

Run it from the folder for the rig you are using. It needs only numpy and pyserial:
    cd Code/SinglePendulum            (or Code/DoublePendulum)
    python3 PendulumSysID.py                  full plan, about 5 minutes
    python3 PendulumSysID.py --plan quick     about 1.5 minutes
    python3 PendulumSysID.py --plan decay     mostly free swings: friction and inertia
    python3 PendulumSysID.py --plan excite    chirp, dwells, multisine and random motion
    python3 PendulumSysID.py --plan motor     many cart moves at different speeds and accelerations
    python3 PendulumSysID.py --seed 123456    repeat an earlier run's random choices
Keys while it runs: n = skip to the next test, q = finish early (cart goes home, files saved).
Ctrl+C stops at once (the cart brakes, files are saved).

Then, on any computer with numpy, scipy and matplotlib:
    python3 analyze_sysid.py run_logs/sysid_...      (one run, or several folders at once)
"""
import argparse
import atexit
import binascii
import csv
import hashlib
import io
import json
import math
import os
import platform
import queue
import random
import select
import shutil
import struct
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import serial

# =============================== SETTINGS ===============================
ARMS = None              # 1 or 2. None: 2 when this file is in a folder named DoublePendulum, else 1
PLAN = "full"            # full | quick | decay | excite | motor   (--plan overrides)
SEED = None              # None: new random tests every run. The seed is saved; reuse it with --seed

# Safety. Centre the cart before starting; positions are measured from where it starts.
TRACK_LIMIT_M = 0.4     # motor is disabled at once if the cart gets this far from the start
PLAN_LIMIT_M = 0.25      # tests keep the cart inside this; it brakes before reaching it
SPEED_MAX_M_S = 0.8      # fastest cart speed ever commanded
ACCEL_MAX_M_S2 = 8.0     # hardest cart acceleration ever commanded
MOTOR_TEST_ACCEL_MAX_M_S2 = 12.0   # the 'motor' plan tries accelerations up to this (still <= SPEED_MAX)
EXCITE_SWING_LIMIT_DEG = {1: 60.0, 2: 35.0}   # shaking tests stop and damp the swing past this
PUMP_SWING_MAX_DEG = {1: 150.0, 2: 40.0}      # largest swing the pump-and-decay tests aim for
DECAY_SECONDS_MAX = 25.0  # longest free decay recorded per test
DECAY_STOP_DEG = 0.7      # ... or stop once the swing is smaller than this

ESP32_PORT, ESP32_BAUD = "/dev/ttyUSB0", 921600
ARDUINO_PORT, ARDUINO_BAUD = "/dev/ttyACM0", 115200
M_PER_STEP = 0.638175 / 6400.0   # cart travel per step, as in the repo's scripts

# Things a scale and a ruler can tell you. Fill in once (None = not measured). They are saved with
# every run, and analyze_sysid.py uses them to turn the fitted dynamics into masses, centres of
# mass and inertias for the simulation.
RIG = {
    "arm1_mass_kg": None,           # everything that swings with arm 1: rod, weights, TX1 board, battery, screws
    "arm1_pivot_to_com_m": None,    # take the arm off and balance it on a knife edge; distance from the pivot axis
    "arm1_pivot_to_tip_m": None,
    "arm1_pivot_to_joint_m": None,  # double: pivot axis to arm 2's joint axis
    "arm2_mass_kg": None,           # double: everything that swings with arm 2
    "arm2_joint_to_com_m": None,
    "arm2_joint_to_tip_m": None,
    "cart_mass_kg": None,           # cart and what rides on it, without the arms
    "notes": "",
}
# ========================================================================

SETTING_NAMES = ("ARMS", "PLAN", "SEED", "TRACK_LIMIT_M", "PLAN_LIMIT_M", "SPEED_MAX_M_S", "ACCEL_MAX_M_S2",
                 "MOTOR_TEST_ACCEL_MAX_M_S2", "EXCITE_SWING_LIMIT_DEG", "PUMP_SWING_MAX_DEG", "DECAY_SECONDS_MAX",
                 "DECAY_STOP_DEG", "ESP32_PORT", "ESP32_BAUD", "ARDUINO_PORT", "ARDUINO_BAUD", "M_PER_STEP")
SCRIPT_VERSION = "1.0"
ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
RAD_PER_COUNT = 2.0 * math.pi / ENCODER_COUNTS
TIMER_STEP_HZ = 125000.0          # step rate = 125000 / (ICR3 + 1) on the Mega (16 MHz / 64 / 2)
SLOW_TIMER = 65535                # the repo's "almost stopped" command: about 1.9 steps/s
G = 9.81
# Receiver protocol v1 (Code/Wireless/ESP32_Code/Receivermaster/angle_protocol.h)
MAGIC = b"\xa5\x5a\xc3\x3c"
FRAME_SIZE = 37
STATUS_NAMES = ("valid", "encoder_read_failed", "wireless_timeout", "never_received")
# Nominal arm values (PythonMasterMath2.py), used only until the run measures the real ones
NOMINAL = {"w0_single": 7.94, "k_single": 6.43, "p_ratio_double": 0.371, "f_modes_double": (0.89, 1.90)}

HERE = Path(__file__).resolve().parent
started = time.monotonic()


def now():
    return time.monotonic() - started


def wrap_counts(c):
    return (c + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def circular_mean_counts(values):
    ph = np.asarray(values, dtype=float) * RAD_PER_COUNT
    return (math.atan2(float(np.mean(np.sin(ph))), float(np.mean(np.cos(ph)))) / RAD_PER_COUNT) % ENCODER_COUNTS


def smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def sgn(v):
    return 1.0 if v > 0 else -1.0 if v < 0 else 0.0


# ------------------------------------------------------------------ files
class Writer:
    """Writes every file from a background thread, so a slow SD card never delays the control loop."""

    def __init__(self):
        self.q = queue.Queue()
        self.files = {}
        self.thread = threading.Thread(target=self._run, name="sysid-writer", daemon=True)
        self.thread.start()
        self.closed = False

    def open(self, key, path, binary=False):
        self.q.put(("open", key, (str(path), binary)))

    def write(self, key, data):
        if not self.closed:
            self.q.put(("write", key, data))

    def flush(self):
        self.q.put(("flush", None, None))

    def _run(self):
        while True:
            op, key, data = self.q.get()
            try:
                if op == "open":
                    path, binary = data
                    self.files[key] = open(path, "wb" if binary else "w", newline="" if not binary else None,
                                           **({} if binary else {"encoding": "utf-8"}))
                elif op == "write":
                    self.files[key].write(data)
                elif op == "flush":
                    for f in self.files.values():
                        f.flush()
                elif op == "close":
                    for f in self.files.values():
                        f.close()
                    return
            except Exception as exc:          # keep writing the other files
                sys.__stderr__.write(f"[sysid writer] {op} {key}: {exc}\n")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.q.put(("close", None, None))
        self.thread.join(timeout=30)


class CsvFile:
    def __init__(self, writer, key, path, header):
        self.w, self.key = writer, key
        writer.open(key, path)
        self._emit(header)

    def _emit(self, values):
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerow(values)
        self.w.write(self.key, buf.getvalue())

    def row(self, values):
        self._emit(["" if v is None else (f"{v:.7g}" if isinstance(v, float) else v) for v in values])


class RawCapture:
    """Every serial byte with the time it was read or written, so the frames can be decoded again later.
    Record: <d t_s><B channel><H length><bytes>; channel 1 = ESP32 in, 2 = Arduino in, 3 = Arduino out."""
    ESP_IN, ARD_IN, ARD_OUT = 1, 2, 3

    def __init__(self, writer, path):
        self.w = writer
        writer.open("raw", path, binary=True)
        writer.write("raw", b"ENGT480-SYSID-RAW1\n")

    def log(self, channel, t, data):
        if data:
            self.w.write("raw", struct.pack("<dBH", t, channel, len(data)) + bytes(data))


COLUMNS = {
    "samples.csv": [
        ("t_s", "Pi time of the control cycle, from the start of the script (time.monotonic)"),
        ("cycle", "control cycle number"),
        ("seg", "test number (see segments.csv)"), ("test", "test name"), ("phase", "what the test was doing"),
        ("cycle_ms", "time since the previous cycle"), ("wait_ms", "time spent waiting for an angle frame"),
        ("frames", "receiver frames read this cycle (normally 1)"),
        ("rx_seq", "newest receiver frame sequence number"), ("rx_ms", "newest receiver frame time, receiver clock"),
        ("raw1", "TX1 encoder counts (0-16383), blank if not valid"), ("age1_ms", "age of that reading at the receiver"),
        ("raw2", "TX2 encoder counts"), ("age2_ms", ""),
        ("psi1_deg", "arm 1 angle from hanging, + in the encoder's count-up direction"),
        ("rate1_deg_s", "arm 1 angular speed (filtered difference)"),
        ("psi2_deg", "double: arm 2 absolute angle from hanging (arm 1 + joint)"),
        ("joint_deg", "double: arm 2 relative to arm 1"), ("rate2_deg_s", "double: arm 2 absolute angular speed"),
        ("amp1_deg", "largest |psi1| in the last 1.3 s"), ("amp2_deg", "largest |psi2| in the last 1.3 s"),
        ("amp_energy_deg", "single: swing amplitude from the energy (angle and speed)"),
        ("x_ref_m", "cart position the test wanted"), ("v_ref_m_s", ""), ("a_ref_m_s2", ""),
        ("a_cmd_m_s2", "cart acceleration used this cycle (after limits)"),
        ("v_req_m_s", "cart speed requested"), ("timer_cmd", "value sent to the Arduino (Timer3 ICR3, sign = direction)"),
        ("v_cmd_m_s", "cart speed that timer value really gives"), ("step_hz", "steps per second of that timer value, + = count up"),
        ("t_write_s", "Pi time the command was written"), ("reply_ms", "write-to-reply time"),
        ("steps", "Arduino step count from the reply (unwrapped)"),
        ("x_steps_m", "(steps - step count at the first reply) x m_per_step: cart position from where it started"),
        ("x_pred_m", "cart position estimated at the start of the cycle"),
        ("scale", "excitation amplitude scale (drops when a swing gets too big)"),
        ("flags", "a_clip, v_clip, brake, stale_angle, ..."),
    ],
    "angle_frames.csv": [
        ("t_host_s", "Pi time the frame's bytes were read"), ("seq", "receiver sequence number"),
        ("rx_ms", "receiver uptime when it sent the frame"),
        ("raw1", ""), ("status1", "valid / encoder_read_failed / wireless_timeout / never_received"),
        ("age1_ms", "ms since the receiver got this TX1 reading (sample time = rx_ms - age)"),
        ("raw2", ""), ("status2", ""), ("age2_ms", ""), ("raw3", ""), ("status3", ""), ("age3_ms", ""),
        ("gap", "frames missing just before this one (sequence jump - 1)"),
        ("crc_errors", "running total"), ("discarded_bytes", "running total"),
    ],
    "motor_commands.csv": [
        ("t_write_s", ""), ("timer_cmd", "int32 sent (little-endian); -1 disables the motor"),
        ("v_cmd_m_s", "speed that value gives (+ = step count up)"), ("t_reply_s", ""), ("reply_ms", ""),
        ("steps_raw", "int16 from the Arduino"), ("steps", "unwrapped"), ("extra_bytes", "unexpected bytes after the reply"),
        ("note", ""),
    ],
    "events.csv": [("t_s", ""), ("seg", ""), ("test", ""), ("kind", ""), ("text", "")],
    "segments.csv": [("seg", ""), ("test", ""), ("kind", ""), ("t_start_s", ""), ("t_end_s", ""),
                     ("status", "done / skipped / stopped"), ("params", "JSON: the random choices"),
                     ("results", "JSON: what the test measured on the fly")],
}
SAMPLE_HEADER = [c for c, _ in COLUMNS["samples.csv"]]


# ------------------------------------------------------------------ hardware
class ArduinoSilent(RuntimeError):
    pass


class Frames:
    """Receiver protocol v1 reader that keeps every frame (not just the newest)."""

    def __init__(self, port, raw, csvf):
        self.port, self.raw, self.csv = port, raw, csvf
        self.buf = bytearray()
        self.crc_errors = self.discarded = self.gaps = self.duplicates = self.restarts = 0
        self.count = 0
        self.last = None

    def _parse(self, t):
        out = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                keep = len(MAGIC) - 1
                if len(self.buf) > keep:
                    self.discarded += len(self.buf) - keep
                    del self.buf[:len(self.buf) - keep]
                break
            if i:
                self.discarded += i
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            if self.buf[4:6] != b"\x01\x1d":
                self.discarded += 1
                del self.buf[:1]
                continue
            if len(self.buf) < FRAME_SIZE:
                break
            fr = bytes(self.buf[:FRAME_SIZE])
            if binascii.crc_hqx(fr[4:-2], 0xFFFF) != int.from_bytes(fr[-2:], "big"):
                self.crc_errors += 1
                self.discarded += 1
                del self.buf[:1]
                continue
            del self.buf[:FRAME_SIZE]
            seq, rx_ms = struct.unpack_from(">II", fr, 6)
            ch = [struct.unpack_from(">HBI", fr, 14 + 7 * k) for k in range(3)]
            gap = 0
            if self.last is not None:
                d = (seq - self.last[0]) & 0xFFFFFFFF
                dt = (rx_ms - self.last[1]) & 0xFFFFFFFF
                if dt >= 0x80000000:
                    self.restarts += 1
                elif d == 0 or d >= 0x80000000:
                    self.duplicates += 1
                    continue
                elif d > 1:
                    gap = d - 1
                    self.gaps += gap
            self.last = (seq, rx_ms)
            self.count += 1
            f = {"t": t, "seq": seq, "rx_ms": rx_ms, "gap": gap,
                 "raw": [c[0] if c[1] == 0 and c[0] < ENCODER_COUNTS else None for c in ch],
                 "status": [c[1] for c in ch], "age": [c[2] for c in ch]}
            out.append(f)
            self.csv.row([f"{t:.6f}", seq, rx_ms,
                          ch[0][0], STATUS_NAMES[ch[0][1]] if ch[0][1] < 4 else ch[0][1], ch[0][2],
                          ch[1][0], STATUS_NAMES[ch[1][1]] if ch[1][1] < 4 else ch[1][1], ch[1][2],
                          ch[2][0], STATUS_NAMES[ch[2][1]] if ch[2][1] < 4 else ch[2][1], ch[2][2],
                          gap, self.crc_errors, self.discarded])
        return out

    def resync(self):
        """After a prompt the input buffer is stale (and may have overflowed): drop it without counting gaps."""
        self.port.reset_input_buffer()
        self.buf = bytearray()
        self.last = None

    def poll(self, wait_s):
        """Read everything waiting. If no new frame yet, wait up to wait_s for one."""
        deadline = time.monotonic() + wait_s
        frames = []
        while True:
            n = int(self.port.in_waiting)
            if n:
                data = self.port.read(min(n, 8192))
            elif frames or time.monotonic() >= deadline:
                break
            else:
                data = self.port.read(1)          # waits at most the port timeout (3 ms)
            if data:
                t = now()
                self.raw.log(RawCapture.ESP_IN, t, data)
                self.buf.extend(data)
                frames += self._parse(t)
        return frames


class Arduino:
    """ArduinoSlave.ino: each 4-byte int32 sets Timer3 (step rate and direction) and gets the int16 step count back."""

    def __init__(self, port, raw, csvf):
        self.port, self.raw, self.csv = port, raw, csvf
        self.steps = None
        self.steps_raw = None
        self.zero = None          # step count at the first reply: the counter drifts while the motor is off
        self.t_reply = None
        self.v_sent = 0.0
        self.sent_any = False
        self.disabled = False

    @staticmethod
    def timer_for(v, toward=0.0):
        """Timer value for cart speed v (m/s, + = step count up). Returns (timer, real speed, step Hz)."""
        f = -v / M_PER_STEP                      # repo convention: negative step Hz counts up
        if abs(f) > 2.0:
            icr = max(1, int(round(TIMER_STEP_HZ / abs(f))) - 1)
            icr = min(icr, SLOW_TIMER)
            timer = icr if f > 0 else -icr
        else:
            # Firmware cannot stop the timer without disabling the motor: creep at 1.9 steps/s toward 'toward'
            timer = SLOW_TIMER if toward < 0 else -SLOW_TIMER
        hz = math.copysign(TIMER_STEP_HZ / (abs(timer) + 1), timer)
        return timer, -hz * M_PER_STEP, -hz

    def command(self, timer, note=""):
        data = struct.pack("<i", int(timer))
        t_w = now()
        self.port.write(data)
        self.raw.log(RawCapture.ARD_OUT, t_w, data)
        self.sent_any = True
        buf = bytearray()
        deadline = time.monotonic() + 0.1
        while len(buf) < 2 and time.monotonic() < deadline:
            chunk = self.port.read(2 - len(buf))
            if chunk:
                self.raw.log(RawCapture.ARD_IN, now(), chunk)
                buf.extend(chunk)
        if len(buf) < 2:
            self.csv.row([f"{t_w:.6f}", int(timer), "", "", "", "", "", "", "no reply " + note])
            raise ArduinoSilent("the Arduino stopped replying (limit switch or stop button pressed, "
                                "or the USB cable). The motor is off; reset the Arduino before the next run.")
        t_r = now()
        extra = b""
        if int(self.port.in_waiting):
            extra = self.port.read(int(self.port.in_waiting))
            self.raw.log(RawCapture.ARD_IN, now(), extra)
        raw = struct.unpack(">h", bytes(buf))[0]
        if self.steps is None:
            self.steps = raw
        else:
            self.steps += ((raw - self.steps_raw + 32768) & 0xFFFF) - 32768
        self.steps_raw = raw
        if self.zero is None:
            self.zero = self.steps
        self.t_reply = t_r
        if int(timer) == -1:
            v, self.disabled = 0.0, True
        else:
            v = -math.copysign(TIMER_STEP_HZ / (abs(int(timer)) + 1), int(timer)) * M_PER_STEP
        self.v_sent = v
        self.csv.row([f"{t_w:.6f}", int(timer), v, f"{t_r:.6f}", (t_r - t_w) * 1000.0, raw, self.steps,
                      len(extra), note])
        return t_w, t_r

    def position(self, t=None):
        if self.steps is None:
            return 0.0
        x = (self.steps - self.zero) * M_PER_STEP
        if t is not None and self.t_reply is not None:
            x += self.v_sent * max(0.0, t - self.t_reply)
        return x


class Keys:
    """n = next test, q = finish. Ctrl+C still works."""

    def __init__(self):
        self.fd = None
        self.saved = None
        try:
            if sys.stdin.isatty():
                import termios
                import tty
                self.fd = sys.stdin.fileno()
                self.saved = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd, termios.TCSANOW)
        except Exception:
            self.fd = None

    def poll(self):
        if self.fd is None:
            return ""
        try:
            if select.select([self.fd], [], [], 0)[0]:
                return os.read(self.fd, 32).decode("ascii", "ignore").lower()
        except Exception:
            pass
        return ""

    def close(self):
        if self.saved is not None:
            try:
                import termios
                termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            except Exception:
                pass
            self.saved = None


# ------------------------------------------------------------------ the tests
class Ctx:
    """What a test can see each cycle."""
    pass


class Test:
    kind = "test"

    def __init__(self, name, params):
        self.name, self.p = name, params
        self.phase = "run"
        self.done = False
        self.result = {}
        self.t0 = 0.0

    def begin(self, c):
        pass

    def step(self, c):
        """Return (a_cmd, x_ref, v_ref, a_ref)."""
        raise NotImplementedError


def hold_accel(c, x_hold):
    return 30.0 * (x_hold - c.x) - 12.0 * c.v


def coupling_signal(c):
    """Rate of energy flow out of the arm(s) per unit cart acceleration, up to a positive factor."""
    s = c.model["k_sign"]
    if c.arms == 1:
        return s * math.cos(c.psi[0]) * c.rate[0]
    r = c.model["p_ratio"]
    return s * (math.cos(c.psi[0]) * c.rate[0] + r * math.cos(c.psi[1]) * c.rate[1])


def damp_accel(c, x_center=0.0):
    """Take swing energy out with the cart, and drift back to x_center."""
    if not c.model.get("sign_known"):
        return 30.0 * (c.x0_hold - c.x) - 12.0 * c.v      # direction not measured yet: stand still instead
    c.damping = True
    y = coupling_signal(c)
    gain = 0.12 if c.arms == 1 else 0.10          # m/s^2 per (rad/m * rad/s)
    a = float(np.clip(gain * abs(c.model["k"]) * y, -4.0, 4.0))
    a += -2.25 * (c.x - x_center) - 3.0 * c.v
    return a


class Still(Test):
    kind = "still"

    def step(self, c):
        if c.ts >= self.p["seconds"]:
            self.done = True
        return hold_accel(c, self.x_hold), self.x_hold, 0.0, 0.0

    def begin(self, c):
        self.x_hold = c.x


class Settle(Test):
    """Active damping until the arm(s) hang still, or a time limit."""
    kind = "settle"

    def begin(self, c):
        self.calm_since = None

    def step(self, c):
        calm = max(c.amp[:c.arms]) < self.p.get("calm_deg", 1.5)
        if calm and abs(c.x) < 0.01 and abs(c.v) < 0.02:
            if self.calm_since is None:
                self.calm_since = c.ts
            if c.ts - self.calm_since > 0.8:
                self.done = True
        else:
            self.calm_since = None
        if c.ts > self.p.get("timeout_s", 20.0):
            self.done = True
            self.result["timed_out"] = True
        self.result["seconds"] = round(c.ts, 3)
        return damp_accel(c), 0.0, 0.0, 0.0


class Moves(Test):
    """Trapezoidal point-to-point moves; the swings they leave are recorded during the pauses."""
    kind = "moves"

    def begin(self, c):
        self.i = 0
        self.state = "pause"
        self.state_t = c.ts
        self.pause = self.p["moves"][0]["pause_s"] * 0.3 if self.p["moves"] else 0.0
        self.x_hold = c.x
        self.prof = None
        self.result["moves"] = []

    def _profile(self, xs, mv):
        d = mv["to_m"] - xs
        D, s = abs(d), sgn(d)
        a, v = mv["accel_m_s2"], mv["speed_m_s"]
        ta = v / a
        if a * ta * ta > D:                    # never reaches full speed
            ta = math.sqrt(D / a)
            v = a * ta
        tf = (D - a * ta * ta) / v if v > 0 else 0.0
        return {"xs": xs, "s": s, "a": a, "v": v, "ta": ta, "tf": tf, "T": 2 * ta + tf}

    @staticmethod
    def _ref(pr, t):
        a, v, ta, tf, s, xs = pr["a"], pr["v"], pr["ta"], pr["tf"], pr["s"], pr["xs"]
        if t <= 0:
            return xs, 0.0, 0.0
        if t < ta:
            return xs + s * 0.5 * a * t * t, s * a * t, s * a
        d1 = 0.5 * a * ta * ta
        if t < ta + tf:
            return xs + s * (d1 + v * (t - ta)), s * v, 0.0
        t3 = t - ta - tf
        if t3 < ta:
            return xs + s * (d1 + v * tf + v * t3 - 0.5 * a * t3 * t3), s * (v - a * t3), -s * a
        return xs + s * (2 * d1 + v * tf), 0.0, 0.0

    def step(self, c):
        lim = self.p["swing_wait_deg"]
        if self.state == "pause":
            self.phase = "pause"
            if c.ts - self.state_t >= self.pause:
                if max(c.amp[:c.arms]) > lim:
                    self.state, self.state_t = "damp", c.ts
                elif self.i >= len(self.p["moves"]):
                    self.done = True
                else:
                    mv = self.p["moves"][self.i]
                    self.prof = self._profile(c.x, mv)
                    self.state, self.state_t = "move", c.ts
                    self.result["moves"].append({"t_start_s": round(c.t, 4), "from_m": round(c.x, 5),
                                                 "to_m": mv["to_m"], "peak_speed_m_s": round(self.prof["v"], 4),
                                                 "accel_m_s2": mv["accel_m_s2"]})
                    c.event("move", f"move {self.i + 1}/{len(self.p['moves'])}: {c.x:+.3f} -> {mv['to_m']:+.3f} m, "
                                    f"peak {self.prof['v']:.2f} m/s, {mv['accel_m_s2']:.1f} m/s^2")
            return hold_accel(c, self.x_hold), self.x_hold, 0.0, 0.0
        if self.state == "damp":
            self.phase = "damp"
            if max(c.amp[:c.arms]) < lim * 0.5 or c.ts - self.state_t > 12.0:
                self.state, self.state_t, self.pause = "pause", c.ts, 0.3
                self.x_hold = c.x
            return damp_accel(c, x_center=self.x_hold), self.x_hold, 0.0, 0.0
        self.phase = "move"
        t = c.ts - self.state_t
        xr, vr, ar = self._ref(self.prof, t)
        if t >= self.prof["T"]:
            self.x_hold = xr
            self.pause = self.p["moves"][self.i]["pause_s"]
            self.i += 1
            self.state, self.state_t = "pause", c.ts
        return ar + 60.0 * (xr - c.x) + 15.0 * (vr - c.v), xr, vr, ar


def fade(tau, T, ramp):
    if tau <= 0.0 or tau >= T:
        return 0.0
    w = 1.0
    if tau < ramp:
        w *= 0.5 - 0.5 * math.cos(math.pi * tau / ramp)
    if tau > T - ramp:
        w *= 0.5 - 0.5 * math.cos(math.pi * (T - tau) / ramp)
    return w


class Excite(Test):
    """Cart follows a designed position signal; if the swing gets too big it pauses to damp it."""
    kind = "excite"

    def begin(self, c):
        self.tau = 0.0                 # signal time (does not run while damping)
        self.env_t = 0.0               # fade-in time since the last (re)start
        self.scale = 1.0
        self.x_start = c.x
        self.state = "run"
        self.state_t = c.ts
        self.result.update({"pauses": 0, "scale_end": 1.0})
        self.comp = self.p.get("components")
        if self.p["signal"] == "dwell":
            self.plan_dwell(c)

    def plan_dwell(self, c):
        """Frequency and amplitude from the natural frequencies measured earlier in this run."""
        p, m = self.p, c.model
        if c.arms == 1:
            f_ref, k = m["w0"] / (2 * math.pi), abs(m["k"])
        else:
            f_ref, k = m["f_modes"][p["mode"]], abs(m["k"]) * (0.8 if p["mode"] == 0 else 0.3)
        f = f_ref * p["ratio"]
        w, w0 = 2 * math.pi * f, 2 * math.pi * f_ref
        den = abs(complex(w0 * w0 - w * w, 0.03 * w0 * w))     # assumes light damping
        a_amp = math.radians(p["swing_deg"]) * den / max(k, 0.3)
        a_amp = min(a_amp, 1.2, 0.65 * PLAN_LIMIT_M * w * w, 0.3 * w)
        p.update(f_hz=round(f, 5), a_amp_m_s2=round(a_amp, 4), x_amp_m=round(a_amp / (w * w), 6))
        self.result.update(f_hz=p["f_hz"], a_amp_m_s2=p["a_amp_m_s2"])

    def signal(self, tau):
        p = self.p
        typ = p["signal"]
        if typ == "chirp":
            fa, fb, T = p["f_start_hz"], p["f_end_hz"], p["seconds"]
            r = fb / fa
            f = fa * r ** (tau / T)
            ph = 2 * math.pi * fa * T / math.log(r) * (r ** (tau / T) - 1.0)
            w = 2 * math.pi * f
            X = min(p["x_amp_m"], p["v_amp_m_s"] / w, p["a_amp_m_s2"] / (w * w))
            return X * math.sin(ph)
        if typ in ("multisine", "random"):
            return sum(X * math.sin(2 * math.pi * f * tau + ph) for f, X, ph in self.comp)
        if typ == "dwell":
            return p["x_amp_m"] * math.sin(2 * math.pi * p["f_hz"] * tau)
        raise ValueError(typ)

    def xref(self, tau, env_t):
        T = self.p["seconds"]
        ramp = self.p.get("ramp_s", 1.0)
        env = fade(tau, T, ramp) if env_t >= ramp else fade(tau, T, ramp) * (0.5 - 0.5 * math.cos(math.pi * env_t / ramp))
        x_c = self.x_start * (1.0 - smoothstep(env_t / 2.0))
        return x_c + self.scale * env * self.signal(tau)

    def step(self, c):
        lim = c.excite_limit
        if self.state == "damp":
            self.phase = "damp"
            if (max(c.amp[:c.arms]) < 0.45 * lim and abs(c.v) < 0.05) or c.ts - self.state_t > 15.0:
                self.state = "run"
                self.env_t = 0.0
                self.x_start = c.x
                c.event("resume", f"{self.name}: resumes at {self.scale:.2f} of the planned amplitude")
            return damp_accel(c), 0.0, 0.0, 0.0
        self.phase = "run"
        if max(c.amp[:c.arms]) > lim:
            self.state, self.state_t = "damp", c.ts
            self.scale *= 0.6
            self.result["pauses"] += 1
            self.result["scale_end"] = round(self.scale, 3)
            c.event("governor", f"{self.name}: swing {max(c.amp[:c.arms]):.0f} deg > {lim:.0f}; damping, then "
                                f"continuing at {self.scale:.2f} of the planned amplitude")
            return damp_accel(c), 0.0, 0.0, 0.0
        h = 0.002
        tau, et = self.tau, self.env_t
        x0 = self.xref(tau, et)
        xp = self.xref(tau + h, et + h)
        xm = self.xref(max(tau - h, 0.0), max(et - h, 0.0))
        vr = (xp - xm) / (2 * h)
        ar = (xp - 2 * x0 + xm) / (h * h)
        self.tau += c.dt
        self.env_t += c.dt
        if self.tau >= self.p["seconds"]:
            self.done = True
        return ar + 60.0 * (x0 - c.x) + 15.0 * (vr - c.v), x0, vr, ar


class PumpDecay(Test):
    """Swing the arm up to a target amplitude with the cart, stop, and record the free decay."""
    kind = "pump_decay"

    def begin(self, c):
        self.state, self.state_t = "kick", c.ts
        self.x_hold = c.x
        self.peaks = []
        self.result.update({"target_deg": self.p["target_deg"]})

    def step(self, c):
        p = self.p
        st = c.ts - self.state_t
        amp_now = c.amp_energy if c.arms == 1 else c.amp[0]
        if self.state == "kick":
            self.phase = "kick"
            # a short wiggle that ends at rest gets a still arm swinging
            if amp_now > 3.0 or st >= 0.6:
                self.state, self.state_t = "pump", c.ts
            k = int(st // 0.15)
            a = {0: 2.5, 1: -2.5, 2: -2.5, 3: 2.5}.get(k, 0.0)
            return a + 3.0 * (-c.v * (k >= 4)), c.x, c.v, a
        if self.state == "pump":
            self.phase = "pump"
            if c.arms == 2 and c.amp[1] > PUMP_SWING_MAX_DEG[2] + 15.0:
                reached = True
            else:
                reached = amp_now >= p["target_deg"]
            if reached or st > p["pump_timeout_s"]:
                self.result["reached_deg"] = round(amp_now, 2)
                self.result["pump_s"] = round(st, 3)
                if not reached:
                    c.event("pump", f"{self.name}: reached {amp_now:.1f} deg of {p['target_deg']:.0f} in "
                                    f"{st:.0f} s; recording the decay from there")
                self.state, self.state_t = "brake", c.ts
                return self.step(c)
            y = coupling_signal(c)
            if c.arms == 1:
                w0 = c.model["w0"]
                e = 0.5 * (c.rate[0] / w0) ** 2 + 1.0 - math.cos(c.psi[0])
                et = 1.0 - math.cos(math.radians(p["target_deg"]))
                pump = float(np.clip(4.0 * (et - e) / max(et, 0.05), -1.0, 1.0))
                shape = 1.0 / (1.0 + (c.rate[0] * math.cos(c.psi[0]) / w0) ** 2)
            else:
                pump = 1.0
                shape = 1.0 / (1.0 + (y / 4.0) ** 2)
            a = -p["accel_m_s2"] * pump * sgn(y)
            a += (-12.0 * c.x - 6.0 * c.v) * shape
            return float(np.clip(a, -p["accel_m_s2"] * 1.2, p["accel_m_s2"] * 1.2)), c.x, c.v, a
        if self.state == "brake":
            self.phase = "brake"
            if abs(c.v) < 0.003:
                self.state, self.state_t = "decay", c.ts
                self.x_hold = c.x
                self.result["decay_start_s"] = round(c.t, 4)
                self.result["decay_start_deg"] = round(amp_now, 2)
                c.event("decay", f"{self.name}: free decay from {amp_now:.1f} deg (cart held at {c.x:+.3f} m)")
            a = -sgn(c.v) * min(ACCEL_MAX_M_S2, abs(c.v) / max(c.dt, 1e-3))
            return a, c.x, c.v, a
        self.phase = "decay"
        if st >= p["decay_s"] or (st > 3.0 and max(c.amp[:c.arms]) < DECAY_STOP_DEG):
            self.result["decay_s"] = round(st, 3)
            self.result["decay_end_deg"] = round(max(c.amp[:c.arms]), 3)
            self.done = True
        return hold_accel(c, self.x_hold), self.x_hold, 0.0, 0.0


class ModeDecay(Test):
    """Double: shake at one of the two natural frequencies, stop, record the free decay.
    Mode 0 (both arms together) is judged by the larger arm swing, mode 1 by the joint swing."""
    kind = "mode_decay"

    def begin(self, c):
        self.state, self.state_t = "shake", c.ts
        self.x_hold = c.x
        self.x_start = c.x
        self.f = c.model["f_modes"][self.p["mode"]]
        self.result["f_hz"] = round(self.f, 4)

    def size(self, c):
        return max(c.amp[0], c.amp[1]) if self.p["mode"] == 0 else c.amp_joint

    def step(self, c):
        p = self.p
        st = c.ts - self.state_t
        if self.state == "shake":
            self.phase = "shake"
            if self.size(c) >= p["target_deg"] or max(c.amp[:2]) > c.excite_limit or st > p["shake_s"]:
                self.result["reached_deg"] = round(self.size(c), 2)
                self.result["shake_s"] = round(st, 3)
                self.state, self.state_t = "brake", c.ts
                return self.step(c)
            w = 2 * math.pi * self.f
            env = smoothstep(st / 1.5)
            X = min(p["a_amp_m_s2"] / (w * w), 0.5 * PLAN_LIMIT_M)
            xc = self.x_start * (1.0 - smoothstep(st / 2.0))
            dxc = -self.x_start * 6.0 * (st / 2.0) * (1 - st / 2.0) / 2.0 if st < 2.0 else 0.0
            xr = xc + env * X * math.sin(w * st)
            vr = dxc + env * X * w * math.cos(w * st)
            ar = -env * X * w * w * math.sin(w * st)
            return ar + 60.0 * (xr - c.x) + 15.0 * (vr - c.v), xr, vr, ar
        if self.state == "brake":
            self.phase = "brake"
            if abs(c.v) < 0.003:
                self.state, self.state_t = "decay", c.ts
                self.x_hold = c.x
                self.result["decay_start_s"] = round(c.t, 4)
                self.result["decay_start_deg"] = round(self.size(c), 2)
                c.event("decay", f"{self.name}: free decay, {'arm' if p['mode'] == 0 else 'joint'} swing {self.size(c):.1f} deg")
            a = -sgn(c.v) * min(ACCEL_MAX_M_S2, abs(c.v) / max(c.dt, 1e-3))
            return a, c.x, c.v, a
        self.phase = "decay"
        if st >= p["decay_s"] or (st > 2.0 and max(c.amp[0], c.amp[1], c.amp_joint) < DECAY_STOP_DEG):
            self.result["decay_s"] = round(st, 3)
            self.result["decay_end_deg"] = round(self.size(c), 3)
            self.done = True
        return hold_accel(c, self.x_hold), self.x_hold, 0.0, 0.0


class Home(Moves):
    kind = "home"


TEST_CLASSES = {"still": Still, "settle": Settle, "moves": Moves, "excite": Excite,
                "pump_decay": PumpDecay, "mode_decay": ModeDecay, "home": Home}


# ------------------------------------------------------------------ plans
def make_plan(plan, arms, rng, model):
    """The list of tests, with every random choice written into its parameters."""
    L = PLAN_LIMIT_M
    xa = 0.65 * L                            # largest planned excitation amplitude
    tests = []

    def add(name, kind, **p):
        tests.append({"name": name, "kind": kind, "params": p})

    def moves(n, vmax, amax, wait_deg):
        out = []
        x = 0.0
        for _ in range(n):
            for _ in range(50):
                to = rng.uniform(-0.8 * L, 0.8 * L)
                if abs(to - x) >= 0.03:
                    break
            out.append({"to_m": round(to, 4), "speed_m_s": round(rng.uniform(0.12, vmax), 3),
                        "accel_m_s2": round(rng.uniform(1.0, amax), 2), "pause_s": round(rng.uniform(0.8, 2.2), 2)})
            x = to
        return {"moves": out, "swing_wait_deg": wait_deg}

    def settle():
        add("settle", "settle", calm_deg=1.5 if arms == 1 else 2.0, timeout_s=20.0)

    def pump(i, lo, hi):
        tgt = round(rng.uniform(lo, hi), 1)
        acc = round(min(ACCEL_MAX_M_S2, (1.5 + 3.5 * tgt / 150.0) if arms == 1 else 2.5) * rng.uniform(0.8, 1.1), 2)
        add(f"pump_decay_{i}", "pump_decay", target_deg=tgt, accel_m_s2=acc, pump_timeout_s=25.0,
            decay_s=round(rng.uniform(0.8, 1.0) * DECAY_SECONDS_MAX, 1))
        settle()

    def chirp(seconds):
        up = rng.random() < 0.5
        fa, fb = rng.uniform(0.25, 0.45), rng.uniform(3.0, 5.0)
        add("chirp", "excite", signal="chirp", f_start_hz=round(fa if up else fb, 3), f_end_hz=round(fb if up else fa, 3),
            seconds=round(seconds, 1), x_amp_m=round(xa, 3), v_amp_m_s=round(rng.uniform(0.15, 0.3), 3),
            a_amp_m_s2=round(rng.uniform(0.25, 0.6) if arms == 1 else rng.uniform(0.2, 0.45), 3), ramp_s=1.0)
        settle()

    def comps(nf, f_lo, f_hi, log_spaced):
        if log_spaced:
            fs = np.exp(np.linspace(math.log(f_lo), math.log(f_hi), nf))
            fs = fs * np.exp(np.array([rng.uniform(-0.08, 0.08) for _ in range(nf)]))
        else:
            fs = np.array(sorted(rng.uniform(f_lo, f_hi) for _ in range(nf)))
        accel = np.array([rng.uniform(0.5, 1.0) for _ in range(nf)])
        X = accel / (2 * np.pi * fs) ** 2
        ph = [rng.uniform(0, 2 * math.pi) for _ in range(nf)]
        t = np.linspace(0, 60, 6001)
        xs = sum(Xi * np.sin(2 * np.pi * f * t + p) for f, Xi, p in zip(fs, X, ph))
        acs = sum(-Xi * (2 * np.pi * f) ** 2 * np.sin(2 * np.pi * f * t + p) for f, Xi, p in zip(fs, X, ph))
        a_pk = rng.uniform(0.35, 0.7) if arms == 1 else rng.uniform(0.25, 0.5)
        k = min(xa / np.max(np.abs(xs)), a_pk / np.max(np.abs(acs)))
        return [[round(float(f), 5), float(Xi * k), round(p, 5)] for f, Xi, p in zip(fs, X, ph)]

    def dwells(n):
        # frequency = ratio x a natural frequency measured during the run (mode 0 = slowest)
        if arms == 1:
            bands = [(0, 0.35, 0.75), (0, 0.75, 0.9), (0, 1.12, 1.4), (0, 1.5, 3.0), (0, 0.5, 0.85), (0, 1.2, 2.2)]
        else:
            bands = [(0, 0.4, 0.8), (0, 1.2, 1.6), (1, 0.8, 0.92), (1, 1.1, 1.4), (0, 1.7, 2.0), (1, 0.6, 0.75)]
        chosen = bands[:n]
        rng.shuffle(chosen)
        for i, (mode, lo, hi) in enumerate(chosen):
            add(f"dwell_{i + 1}", "excite", signal="dwell", mode=mode, ratio=round(rng.uniform(lo, hi), 4),
                swing_deg=round(rng.uniform(8, 20) if arms == 1 else rng.uniform(4, 9), 2),
                seconds=round(rng.uniform(9, 13), 1), ramp_s=1.0)
            settle()

    vmax_moves = min(SPEED_MAX_M_S, 0.7 if arms == 1 else 0.35)
    wait = 40.0 if arms == 1 else 20.0
    add("still_off", "still", seconds=5.0, motor="off")
    add("still_on", "still", seconds=3.0, motor="on")
    if plan == "motor":
        mv = moves(14, SPEED_MAX_M_S, min(MOTOR_TEST_ACCEL_MAX_M_S2, 20.0), wait)
        for m in mv["moves"]:
            m["pause_s"] = round(rng.uniform(0.5, 1.5), 2)
        add("moves", "moves", **mv)
    else:
        add("moves", "moves", **moves(6 if plan == "full" else 3 if plan == "quick" else 4,
                                      vmax_moves, min(ACCEL_MAX_M_S2, 6.0), wait))
    settle()
    hi = PUMP_SWING_MAX_DEG[arms]
    if plan in ("full", "decay", "quick"):
        if arms == 1:
            bands = [(15, 35), (45, 80), (90, hi)] if plan == "full" else \
                    [(10, 25), (25, 50), (50, 80), (80, 115), (115, hi)] if plan == "decay" else [(60, 100)]
            rng.shuffle(bands)
            for i, (lo, h) in enumerate(bands):
                pump(i + 1, lo, h)
        else:
            # each mode on its own; swings under 20 deg keep the two modes from trading energy
            runs = [(0, 6, 14), (1, 8, 16), (0, 22, hi)] if plan == "full" else \
                   [(0, 5, 10), (0, 10, 18), (1, 6, 12), (1, 12, 20), (0, 25, hi)] if plan == "decay" else [(0, 10, 18)]
            rng.shuffle(runs)
            for i, (mode, lo, h) in enumerate(runs):
                add(f"mode{mode + 1}_decay_{i + 1}", "mode_decay", mode=mode, target_deg=round(rng.uniform(lo, h), 1),
                    a_amp_m_s2=round(rng.uniform(0.3, 0.8) if mode == 0 else rng.uniform(0.6, 1.4), 3), shake_s=15.0,
                    decay_s=round(rng.uniform(0.7, 0.9) * DECAY_SECONDS_MAX, 1))
                settle()
    if plan in ("full", "excite", "quick"):
        chirp(rng.uniform(28, 40) if plan != "quick" else 20.0)
    if plan in ("full", "excite"):
        dwells(4 if plan == "full" else 6)
        add("multisine", "excite", signal="multisine", seconds=round(rng.uniform(25, 35), 1), ramp_s=1.5,
            components=comps(rng.randint(6, 9), 0.25, 4.0, True))
        settle()
        add("random", "excite", signal="random", seconds=round(rng.uniform(20, 30), 1), ramp_s=1.5,
            components=comps(25, 0.2, 3.5, False))
        settle()
    add("home", "home", moves=[{"to_m": 0.0, "speed_m_s": 0.15, "accel_m_s2": 1.0, "pause_s": 1.5}],
        swing_wait_deg=wait)
    add("still_end", "still", seconds=2.0, motor="on")
    return tests


# ------------------------------------------------------------------ quick model fits
def sg_kernel(window, order, deriv, dt):
    half = window // 2
    x = np.arange(-half, half + 1) * dt
    A = np.vander(x, order + 1, increasing=True)
    return np.linalg.pinv(A)[deriv] * math.factorial(deriv)


def sg_apply(y, kern):
    half = len(kern) // 2
    yp = np.concatenate([np.full(half, y[0]), y, np.full(half, y[-1])])
    return np.convolve(yp, kern[::-1], mode="valid")


def quick_identify(hist, arms, t0, t1, min_samples=200):
    """Fit psi'' = f(psi, psi', cart accel) to the 'moves' test. Returns a dict or None."""
    t = np.array(hist["t"])
    sel = (t >= t0) & (t <= t1)
    if sel.sum() < min_samples:
        return None
    t = t[sel]
    grid = np.arange(t[0] + 0.1, t[-1] - 0.1, 0.01)
    if len(grid) < 0.75 * min_samples:
        return None
    dt = 0.01
    k0, k1, k2 = (sg_kernel(15, 4, d, dt) for d in (0, 1, 2))
    p1 = np.interp(grid, t, np.unwrap(np.array(hist["psi1"])[sel]))
    v = np.interp(grid, np.array(hist["t_write"])[sel], np.array(hist["v_cmd"])[sel])
    acc = sg_apply(v, k1)
    th, thd, thdd = sg_apply(p1, k0), sg_apply(p1, k1), sg_apply(p1, k2)
    m = slice(10, -10)
    out = {}
    if arms == 1:
        A = np.column_stack([np.sin(th), np.cos(th) * acc, thd])[m]
        b = thdd[m]
        coef, *_ = np.linalg.lstsq(A, b, rcond=None)
        pred = A @ coef
        r2 = 1.0 - np.sum((b - pred) ** 2) / max(np.sum((b - b.mean()) ** 2), 1e-12)
        w0sq, k, c = -coef[0], -coef[1], -coef[2]
        out.update(r2=float(r2), w0=float(math.sqrt(w0sq)) if w0sq > 0 else None, k=float(k), damping_1_s=float(c))
        out["ok"] = bool(r2 > 0.6 and out["w0"] and 3.0 < out["w0"] < 25.0 and abs(k) > 0.5)
    else:
        p2 = np.interp(grid, t, np.unwrap(np.array(hist["psi2"])[sel]))
        th2, thd2, thdd2 = sg_apply(p2, k0), sg_apply(p2, k1), sg_apply(p2, k2)
        A = np.column_stack([th, th2, acc, thd, thd2])[m]
        c1, *_ = np.linalg.lstsq(A, thdd[m], rcond=None)
        c2, *_ = np.linalg.lstsq(A, thdd2[m], rcond=None)
        r2s = []
        for cc, b in ((c1, thdd[m]), (c2, thdd2[m])):
            pred = A @ cc
            r2s.append(1.0 - np.sum((b - pred) ** 2) / max(np.sum((b - b.mean()) ** 2), 1e-12))
        Am = np.array([[c1[0], c1[1]], [c2[0], c2[1]]])
        ev = np.linalg.eigvals(-Am)
        ws = sorted(math.sqrt(e.real) for e in ev if e.real > 0)
        out.update(r2=[float(r) for r in r2s], k=float(-c1[2]), k2=float(-c2[2]),
                   f_modes=[w / (2 * math.pi) for w in ws])
        out["ok"] = bool(min(r2s) > 0.5 and len(ws) == 2 and abs(out["k"]) > 0.3)
    return out


def zero_cross_w0(hist, t0, t1):
    t = np.array(hist["t"])
    sel = (t >= t0) & (t <= t1)
    a = np.unwrap(np.array(hist["psi1"])[sel])
    t = t[sel]
    if len(a) < 100:
        return None
    a = a - np.mean(a)
    cr = [t[i - 1] + a[i - 1] / (a[i - 1] - a[i]) * (t[i] - t[i - 1])
          for i in range(1, len(a)) if a[i - 1] * a[i] < 0]
    if len(cr) < 5:
        return None
    T = 2 * float(np.median(np.diff(cr)))
    return 2 * math.pi / T if 0.3 < T < 4 else None


# ------------------------------------------------------------------ the run
class Run:
    def __init__(self, args):
        self.args = args
        self.arms = args.arms
        self.plan_name = args.plan
        self.seed = args.seed
        self.rng = random.Random(self.seed)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.dir = HERE / "run_logs" / f"sysid_{stamp}_{'double' if self.arms == 2 else 'single'}_{self.plan_name}_seed{self.seed}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.w = Writer()
        self.raw = RawCapture(self.w, self.dir / "serial_raw.bin")
        self.w.open("term", self.dir / "terminal.log")
        self.samples = CsvFile(self.w, "samples", self.dir / "samples.csv", SAMPLE_HEADER)
        self.frames_csv = CsvFile(self.w, "frames", self.dir / "angle_frames.csv", [c for c, _ in COLUMNS["angle_frames.csv"]])
        self.motor_csv = CsvFile(self.w, "motor", self.dir / "motor_commands.csv", [c for c, _ in COLUMNS["motor_commands.csv"]])
        self.events_csv = CsvFile(self.w, "events", self.dir / "events.csv", [c for c, _ in COLUMNS["events.csv"]])
        self.seg_csv = CsvFile(self.w, "segs", self.dir / "segments.csv", [c for c, _ in COLUMNS["segments.csv"]])
        with open(self.dir / "columns.txt", "w", encoding="utf-8") as f:
            f.write(columns_text())
        try:
            shutil.copy2(__file__, self.dir / "PendulumSysID_used.py")
        except Exception:
            pass
        self.meta = {"script": "PendulumSysID.py", "script_version": SCRIPT_VERSION,
                     "started_local": datetime.now().astimezone().isoformat(timespec="seconds"),
                     "arms": self.arms, "plan": self.plan_name, "seed": self.seed,
                     "settings": {k: globals()[k] for k in SETTING_NAMES},
                     "rig_measurements": RIG, "nominal_model": NOMINAL, "host": host_info(),
                     "angle_convention": "psi = angle from hanging, + in the direction TX1's counts go up; "
                                         "double: psi2 = psi1 + joint (TX2 counts)",
                     "cart_convention": "x = step count x m_per_step from where the cart started; + = count up",
                     "firmware": "ArduinoSlave.ino: int32 timer value in, int16 step count back; "
                                 "step Hz = 125000/(|value|+1); value > 0 counts down; -1 disables the motor"}
        self.events = []
        self.seg_index = -1
        self.seg_name = "setup"
        self.stop_reason = "finished"
        self.model = {"w0": NOMINAL["w0_single"], "k": NOMINAL["k_single"], "k_sign": 1.0,
                      "p_ratio": NOMINAL["p_ratio_double"], "f_modes": list(NOMINAL["f_modes_double"]),
                      "source": "nominal (PythonMasterMath2.py values)"}
        self.hist = {"t": [], "psi1": [], "psi2": [], "t_write": [], "v_cmd": []}
        self.keys = None
        self.esp = self.ard = None
        self.frames = self.arduino = None
        self.cycle = 0
        self.hang = [None, None]
        self.angle = {"psi": [0.0, 0.0], "rate": [0.0, 0.0], "t_samp": [None, None], "ok": False,
                      "t_last": None, "raw": [None, None, None], "age": [None, None, None]}
        self.amp_hist = [[], [], []]
        self.v_cmd = 0.0
        self.guard = None
        self.current = None
        self.last_sign_try = 0.0
        self.results = {"tests": []}

    # ---------------- output
    def say(self, tag, msg):
        line = f"[{now():7.2f}s] [{tag}] {msg}"
        print(line, flush=True)
        self.w.write("term", line + "\n")

    def event(self, kind, text):
        t = now()
        self.events.append({"t_s": round(t, 4), "seg": self.seg_index, "test": self.seg_name, "kind": kind, "text": text})
        self.events_csv.row([f"{t:.6f}", self.seg_index, self.seg_name, kind, text])
        self.say(kind.upper()[:8], text)

    def ask(self, prompt):
        self.w.write("term", prompt + "\n")
        try:
            ans = input(prompt)
        except EOFError:
            ans = ""
        self.w.write("term", f"> {ans}\n")
        return ans.strip()

    # ---------------- angles
    def take_frames(self, frames):
        a = self.angle
        for f in frames:
            for i in range(3):
                a["raw"][i] = f["raw"][i]
                a["age"][i] = f["age"][i]
            if self.hang[0] is None:
                continue
            ok = f["raw"][0] is not None and (self.arms == 1 or f["raw"][1] is not None)
            if not ok:
                continue
            p1 = wrap_counts(f["raw"][0] - self.hang[0]) * RAD_PER_COUNT
            vals = [p1, 0.0]
            if self.arms == 2:
                vals[1] = p1 + wrap_counts(f["raw"][1] - self.hang[1]) * RAD_PER_COUNT
            for i in range(self.arms):
                ts = (f["rx_ms"] - f["age"][i]) / 1000.0
                prev, tp = a["psi"][i], a["t_samp"][i]
                if tp is not None and ts > tp:
                    d = (vals[i] - prev + math.pi) % (2 * math.pi) - math.pi
                    r = d / (ts - tp)
                    a["rate"][i] = 0.3 * a["rate"][i] + 0.7 * r
                elif tp is not None and ts < tp - 1.0:
                    a["rate"][i] = 0.0               # receiver restarted
                a["psi"][i] = vals[i]
                a["t_samp"][i] = ts
            a["ok"] = True
            a["t_last"] = f["t"]
            tnow = f["t"]
            amps = [abs(math.degrees(a["psi"][0])), abs(math.degrees(a["psi"][1])),
                    abs(math.degrees(a["psi"][1] - a["psi"][0]))]
            for i in range(3):
                h = self.amp_hist[i]
                h.append((tnow, amps[i]))
                while h and h[0][0] < tnow - 1.3:
                    h.pop(0)

    def amp(self, i):
        h = self.amp_hist[i]
        return max(v for _, v in h) if h else 0.0

    def amp_energy(self):
        a = self.angle
        w0 = self.model["w0"]
        e = 0.5 * (a["rate"][0] / w0) ** 2 + 1.0 - math.cos(a["psi"][0])
        return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - e))))

    # ---------------- setup
    def open_ports(self):
        self.say("START", f"opening ESP32 receiver {ESP32_PORT} at {ESP32_BAUD}")
        self.esp = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
        self.say("START", f"opening Arduino {ARDUINO_PORT} at {ARDUINO_BAUD}")
        self.ard = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
        self.frames = Frames(self.esp, self.raw, self.frames_csv)
        self.arduino = Arduino(self.ard, self.raw, self.motor_csv)
        self.say("START", "resetting the Arduino (step count starts at 0 where the cart is now)")
        self.ard.dtr = False
        time.sleep(0.1)
        self.ard.dtr = True
        time.sleep(3.0)
        self.ard.reset_input_buffer()
        self.esp.reset_input_buffer()

    def record_still(self, seconds, label):
        """Motor off: just read frames, nothing commanded."""
        t_end = now() + seconds
        raw1, raw2 = [], []
        while now() < t_end:
            fr = self.frames.poll(0.02)
            for f in fr:
                if f["raw"][0] is not None:
                    raw1.append(f["raw"][0])
                if f["raw"][1] is not None:
                    raw2.append(f["raw"][1])
            self.take_frames(fr)
        return raw1, raw2

    def calibrate(self, raw1, raw2):
        if len(raw1) < 50:
            raise RuntimeError(f"only {len(raw1)} valid TX1 readings in the still test: check TX1 and the receiver")
        self.hang[0] = circular_mean_counts(raw1)
        dev1 = np.abs(wrap_counts(np.asarray(raw1) - self.hang[0]))
        cal = {"hang1_counts": round(self.hang[0], 3), "tx1_valid": len(raw1),
               "tx1_noise_std_counts": round(float(np.std(wrap_counts(np.asarray(raw1) - self.hang[0]))), 4),
               "tx1_max_dev_counts": round(float(dev1.max()), 2)}
        if self.arms == 2:
            if len(raw2) < 50:
                raise RuntimeError(f"only {len(raw2)} valid TX2 readings: check TX2")
            self.hang[1] = circular_mean_counts(raw2)
            dev2 = np.abs(wrap_counts(np.asarray(raw2) - self.hang[1]))
            cal.update({"aligned2_counts": round(self.hang[1], 3), "tx2_valid": len(raw2),
                        "tx2_noise_std_counts": round(float(np.std(wrap_counts(np.asarray(raw2) - self.hang[1]))), 4),
                        "tx2_max_dev_counts": round(float(dev2.max()), 2)})
        elif raw2:
            cal["tx2_present_counts"] = round(circular_mean_counts(raw2), 3)
        self.meta["calibration"] = cal
        self.say("CAL", "hanging reference: " + ", ".join(f"{k}={v}" for k, v in cal.items()))
        return max(dev1.max(), 0.0)

    # ---------------- one control cycle
    def cycle_once(self, test, seg_t0):
        self.cycle += 1
        t_c0 = now()
        frames = self.frames.poll(0.015)
        t_c = now()
        wait_ms = (t_c - t_c0) * 1000.0
        self.take_frames(frames)
        a = self.angle
        stale = a["t_last"] is None or t_c - a["t_last"] > 0.25
        dt = 0.01 if self.last_cycle_t is None else min(max(t_c - self.last_cycle_t, 0.002), 0.03)
        cyc_ms = None if self.last_cycle_t is None else (t_c - self.last_cycle_t) * 1000.0
        self.last_cycle_t = t_c
        x = self.arduino.position(t_c)
        if abs(x) > TRACK_LIMIT_M:
            self.arduino.command(-1, note="track limit")
            raise RuntimeError(f"cart at {x:+.3f} m passed TRACK_LIMIT_M={TRACK_LIMIT_M}: motor disabled")
        c = self.ctx
        c.t, c.ts, c.dt, c.x, c.v = t_c, t_c - seg_t0, dt, x, self.v_cmd
        c.psi, c.rate = a["psi"], a["rate"]
        c.amp = [self.amp(0), self.amp(1)]
        c.amp_joint = self.amp(2)
        c.amp_energy = self.amp_energy() if self.arms == 1 else c.amp[0]
        c.stale = stale
        if not self.model.get("sign_known"):
            if getattr(c, "x0_hold", None) is None or abs(self.v_cmd) > 0.01:
                c.x0_hold = x
        else:
            c.x0_hold = x
        flags = []
        if stale:
            flags.append("stale_angle")
            if t_c - (a["t_last"] or 0.0) > 2.0:
                raise RuntimeError("no angle frames for 2 s: check the receiver and transmitters")
            a_des, xr, vr, ar = hold_accel(c, x), x, 0.0, 0.0
            a_des = -sgn(self.v_cmd) * min(ACCEL_MAX_M_S2, abs(self.v_cmd) / dt) if abs(self.v_cmd) > 0.01 else a_des
        else:
            c.damping = False
            a_des, xr, vr, ar = test.step(c)
            self.sign_guard(c)
        # limits: acceleration, speed, and a braking zone before PLAN_LIMIT_M
        amax = ACCEL_MAX_M_S2
        if test.kind == "moves" and self.plan_name == "motor":
            amax = max(ACCEL_MAX_M_S2, MOTOR_TEST_ACCEL_MAX_M_S2)
        a_cmd = a_des
        if abs(a_cmd) > amax:
            a_cmd = math.copysign(amax, a_cmd)
            flags.append("a_clip")
        v_new = self.v_cmd + a_cmd * dt
        stop_d = v_new * v_new / (2.0 * 0.8 * ACCEL_MAX_M_S2)
        if v_new != 0 and abs(x) > 0 and sgn(v_new) == sgn(x) and abs(x) + stop_d > PLAN_LIMIT_M:
            v_brake = self.v_cmd - sgn(self.v_cmd) * ACCEL_MAX_M_S2 * dt
            if sgn(v_brake) != sgn(self.v_cmd):
                v_brake = 0.0
            if abs(v_brake) < abs(v_new):
                v_new = v_brake
                flags.append("brake")
            if abs(x) > PLAN_LIMIT_M and sgn(v_new) == sgn(x):
                v_new = 0.0
        if abs(v_new) > SPEED_MAX_M_S:
            v_new = math.copysign(SPEED_MAX_M_S, v_new)
            flags.append("v_clip")
        a_used = (v_new - self.v_cmd) / dt
        timer, v_real, hz = Arduino.timer_for(v_new, toward=(xr - x) if abs(v_new) < 1e-4 else v_new)
        t_w, t_r = self.arduino.command(timer)
        self.v_cmd = v_new if abs(v_new) >= 1e-4 else 0.0
        steps = self.arduino.steps
        last = frames[-1] if frames else None
        deg = math.degrees
        self.samples.row([
            f"{t_c:.6f}", self.cycle, self.seg_index, self.seg_name, test.phase,
            cyc_ms, wait_ms, len(frames),
            last["seq"] if last else None, last["rx_ms"] if last else None,
            a["raw"][0], a["age"][0], a["raw"][1], a["age"][1],
            deg(a["psi"][0]), deg(a["rate"][0]),
            deg(a["psi"][1]) if self.arms == 2 else None,
            deg(a["psi"][1] - a["psi"][0]) if self.arms == 2 else None,
            deg(a["rate"][1]) if self.arms == 2 else None,
            c.amp[0], c.amp[1] if self.arms == 2 else None, c.amp_energy if self.arms == 1 else None,
            xr, vr, ar, a_used, v_new, timer, v_real, hz,
            f"{t_w:.6f}", (t_r - t_w) * 1000.0, steps, (steps - self.arduino.zero) * M_PER_STEP, x,
            getattr(test, "scale", None), "|".join(flags),
        ])
        self.hist["t"].append(t_c)
        self.hist["psi1"].append(a["psi"][0])
        self.hist["psi2"].append(a["psi"][1])
        self.hist["t_write"].append(t_w)
        self.hist["v_cmd"].append(v_real)
        return c

    def early_sign(self, t_seg):
        """After the first move: which way does the cart push the arm? Needed before any damping."""
        self.last_sign_try = now()
        try:
            q = quick_identify(self.hist, self.arms, t_seg, now(), min_samples=120)
        except Exception:
            q = None
        if q and q["ok"]:
            self.model.update(k=q["k"], k_sign=sgn(q["k"]), sign_known=True)
            if self.arms == 1 and q.get("w0"):
                self.model["w0"] = q["w0"]
            self.event("model", f"cart pushes the arm with coupling {q['k']:+.2f} (sign {'+' if q['k'] > 0 else '-'}); "
                                f"damping enabled")

    def sign_guard(self, c):
        """If damping makes the swing grow, the cart pushes the arm the other way than assumed: flip it."""
        amp = max(c.amp[:self.arms])
        if not c.damping:
            self.guard = None
            return
        if self.guard is None:
            self.guard = (c.t, amp)
            return
        t0, a0 = self.guard
        if c.t - t0 > 1.0:
            if a0 > 2.0 and amp > 1.3 * a0:
                self.model["k_sign"] = -self.model["k_sign"]
                self.model["k"] = -self.model["k"]
                self.event("sign", f"damping made the swing grow ({a0:.1f} -> {amp:.1f} deg): reversing the assumed "
                                   f"direction of the cart's push on the arm")
            self.guard = (c.t, amp)

    # ---------------- main
    def run(self):
        self.say("START", f"PendulumSysID {SCRIPT_VERSION}: {'double' if self.arms == 2 else 'single'} pendulum, "
                          f"plan '{self.plan_name}', seed {self.seed}")
        self.say("START", f"saving to {self.dir}")
        self.tests = make_plan(self.plan_name, self.arms, self.rng, self.model)
        self.meta["plan_tests"] = self.tests
        self.save_meta()
        note = self.ask("Optional note for this run (Enter to skip): ")
        if note:
            self.meta["run_note"] = note
        self.say("READY", "Turn motor power OFF. Centre the cart on the track and let the arm(s) hang still.")
        self.ask("With motor power OFF and the arm(s) still, press Enter: ")
        self.open_ports()
        self.keys = Keys()
        self.ctx = Ctx()
        self.ctx.arms = self.arms
        self.ctx.model = self.model
        self.ctx.event = self.event
        self.ctx.excite_limit = EXCITE_SWING_LIMIT_DEG[self.arms]
        self.ctx.x0_hold = None
        self.last_cycle_t = None

        # ---- still, motor off (no commands at all)
        spec = self.tests[0]
        self.begin_segment(0, spec)
        t0 = now()
        raw1, raw2 = self.record_still(spec["params"]["seconds"], "still_off")
        dev = self.calibrate(raw1, raw2)
        tries = 0
        while dev > 60 and tries < 3:
            tries += 1
            self.event("not_still", f"arm moved {dev:.0f} counts while calibrating; waiting for it to settle")
            raw1, raw2 = self.record_still(4.0, "still_off")
            dev = self.calibrate(raw1, raw2)
        self.end_segment(spec, t0, "done", self.meta["calibration"])

        self.say("READY", "Turn motor power ON now. Keep hands clear: the cart will move by itself for several minutes.")
        self.say("READY", "Keys: n = skip to the next test, q = finish early (cart goes home). Ctrl+C stops at once.")
        self.ask("Turn motor power ON (arms hanging freely), then press Enter: ")
        self.frames.resync()
        self.event("resync", "angle bytes that queued up during the prompt were dropped")
        self.arduino.command(-SLOW_TIMER, note="first command (enables the motor)")
        self.meta["step_zero"] = self.arduino.zero
        self.say("RUN", f"Arduino replied: step count {self.arduino.steps} (taken as the cart's start position)")

        quit_requested = False
        for i, spec in enumerate(self.tests[1:], start=1):
            if quit_requested and spec["kind"] not in ("home", "still"):
                self.seg_csv.row([i, spec["name"], spec["kind"], "", "", "skipped", json.dumps(spec["params"]), ""])
                continue
            test = TEST_CLASSES[spec["kind"]](spec["name"], spec["params"])
            self.begin_segment(i, spec)
            t_seg = now()
            self.current = (spec, t_seg, test)
            c = self.cycle_once_ctx_init(t_seg)
            test.begin(c)
            status = "done"
            last_print = now()
            while not test.done:
                c = self.cycle_once(test, t_seg)
                key = self.keys.poll() if self.keys else ""
                if "n" in key and spec["kind"] not in ("home",):
                    status = "skipped"
                    self.event("key", f"n pressed: skipping the rest of {spec['name']}")
                    break
                if "q" in key:
                    quit_requested = True
                    status = "stopped"
                    self.event("key", "q pressed: finishing early (cart goes home)")
                    break
                if now() - last_print >= 1.0:
                    last_print = now()
                    self.status_line(test, c)
                if (spec["kind"] == "moves" and not self.model.get("sign_known") and getattr(test, "i", 0) >= 1
                        and test.state == "pause" and c.ts - test.state_t > 0.6 and now() - self.last_sign_try > 0.5):
                    self.early_sign(t_seg)
            res = dict(test.result)
            if spec["kind"] == "moves":
                res.update(self.after_moves(t_seg, now()))
            self.current = None
            self.end_segment(spec, t_seg, status, res)
        self.finish()

    def cycle_once_ctx_init(self, t_seg):
        c = self.ctx
        a = self.angle
        c.t = c.ts = 0.0
        c.t = now()
        c.dt = 0.01
        c.x = self.arduino.position(now())
        c.v = self.v_cmd
        c.psi, c.rate = a["psi"], a["rate"]
        c.amp = [self.amp(0), self.amp(1)]
        c.amp_joint = self.amp(2)
        c.amp_energy = self.amp_energy() if self.arms == 1 else c.amp[0]
        return c

    def status_line(self, test, c):
        a = self.angle
        msg = (f"{test.name:<14} {test.phase:<6} psi1={math.degrees(a['psi'][0]):+7.1f}deg ")
        if self.arms == 2:
            msg += f"psi2={math.degrees(a['psi'][1]):+7.1f}deg "
        msg += (f"swing={c.amp[0]:5.1f}deg cart={c.x:+.3f}m v={self.v_cmd:+.2f}m/s "
                f"frames={self.frames.count} gaps={self.frames.gaps}")
        self.say("RUN", msg)

    def begin_segment(self, i, spec):
        self.seg_index, self.seg_name = i, spec["name"]
        self.event("test", f"start {spec['name']} ({spec['kind']}) " +
                   json.dumps({k: v for k, v in spec["params"].items() if k not in ("moves", "components")}))

    def end_segment(self, spec, t0, status, res):
        t1 = now()
        self.results["tests"].append({"seg": self.seg_index, "name": spec["name"], "kind": spec["kind"],
                                      "t_start_s": round(t0, 4), "t_end_s": round(t1, 4), "status": status,
                                      "results": res})
        self.seg_csv.row([self.seg_index, spec["name"], spec["kind"], f"{t0:.6f}", f"{t1:.6f}", status,
                          json.dumps(spec["params"]), json.dumps(res)])
        self.w.flush()
        self.save_meta()

    def after_moves(self, t0, t1):
        """Measure the arm's natural frequency and which way the cart pushes it, for the tests that follow."""
        res = {}
        try:
            q = quick_identify(self.hist, self.arms, t0, t1)
        except Exception as exc:
            q = None
            res["quick_fit_error"] = repr(exc)
        if q:
            res["quick_fit"] = q
        if self.arms == 1:
            w_zc = zero_cross_w0(self.hist, t0, t1)
            res["w0_zero_cross"] = w_zc
            if q and q["ok"]:
                self.model.update(w0=q["w0"], k=q["k"], k_sign=sgn(q["k"]), sign_known=True,
                                  source="fitted from the moves test")
                if w_zc and abs(w_zc / q["w0"] - 1) > 0.15:
                    self.model["w0"] = w_zc
            elif w_zc:
                self.model.update(w0=w_zc, source="zero crossings in the moves test (direction assumed)")
            self.say("MODEL", f"natural frequency {self.model['w0']:.3f} rad/s = {self.model['w0'] / (2 * math.pi):.3f} Hz "
                              f"(period {2 * math.pi / self.model['w0']:.3f} s); cart coupling "
                              f"{self.model['k']:+.2f} rad/m (k*g/w0^2 = {self.model['k'] * G / self.model['w0'] ** 2:+.2f}, "
                              f"1.00 for a rigid arm); {self.model['source']}")
        else:
            if q and q["ok"]:
                self.model.update(k=q["k"], k_sign=sgn(q["k"]), f_modes=q["f_modes"], sign_known=True,
                                  source="fitted from the moves test")
            self.say("MODEL", f"mode frequencies {self.model['f_modes'][0]:.3f} Hz and {self.model['f_modes'][1]:.3f} Hz; "
                              f"cart coupling {self.model['k']:+.2f}; {self.model['source']}")
        if not self.model.get("sign_known"):
            self.model["sign_known"] = True
            self.event("model", "could not measure which way the cart pushes the arm; assuming the twin's direction "
                                "(a damping check reverses it if the swing grows)")
        res["model_after"] = dict(self.model)
        self.meta["model_used"] = dict(self.model)
        return res

    def finish(self):
        self.seg_name = "end"
        if self.arduino and self.arduino.sent_any and not self.arduino.disabled:
            self.arduino.command(-1, note="end of run: motor disabled")
            self.say("STOP", "motor disabled (Arduino timer stopped)")
        x = self.arduino.position() if self.arduino else 0.0
        self.say("STOP", f"step count {self.arduino.steps}: {x * 1000:+.1f} mm from the start")
        ans = self.ask("Optional check: how far is the cart from where it started, in mm (+ = the way the "
                       "step count goes up)? Measure with a ruler, or Enter to skip: ")
        if ans:
            try:
                self.meta["end_check"] = {"measured_mm": float(ans), "step_count_mm": round(x * 1000, 2),
                                          "missed_mm": round(float(ans) - x * 1000, 2)}
                self.say("STOP", f"steps say {x * 1000:+.1f} mm, ruler says {float(ans):+.1f} mm: "
                                 f"{float(ans) - x * 1000:+.1f} mm lost or gained")
            except ValueError:
                self.meta["end_check"] = {"answer": ans}

    def summary(self):
        f = self.frames
        out = {"stop_reason": self.stop_reason, "duration_s": round(now(), 3), "cycles": self.cycle,
               "model_used": self.model}
        if f:
            out.update(frames=f.count, frame_gaps=f.gaps, crc_errors=f.crc_errors, discarded_bytes=f.discarded,
                       duplicate_frames=f.duplicates, receiver_restarts=f.restarts)
        decays = [t for t in self.results["tests"] if t["kind"] in ("pump_decay", "mode_decay")]
        out["decays"] = [{"name": d["name"], **d["results"]} for d in decays]
        out["tests"] = self.results["tests"]
        return out

    def save_meta(self):
        self.meta["summary"] = self.summary() if self.frames else {}
        tmp = self.dir / "metadata.json.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        os.replace(tmp, self.dir / "metadata.json")

    def close(self):
        """Always runs: brake if needed, disable the motor, close ports, write the files."""
        if self.current is not None:
            spec, t_seg, test = self.current
            self.current = None
            try:
                self.end_segment(spec, t_seg, "stopped", dict(test.result))
            except Exception:
                pass
        try:
            if self.arduino and self.arduino.sent_any and not self.arduino.disabled:
                try:
                    v = self.v_cmd
                    t_end = time.monotonic() + 0.6
                    while abs(v) > 0.01 and time.monotonic() < t_end:
                        v -= sgn(v) * min(abs(v), ACCEL_MAX_M_S2 * 0.01)
                        timer, _, _ = Arduino.timer_for(v)
                        self.arduino.command(timer, note="braking at stop")
                        self.v_cmd = v
                        time.sleep(0.008)
                    self.arduino.command(-1, note="stop: motor disabled")
                    self.say("STOP", "motor disabled")
                except Exception as exc:
                    self.say("FAULT", f"could not send the stop command: {exc}")
        finally:
            if self.keys:
                self.keys.close()
            for p in (self.ard, self.esp):
                try:
                    if p is not None and p.is_open:
                        p.close()
                except Exception:
                    pass
            self.meta["ended_local"] = datetime.now().astimezone().isoformat(timespec="seconds")
            self.save_meta()
            with open(self.dir / "quick_results.json", "w", encoding="utf-8") as f:
                json.dump(self.summary(), f, indent=1, default=str)
            self.w.close()
            print(f"Saved: {self.dir}", flush=True)


def columns_text():
    out = ["Files in this folder (PendulumSysID.py)", "",
           "metadata.json       settings, rig measurements, calibration, the full test plan with every random choice,",
           "                    computer and library versions, and a summary of what each test measured",
           "quick_results.json  the summary on its own",
           "samples.csv         one row per control cycle (about 100 per second)",
           "angle_frames.csv    every frame from the ESP32 receiver, as received",
           "motor_commands.csv  every command sent to the Arduino and its reply",
           "events.csv          test starts, moves, governor actions, keys, faults",
           "segments.csv        one row per test: times, status, random parameters, results",
           "serial_raw.bin      every serial byte with its time: header line, then records of",
           "                    <float64 t_s><uint8 channel><uint16 n><n bytes>, little-endian;",
           "                    channel 1 = from ESP32, 2 = from Arduino, 3 = to Arduino",
           "terminal.log        what was printed",
           "PendulumSysID_used.py  the exact script that made this run", ""]
    for name, cols in COLUMNS.items():
        out.append(name)
        for col, doc in cols:
            out.append(f"  {col:<18} {doc}")
        out.append("")
    return "\n".join(out)


def host_info():
    info = {"python": sys.version.split()[0], "numpy": np.__version__,
            "pyserial": getattr(serial, "__version__", "?"), "platform": platform.platform(),
            "machine": platform.machine(), "node": platform.node()}
    try:
        with open(__file__, "rb") as f:
            info["script_sha256"] = hashlib.sha256(f.read()).hexdigest()
    except Exception:
        pass
    try:
        import subprocess
        r = subprocess.run(["git", "-C", str(HERE), "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=3)
        if r.returncode == 0:
            info["git_commit"] = r.stdout.strip()
    except Exception:
        pass
    return info


def main():
    ap = argparse.ArgumentParser(description="System identification runs for the ENGT480 pendulum rig.")
    ap.add_argument("--plan", default=PLAN, choices=["full", "quick", "decay", "excite", "motor"])
    ap.add_argument("--arms", type=int, choices=[1, 2], default=ARMS)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()
    if args.arms is None:
        args.arms = 2 if HERE.name.lower().startswith("double") else 1
    if args.seed is None:
        args.seed = random.SystemRandom().randrange(1, 1_000_000)
    run = Run(args)
    atexit.register(run.close)
    try:
        run.run()
    except KeyboardInterrupt:
        run.stop_reason = "Ctrl+C"
        run.event("stop", "Ctrl+C: braking and saving")
    except Exception as exc:
        run.stop_reason = f"{type(exc).__name__}: {exc}"
        run.event("fault", run.stop_reason)
        run.w.write("term", traceback.format_exc())
        traceback.print_exc()
    finally:
        atexit.unregister(run.close)
        run.close()


if __name__ == "__main__":
    main()
