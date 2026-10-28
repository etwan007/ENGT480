#!/usr/bin/env python3
"""PendulumExcite.py - move the cart in many different ways, forever, and log everything.

Made for building and checking a simulation (MuJoCo etc.) of the arm(s). With the arm(s) hanging,
the cart does steps, quick kicks, out-and-back pulses, sine dwells, chirps, multi-sines, PRBS,
smooth random noise, constant-speed sweeps, resonance pumping and long rests (free swings), in a
shuffled order with random sizes, until you press Ctrl+C. Every 10 ms cycle is logged: raw
receiver frames, angles, what the cart was meant to do, what was sent, what the Arduino counted,
and the timing of all of it.

Needs only numpy and pyserial (no other files). Run it from any folder:
    ~/PythonLibraries/bin/python PendulumExcite.py               # arms auto-detected
    ~/PythonLibraries/bin/python PendulumExcite.py --seed 7      # repeat the exact same motions
    ~/PythonLibraries/bin/python PendulumExcite.py --only chirp,step,rest
    ~/PythonLibraries/bin/python PendulumExcite.py --arms 1      # force single (or --arms 1,2)
Startup:
    1. motor power OFF, cart centred, arm(s) hanging still, Enter  -> hanging calibration
    2. motor power ON, arm(s) still hanging, Enter                  -> motions start
Ctrl+C once: the cart brakes to a stop, motor off, files saved. Ctrl+C twice: motor off at once.
A limit switch or the stop button freezes the Arduino. The script notices (no reply), saves and
exits. Move the cart off the switch / release the button; rerunning resets the Arduino.

Output: run_logs/excite_<date>_<time>/ next to this script
    samples.csv      one row per 10 ms cycle (column meanings in columns.txt)
    segments.csv     one row per motion: type, start/end, why it ended, every parameter
    events.csv       calibration, governor pauses, step-count glitches, faults, stop
    metadata.json    all settings, calibration, seed, summary at the end
    esp32_raw.bin    every byte the receiver sent (protocol v1 frames, re-decodable)
    terminal.log     everything printed
    PendulumExcite_used.py   copy of this script as it ran
About 2.5 MB of samples.csv per minute.
"""

import argparse
import binascii
import csv
import json
import math
import platform
import queue
import random
import shutil
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import serial

# =====================================================================================
# SETTINGS
# =====================================================================================
ESP32_PORT, ESP32_BAUD = "/dev/ttyUSB0", 921600        # receiver, protocol v1
ARDUINO_PORT, ARDUINO_BAUD = "/dev/ttyACM0", 115200    # Mega running ArduinoSlave.ino
DISTANCE_PER_6400_STEPS_M = 0.638175                   # cart travel for 6400 steps

# Receiver channels that are arms, in order from the cart up. None = every channel that reads
# valid during the hanging calibration. (1,) = single arm on TX1, (1, 2) = double.
ARM_CHANNELS = None

# Random seed. None = new motions every run (the seed used is printed and saved).
SEED = None

# --- track limits (all measured from where the cart is when the motor turns on) ---
PLAN_LIMIT_M = 0.15      # motions are planned inside +/- this
SOFT_LIMIT_M = 0.20      # the speed command brakes so the cart cannot pass this
ABORT_LIMIT_M = 0.25     # motor off and stop if the step count says the cart is past this
SPEED_MAX_M_S = 0.6      # fastest cart speed ever sent
ACCEL_MAX_M_S2 = 6.0     # hardest acceleration ever sent (lower it if the stepper skips)
MOTION_MARGIN = 0.9      # planned motions use at most this fraction of the two limits above
TRACK_GAIN_PER_S = 4.0   # position feedback that keeps the cart on the planned path
SPEED_DEADBAND_M_S = 0.0003  # slower commands hold position with a +/-1 step dither

# --- arm safety ---
MAX_SWING_DEG = 90.0     # any arm further than this from hanging: motion stops, cart holds still
RESUME_SWING_DEG = 45.0  # ...until every arm is back inside this, then the next motion starts
GOVERNOR_MAX_HOLD_S = 90.0
MISSING_PAUSE_S = 0.3    # angles missing this long: motion stops, cart holds still
MISSING_STOP_S = 5.0     # angles missing this long: the run ends
MAX_STEP_GLITCHES = 20   # step-count jumps (counter noise) allowed before the run ends

# --- motion mix ---
# How often each motion comes up per shuffled round (0 = never).
MOTION_WEIGHTS = {"rest": 2, "step": 2, "kick": 1, "pulse": 1, "sine": 2, "chirp": 2,
                  "multisine": 1, "prbs": 1, "noise": 1, "sweep": 1, "resonance": 2}
FREQ_MIN_HZ, FREQ_MAX_HZ = 0.15, 5.0     # frequency range of the oscillating motions
TAIL_REST_S = (2.0, 6.0)                 # free swing after every motion, s (random in range)
NATURAL_FREQ_GUESS_HZ = 1.25             # first guess for resonance motions; replaced by the
                                         # frequency measured from the free swings

# --- calibration and output ---
HANGING_SAMPLES = 100                    # frames averaged for the hanging reference (1 s)
MAX_HANGING_DEVIATION_COUNTS = 80        # arms must be this still while calibrating (counts)
BASELINE_REST_S = 5.0                    # still cart at the start (sensor noise baseline)
STATUS_INTERVAL_S = 1.0
SAVE_RAW_ESP32_BYTES = True
RUN_LOG_DIRECTORY = Path(__file__).resolve().parent / "run_logs"

# -------------------------------------------------------------------------------------
T = 0.01                                 # receiver frame period = control period, s
ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
RAD_PER_COUNT = 2 * math.pi / ENCODER_COUNTS
M_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_STEP_HZ = 125000.0                 # ArduinoSlave.ino: step rate = 125000 / (|timer| + 1)
OSCILLATING = {"sine", "chirp", "multisine", "noise", "prbs", "sweep", "resonance"}
SCRIPT_VERSION = "2026-10-27"

# =====================================================================================
# Receiver protocol v1 (same as Code/*/angle_protocol.py, copied so this file stands alone)
# =====================================================================================
MAGIC = b'\xa5\x5a\xc3\x3c'
PROTO_VERSION, PAYLOAD_SIZE, FRAME_SIZE = 1, 29, 37
VALID, ENCODER_FAILED, WIRELESS_TIMEOUT, NEVER_RECEIVED = range(4)
STATUS_NAMES = ('valid', 'encoder_read_failed', 'wireless_timeout', 'never_received')
UNKNOWN_AGE = 0xFFFFFFFF
PRESENT_TIMEOUT_MS = 50


class AngleProtocolError(TimeoutError):
    pass


@dataclass(frozen=True)
class AnglePacket:
    sequence: int
    receiver_ms: int
    angles: tuple
    statuses: tuple
    ages_ms: tuple


def decode_frame(frame):
    if len(frame) != FRAME_SIZE or frame[:4] != MAGIC or frame[4:6] != bytes((PROTO_VERSION, PAYLOAD_SIZE)):
        raise ValueError('unsupported frame header')
    if binascii.crc_hqx(frame[4:-2], 0xFFFF) != int.from_bytes(frame[-2:], 'big'):
        raise ValueError('CRC mismatch')
    sequence, receiver_ms = struct.unpack_from('>II', frame, 6)
    angles, statuses, ages = [], [], []
    for i in range(3):
        angle, status, age = struct.unpack_from('>HBI', frame, 14 + 7 * i)
        if status == VALID:
            ok = angle < 16384 and age <= PRESENT_TIMEOUT_MS
        elif status == ENCODER_FAILED:
            ok = angle == 0xFFFF and age <= PRESENT_TIMEOUT_MS
        elif status == WIRELESS_TIMEOUT:
            ok = angle == 0xFFFF and age > PRESENT_TIMEOUT_MS
        elif status == NEVER_RECEIVED:
            ok = angle == 0xFFFF and age == UNKNOWN_AGE
        else:
            ok = False
        if not ok:
            raise ValueError('inconsistent angle/status/age')
        angles.append(angle)
        statuses.append(status)
        ages.append(age)
    return AnglePacket(sequence, receiver_ms, tuple(angles), tuple(statuses), tuple(ages))


class AnglePacketReader:
    """Keeps partial frames, resynchronises on bad bytes, returns the newest complete frame."""
    MAX_READ_BYTES = 4096
    CHUNK_SIZE = 512

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = self.discarded_bytes = self.sequence_gaps = 0
        self.superseded_frames = self.duplicate_frames = self.receiver_restarts = 0
        self.frames = 0
        self.last_packet = None

    def _discard(self, count):
        del self.buffer[:count]
        self.discarded_bytes += count

    def _parse(self):
        latest = None
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self._discard(max(0, len(self.buffer) - len(MAGIC) + 1))
                break
            if start:
                self._discard(start)
            if len(self.buffer) < 6:
                break
            if self.buffer[4:6] != bytes((PROTO_VERSION, PAYLOAD_SIZE)):
                self._discard(1)
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            try:
                packet = decode_frame(bytes(self.buffer[:FRAME_SIZE]))
            except ValueError as exc:
                if str(exc) == 'CRC mismatch':
                    self.crc_errors += 1
                self._discard(1)
                continue
            del self.buffer[:FRAME_SIZE]
            previous = self.last_packet
            if previous is not None:
                delta = (packet.sequence - previous.sequence) & 0xFFFFFFFF
                time_delta = (packet.receiver_ms - previous.receiver_ms) & 0xFFFFFFFF
                if time_delta >= 0x80000000:
                    self.receiver_restarts += 1
                elif delta == 0 or delta >= 0x80000000:
                    self.duplicate_frames += 1
                    continue
                elif delta > 1:
                    self.sequence_gaps += delta - 1
            self.last_packet = packet
            self.frames += 1
            if latest is not None:
                self.superseded_frames += 1
            latest = packet
        return latest

    def read_latest(self, port, timeout_s=0.1):
        deadline = time.monotonic() + timeout_s
        total = 0
        latest = self._parse()
        while time.monotonic() < deadline:
            waiting = max(0, int(port.in_waiting))
            if latest is not None and waiting == 0:
                return latest
            if total >= self.MAX_READ_BYTES:
                raise AngleProtocolError('serial_backlog: more than 4096 bytes queued')
            chunk = port.read(min(waiting or 1, self.CHUNK_SIZE, self.MAX_READ_BYTES - total))
            total += len(chunk)
            self.buffer.extend(chunk)
            parsed = self._parse()
            if parsed is not None:
                if latest is not None:
                    self.superseded_frames += 1
                latest = parsed
        if latest is not None and int(port.in_waiting) == 0:
            return latest
        raise AngleProtocolError(f'no_valid_frame in {timeout_s * 1000:.0f} ms '
                                 f'(crc_errors={self.crc_errors} discarded_bytes={self.discarded_bytes})')

    def discard_pending(self, port, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while int(port.in_waiting):
            if time.monotonic() >= deadline:
                raise AngleProtocolError('serial_backlog: could not drain the receiver input')
            self.buffer.extend(port.read(min(int(port.in_waiting), self.CHUNK_SIZE)))
            self._parse()


class TeePort:
    """Serial port wrapper that hands every byte read to a sink (the raw .bin log)."""

    def __init__(self, port, sink):
        self.port, self.sink = port, sink

    @property
    def in_waiting(self):
        return self.port.in_waiting

    def read(self, n):
        data = self.port.read(n)
        if data and self.sink is not None:
            self.sink(data)
        return data


# =====================================================================================
# Logging (all file writes happen on a background thread, never inside the 10 ms cycle)
# =====================================================================================
#           name, format, meaning
SAMPLE_COLUMNS = [
    ("t_run_s", "%.5f", "time since the motor turned on (frame arrival on the Pi), s"),
    ("t_mono_s", "%.6f", "Pi time.monotonic() at frame arrival, s"),
    ("unix_time_s", "%.6f", "Pi wall clock at frame arrival, s"),
    ("cycle", "%d", "control cycle number"),
    ("cycle_period_ms", "%.3f", "time since the previous cycle's frame arrival, ms"),
    ("sensor_read_ms", "%.3f", "time spent waiting for/reading the receiver frame, ms"),
    ("cmd_sent_after_frame_ms", "%.3f", "frame arrival -> motor command written, ms"),
    ("arduino_reply_ms", "%.3f", "motor command written -> Arduino step reply received, ms"),
    ("loop_ms", "%.3f", "whole cycle compute time (frame arrival -> end of cycle), ms"),
    ("segment", "%d", "motion number (see segments.csv)"),
    ("motion", "%s", "motion type of the current segment"),
    ("seg_t_s", "%.4f", "time since this segment started, s"),
    ("phase", "%s", "move = planned cart path is moving, free = planned cart path is still"),
    ("governor", "%d", "1 while the swing/angle governor holds the cart"),
    ("stopping", "%d", "1 after Ctrl+C while braking"),
    ("frame_ok", "%d", "1 = a frame arrived and every arm channel was valid"),
    ("esp32_sequence", "%d", "receiver frame sequence number"),
    ("receiver_ms", "%d", "receiver uptime stamp in the frame, ms"),
    ("status_1", "%s", "receiver status, channel 1 (valid / encoder_read_failed / wireless_timeout / never_received)"),
    ("status_2", "%s", "receiver status, channel 2"),
    ("status_3", "%s", "receiver status, channel 3"),
    ("radio_age_1_ms", "%d", "age of channel 1's last radio packet at the receiver, ms"),
    ("radio_age_2_ms", "%d", "age of channel 2's last radio packet, ms"),
    ("radio_age_3_ms", "%d", "age of channel 3's last radio packet, ms"),
    ("raw_1", "%d", "raw encoder counts, channel 1 (0..16383, 65535 = missing)"),
    ("raw_2", "%d", "raw encoder counts, channel 2"),
    ("raw_3", "%d", "raw encoder counts, channel 3"),
    ("rel_1_deg", "%.4f", "channel 1 angle from its hanging reference, wrapped to +/-180, deg (repo sign)"),
    ("rel_2_deg", "%.4f", "channel 2 angle from its hanging reference (for arm 2: the joint angle), deg"),
    ("rel_3_deg", "%.4f", "channel 3 angle from its hanging reference, deg"),
    ("unwrapped_1_deg", "%.4f", "channel 1 angle, continuous through full turns, deg"),
    ("unwrapped_2_deg", "%.4f", "channel 2 angle, continuous, deg"),
    ("unwrapped_3_deg", "%.4f", "channel 3 angle, continuous, deg"),
    ("arm1_abs_deg", "%.4f", "arm 1 absolute angle from hanging (continuous), deg"),
    ("arm2_abs_deg", "%.4f", "arm 2 absolute angle from hanging = arm 1 + joint (continuous), deg"),
    ("arm3_abs_deg", "%.4f", "arm 3 absolute angle from hanging, deg"),
    ("arm1_rate_deg_s", "%.3f", "arm 1 angular speed, finite difference over receiver time, deg/s"),
    ("arm2_rate_deg_s", "%.3f", "arm 2 absolute angular speed, deg/s"),
    ("arm3_rate_deg_s", "%.3f", "arm 3 absolute angular speed, deg/s"),
    ("swing_amp_deg", "%.3f", "largest |arm angle| (wrapped) over the last second, deg"),
    ("natural_freq_hz", "%.5f", "arm 1 free-swing frequency measured so far (used by resonance motions), Hz"),
    ("x_ref_m", "%.6f", "planned cart position now, m"),
    ("v_ref_m_s", "%.5f", "planned cart speed now, m/s"),
    ("a_ref_m_s2", "%.4f", "planned cart acceleration now, m/s^2"),
    ("v_ff_m_s", "%.5f", "feed-forward speed: planned average speed over the next 10 ms, m/s"),
    ("v_fb_m_s", "%.5f", "feedback speed: TRACK_GAIN * (planned - estimated position), m/s"),
    ("v_want_m_s", "%.5f", "v_ff + v_fb before limits, m/s"),
    ("v_cmd_m_s", "%.5f", "speed after the speed/accel/soft-limit limiters, m/s"),
    ("a_cmd_m_s2", "%.4f", "(v_cmd - previous v_cmd) / 10 ms, m/s^2"),
    ("limiters", "%s", "which limiters changed the command: speed / accel / soft"),
    ("timer_cmd", "%d", "int32 sent to the Arduino (timer value; sign = direction; +/-65535 = hold dither)"),
    ("step_rate_hz", "%.3f", "step rate that timer value gives, + = step count up, steps/s"),
    ("v_sent_m_s", "%.5f", "cart speed that timer value gives (0 while dithering), m/s"),
    ("hold_x_m", "%.6f", "position held while the speed is inside the deadband, m"),
    ("steps_raw", "%d", "int16 step count the Arduino replied"),
    ("steps_delta", "%d", "change in step count used (a glitch is replaced by the commanded motion)"),
    ("steps", "%d", "step count since the motor turned on (unwrapped)"),
    ("cart_m", "%.6f", "cart position from the step count (counted when the command arrived), m (+ = step count up)"),
    ("cart_est_m", "%.6f", "cart position estimated at the frame time (cart_m + v_sent * time since that send), m"),
    ("track_err_m", "%.6f", "x_ref - cart_est, m"),
    ("step_glitch", "%d", "1 = the step count jumped more than the commanded speed allows"),
    ("crc_errors", "%d", "receiver frames with a bad CRC so far"),
    ("discarded_bytes", "%d", "receiver bytes thrown away while resynchronising so far"),
    ("sequence_gaps", "%d", "receiver frames lost (sequence gaps) so far"),
    ("superseded_frames", "%d", "frames skipped because a newer one was already queued, so far"),
    ("missing_frames", "%d", "cycles without a usable frame so far"),
    ("step_glitches", "%d", "step-count glitches so far"),
    ("log_dropped", "%d", "rows the logger had to drop (queue full) so far"),
]
SAMPLE_NAMES = [c[0] for c in SAMPLE_COLUMNS]
SAMPLE_FORMATS = [c[1] for c in SAMPLE_COLUMNS]
EVENT_COLUMNS = ["t_run_s", "unix_time_s", "cycle", "event", "detail"]
SEGMENT_COLUMNS = ["segment", "motion", "t_start_s", "t_end_s", "duration_s", "planned_duration_s",
                   "end_reason", "x_start_m", "x_end_planned_m", "peak_abs_x_m", "peak_abs_v_m_s",
                   "peak_abs_a_m_s2", "max_swing_deg", "params_json"]


def _fmt(value, spec):
    if value is None or value == "":
        return ""
    if spec == "%s":
        return str(value)
    try:
        return spec % value
    except TypeError:
        return str(value)


class Recorder:
    def __init__(self, run_dir, raw_bytes):
        self.run_dir = run_dir
        self.q = queue.Queue(maxsize=200000)
        self.dropped = 0
        self.files = {}
        self.writers = {}
        for name, header in (("samples", SAMPLE_NAMES), ("events", EVENT_COLUMNS), ("segments", SEGMENT_COLUMNS)):
            f = open(run_dir / f"{name}.csv", "w", newline="", buffering=1 << 16)
            w = csv.writer(f)
            w.writerow(header)
            self.files[name], self.writers[name] = f, w
        self.raw = open(run_dir / "esp32_raw.bin", "wb", buffering=1 << 16) if raw_bytes else None
        self.thread = threading.Thread(target=self._run, name="recorder", daemon=True)
        self.thread.start()

    def _put(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def sample(self, values):
        self._put(("samples", values))

    def event(self, t_run, cycle, event, detail=""):
        self._put(("events", (None if t_run is None else round(t_run, 5), f"{time.time():.6f}",
                              cycle, event, detail)))

    def segment(self, row):
        self._put(("segments", row))

    def raw_bytes(self, data):
        if self.raw is not None:
            self._put(("raw", bytes(data)))

    def _run(self):
        last_flush = time.monotonic()
        while True:
            try:
                item = self.q.get(timeout=0.5)
            except queue.Empty:
                item = None
            if item is not None:
                if item[0] is None:
                    break
                kind, payload = item
                if kind == "samples":
                    self.writers["samples"].writerow([_fmt(v, s) for v, s in zip(payload, SAMPLE_FORMATS)])
                elif kind == "raw":
                    self.raw.write(payload)
                else:
                    self.writers[kind].writerow(["" if v is None else v for v in payload])
            if time.monotonic() - last_flush > 1.0:
                for f in self.files.values():
                    f.flush()
                if self.raw is not None:
                    self.raw.flush()
                last_flush = time.monotonic()

    def close(self):
        self.q.put((None, None))
        self.thread.join(timeout=30)
        for f in self.files.values():
            f.close()
        if self.raw is not None:
            self.raw.close()


started_at = time.monotonic()
terminal_log = None


def log(tag, message):
    line = f"[{time.monotonic() - started_at:8.2f}s] [{tag}] {message}"
    print(line, flush=True)
    if terminal_log is not None:
        terminal_log.write(line + "\n")
        terminal_log.flush()


# =====================================================================================
# Motion planning. Every motion is a cart path sampled every 10 ms that starts where the
# previous one ended, stays inside PLAN_LIMIT_M, MOTION_MARGIN * the speed and accel limits,
# and ends at rest.
# =====================================================================================
V_PLAN = SPEED_MAX_M_S * MOTION_MARGIN
A_PLAN = ACCEL_MAX_M_S2 * MOTION_MARGIN


def follow(targets, x0, v0, vmax, amax):
    """Fastest path that chases a target sequence within vmax / amax (trapezoid moves)."""
    x, v = float(x0), float(v0)
    out = np.empty(len(targets) + 1)
    out[0] = x
    for i, xt in enumerate(targets):
        e = xt - x
        stop_speed = math.sqrt(2 * amax * max(abs(e) - 0.5 * amax * T * T, 0.0))
        v_des = math.copysign(min(vmax, stop_speed, abs(e) / T), e)
        v += max(-amax * T, min(amax * T, v_des - v))
        x += v * T
        out[i + 1] = x
    return out


def move_time(d, vmax, amax):
    d = abs(d)
    if d < vmax * vmax / amax:
        return 2 * math.sqrt(d / amax)
    return d / vmax + vmax / amax


def taper(n, ramp_n):
    w = np.ones(n)
    ramp_n = max(1, min(ramp_n, n // 2))
    r = 0.5 - 0.5 * np.cos(np.linspace(0, math.pi, ramp_n))
    w[:ramp_n] = r
    w[n - ramp_n:] = r[::-1]
    return w


def derivatives(x):
    v = np.gradient(x, T)
    a = np.gradient(v, T)
    return v, a


def fit(shape, room):
    """Scale a zero-mean shape so it fits the room on the track and the speed/accel limits."""
    v, a = derivatives(shape)
    peaks = (np.max(np.abs(shape)), np.max(np.abs(v)), np.max(np.abs(a)))
    scale = min(1.0, room / max(peaks[0], 1e-9), V_PLAN / max(peaks[1], 1e-9), A_PLAN / max(peaks[2], 1e-9))
    return shape * scale, scale


def amp_limit(f, room):
    w = 2 * math.pi * f
    return min(room, V_PLAN / w, A_PLAN / (w * w))


def log_uniform(rng, lo, hi):
    return math.exp(rng.uniform(math.log(lo), math.log(hi)))


def plan_motion(kind, x0, rng, f_nat):
    """Return (path array starting at x0, params dict, has_own_rest)."""
    room = max(PLAN_LIMIT_M - abs(x0), 0.0)
    p = {"x0": round(x0, 6)}
    own_rest = False
    if kind == "rest":
        dur = rng.uniform(5, 15)
        path = np.full(int(dur / T) + 1, x0)
        own_rest = True
        p["duration_s"] = dur
    elif kind in ("step", "kick", "recenter"):
        if kind == "step":
            while True:
                xt = rng.uniform(-PLAN_LIMIT_M, PLAN_LIMIT_M)
                if abs(xt - x0) > 0.02:
                    break
            vmax, amax = V_PLAN * rng.uniform(0.15, 1.0), A_PLAN * rng.uniform(0.2, 1.0)
            hold = rng.uniform(3, 8)
        elif kind == "kick":
            d = rng.uniform(0.005, 0.03)
            xt = x0 + d if x0 + d <= PLAN_LIMIT_M and (x0 - d < -PLAN_LIMIT_M or rng.random() < 0.5) else x0 - d
            vmax, amax = V_PLAN, A_PLAN * rng.uniform(0.7, 1.0)
            hold = rng.uniform(3, 6)
        else:
            xt = 0.0
            vmax, amax = V_PLAN * 0.4, A_PLAN * 0.3
            hold = 1.0
        n = int((move_time(xt - x0, vmax, amax) + hold) / T) + 1
        path = follow(np.full(n, xt), x0, 0.0, vmax, amax)
        p.update(target_m=xt, vmax=vmax, amax=amax, hold_s=hold)
        own_rest = kind == "recenter"
    elif kind == "pulse":
        d = rng.uniform(0.01, 0.06)
        sign = 1 if x0 + d <= PLAN_LIMIT_M and (x0 - d < -PLAN_LIMIT_M or rng.random() < 0.5) else -1
        vmax, amax = V_PLAN * rng.uniform(0.4, 1.0), A_PLAN * rng.uniform(0.5, 1.0)
        dwell = rng.uniform(0.0, 0.4)
        n_out = int((move_time(d, vmax, amax) + dwell) / T) + 1
        n_back = int((move_time(d, vmax, amax) + 0.3) / T) + 1
        targets = np.concatenate([np.full(n_out, x0 + sign * d), np.full(n_back, x0)])
        path = follow(targets, x0, 0.0, vmax, amax)
        p.update(distance_m=sign * d, vmax=vmax, amax=amax, dwell_s=dwell)
    elif kind == "sine":
        f = log_uniform(rng, FREQ_MIN_HZ * 1.5, FREQ_MAX_HZ)
        cycles = rng.randint(4, 15)
        n = int(cycles / f / T) + 1
        t = np.arange(n) * T
        amp = amp_limit(f, room) * rng.uniform(0.3, 1.0)
        shape = amp * np.sin(2 * math.pi * f * t) * taper(n, int(min(1 / f, cycles / f / 4) / T))
        shape, scale = fit(shape, room)
        path = x0 + shape
        p.update(freq_hz=f, cycles=cycles, amplitude_m=amp * scale)
    elif kind == "chirp":
        fa, fb = sorted((log_uniform(rng, FREQ_MIN_HZ, 1.0), log_uniform(rng, 1.5, FREQ_MAX_HZ)))
        if rng.random() < 0.5:
            fa, fb = fb, fa
        dur = rng.uniform(15, 40)
        n = int(dur / T) + 1
        t = np.arange(n) * T
        k = math.log(fb / fa)
        phase = 2 * math.pi * fa * dur / k * (np.exp(k * t / dur) - 1)
        f_t = fa * np.exp(k * t / dur)
        frac = rng.uniform(0.3, 1.0)
        env = np.array([amp_limit(f, room) for f in f_t]) * frac
        shape = env * np.sin(phase) * taper(n, int(min(1.0 / min(fa, fb), dur / 4) / T))
        shape, scale = fit(shape, room)
        path = x0 + shape
        p.update(f_start_hz=fa, f_end_hz=fb, duration_s=dur, sweep="log", amplitude_fraction=frac * scale)
    elif kind == "multisine":
        m = rng.randint(3, 8)
        freqs = sorted(log_uniform(rng, FREQ_MIN_HZ * 1.5, FREQ_MAX_HZ * 0.8) for _ in range(m))
        tilt = rng.uniform(0.0, 2.0)          # amplitude ~ 1/f^tilt
        phases = [rng.uniform(0, 2 * math.pi) for _ in range(m)]
        dur = rng.uniform(10, 30)
        n = int(dur / T) + 1
        t = np.arange(n) * T
        shape = sum((1 / f ** tilt) * np.sin(2 * math.pi * f * t + ph) for f, ph in zip(freqs, phases))
        shape = shape * taper(n, int(1.0 / T))
        shape, scale = fit(shape, room * rng.uniform(0.4, 1.0))
        path = x0 + shape
        p.update(freqs_hz=freqs, phases_rad=phases, tilt=tilt, duration_s=dur, scale=scale)
    elif kind == "noise":
        fl = log_uniform(rng, FREQ_MIN_HZ, 1.0)
        fh = log_uniform(rng, max(fl * 2, 1.0), FREQ_MAX_HZ)
        dur = rng.uniform(10, 30)
        n = int(dur / T) + 1
        seed = rng.randrange(1 << 30)
        spec = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
        fr = np.fft.rfftfreq(n, T)
        spec[(fr < fl) | (fr > fh)] = 0
        shape = np.fft.irfft(spec, n) * taper(n, int(1.0 / T))
        shape, scale = fit(shape, room * rng.uniform(0.4, 1.0))
        path = x0 + shape
        p.update(band_hz=[fl, fh], duration_s=dur, noise_seed=seed, scale=scale)
    elif kind == "prbs":
        amp = rng.uniform(0.01, max(room, 0.011))
        amp = min(amp, room)
        base = rng.uniform(0.1, 0.8)
        vmax, amax = V_PLAN * rng.uniform(0.5, 1.0), A_PLAN * rng.uniform(0.4, 1.0)
        dur = rng.uniform(10, 30)
        targets, levels = [], []
        while len(targets) * T < dur:
            level = rng.choice((-1, 1))
            hold = base * rng.randint(1, 4)
            levels.append([level, round(hold, 3)])
            targets.extend([x0 + level * amp] * int(hold / T))
        targets.extend([x0] * int((move_time(2 * amp, vmax, amax) + 0.5) / T))
        path = follow(np.array(targets), x0, 0.0, vmax, amax)
        p.update(amplitude_m=amp, base_period_s=base, vmax=vmax, amax=amax, levels=levels)
    elif kind == "sweep":
        amp = min(rng.uniform(0.03, max(room, 0.031)), room)
        vmax, amax = rng.uniform(0.03, V_PLAN * 0.6), A_PLAN * rng.uniform(0.3, 1.0)
        trips = rng.randint(1, 3)
        leg = move_time(2 * amp, vmax, amax) + rng.uniform(0.0, 1.0)
        targets = []
        for _ in range(trips):
            targets += [x0 + amp] * int(leg / T) + [x0 - amp] * int(leg / T)
        targets += [x0] * int((move_time(amp, vmax, amax) + 0.5) / T)
        path = follow(np.array(targets), x0, 0.0, vmax, amax)
        p.update(amplitude_m=amp, speed_m_s=vmax, amax=amax, round_trips=trips)
    elif kind == "resonance":
        f = f_nat * rng.uniform(0.85, 1.15)
        accel = rng.uniform(0.2, 1.5)
        amp = min(accel / (2 * math.pi * f) ** 2, amp_limit(f, room))
        cycles = rng.randint(6, 20)
        n = int(cycles / f / T) + 1
        t = np.arange(n) * T
        shape = amp * np.sin(2 * math.pi * f * t) * taper(n, int(1 / f / T))
        shape, scale = fit(shape, room)
        decay = rng.uniform(10, 25)
        path = np.concatenate([x0 + shape, np.full(int(decay / T), x0)])
        p.update(freq_hz=f, natural_freq_used_hz=f_nat, amplitude_m=amp * scale, cycles=cycles,
                 free_decay_s=decay)
        own_rest = True
    else:
        raise ValueError(f"unknown motion {kind}")
    if not own_rest:
        tail = rng.uniform(*TAIL_REST_S)
        path = np.concatenate([path, np.full(int(tail / T), path[-1])])
        p["tail_rest_s"] = tail
    return path, p, own_rest


def brake_path(x0, v0, extra_s=0.5):
    """Stop from speed v0 as fast as the plan allows, then hold."""
    amax = A_PLAN
    x_stop = x0 + math.copysign(v0 * v0 / (2 * amax), v0)
    n = int((abs(v0) / amax + extra_s) / T) + 2
    return follow(np.full(n, x_stop), x0, v0, V_PLAN, amax)


class Segment:
    def __init__(self, index, kind, path, params, t_start, open_ended=False):
        self.index, self.kind, self.path, self.params = index, kind, path, params
        self.t_start = t_start
        self.open_ended = open_ended
        self.duration = (len(path) - 1) * T
        self.max_swing = 0.0
        v, a = derivatives(path) if len(path) > 2 else (np.zeros(len(path)), np.zeros(len(path)))
        self.v, self.a = v, a
        self.peaks = (float(np.max(np.abs(path))), float(np.max(np.abs(v))), float(np.max(np.abs(a))))

    def at(self, ts):
        """Planned position, speed, acceleration at ts seconds into the segment."""
        f = max(ts, 0.0) / T
        i = int(f)
        if i >= len(self.path) - 1:
            return float(self.path[-1]), 0.0, 0.0
        w = f - i
        x = self.path[i] * (1 - w) + self.path[i + 1] * w
        return float(x), float(self.v[i] * (1 - w) + self.v[i + 1] * w), float(self.a[i])

    def done(self, ts):
        return not self.open_ended and ts >= self.duration


# =====================================================================================
# Hardware helpers
# =====================================================================================
def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f"Arduino sent {len(data)}/{count} reply bytes in {timeout_s:.2f}s")
    return bytes(data)


def send_motor_command(arduino, timer_command):
    payload = struct.pack("<i", int(timer_command))
    if arduino.write(payload) != len(payload):
        raise OSError("Arduino accepted fewer than 4 command bytes")


def timer_for_speed(v, hold_error_m):
    """Timer value for cart speed v (m/s, + = step count up). Inside the deadband the slowest
    command (1.9 steps/s) is pointed back toward the held position every cycle."""
    if abs(v) < SPEED_DEADBAND_M_S:
        return (65535 if hold_error_m > 0 else -65535), 0.0, 0.0, True
    step_hz = v / M_PER_STEP                         # + = step count up
    icr = max(1, min(65535, int(round(TIMER_STEP_HZ / abs(step_hz))) - 1))
    timer = -icr if step_hz > 0 else icr             # Arduino: negative timer counts up
    rate = math.copysign(TIMER_STEP_HZ / (icr + 1), step_hz)
    return timer, rate, rate * M_PER_STEP, False


def signed_count_delta(raw, reference):
    return (raw - reference + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def calibrate_hanging(esp32, reader, forced_channels):
    """Hanging reference per channel from HANGING_SAMPLES frames; also detects the arm channels."""
    packets = []
    deadline = time.monotonic() + HANGING_SAMPLES * T * 3 + 2
    while len(packets) < HANGING_SAMPLES and time.monotonic() < deadline:
        try:
            packets.append(reader.read_latest(esp32, timeout_s=0.1))
        except AngleProtocolError:
            pass
    if len(packets) < HANGING_SAMPLES // 2:
        raise RuntimeError(f"only {len(packets)} receiver frames in the calibration; is the receiver on "
                           f"{ESP32_PORT} running protocol v1 firmware?")
    cal = {}
    for ch in range(3):
        vals = np.array([p.angles[ch] for p in packets if p.statuses[ch] == VALID], dtype=float)
        info = {"valid_fraction": len(vals) / len(packets),
                "statuses": sorted({STATUS_NAMES[p.statuses[ch]] for p in packets})}
        if len(vals):
            ph = vals * RAD_PER_COUNT
            ref = (np.angle(np.mean(np.exp(1j * ph))) % (2 * np.pi)) / RAD_PER_COUNT
            dev = signed_count_delta(vals, ref)
            info.update(hanging_counts=float(ref), max_deviation_counts=float(np.max(np.abs(dev))),
                        std_counts=float(np.std(dev)), distinct_values=int(len(np.unique(vals))),
                        first=int(vals[0]), last=int(vals[-1]))
        cal[ch + 1] = info
    if forced_channels:
        channels = list(forced_channels)
    else:
        channels = [ch for ch in (1, 2, 3) if cal[ch]["valid_fraction"] >= 0.9]
    if not channels:
        raise RuntimeError("no receiver channel read valid while hanging; are the transmitters on?")
    for ch in channels:
        if cal[ch]["valid_fraction"] < 0.9:
            raise RuntimeError(f"channel {ch} valid in only {cal[ch]['valid_fraction']:.0%} of frames "
                               f"({cal[ch]['statuses']})")
    return channels, cal, len(packets)


# =====================================================================================
# Main
# =====================================================================================
stop_requests = 0


def on_sigint(signum, frame):
    global stop_requests
    stop_requests += 1
    if stop_requests >= 2:
        raise KeyboardInterrupt


def parse_args(argv):
    ap = argparse.ArgumentParser(description="Excite the cart in many ways forever and log everything.")
    ap.add_argument("--seed", type=int, default=SEED, help="random seed (repeat a run's motions)")
    ap.add_argument("--only", default="", help="comma list of motions to use, e.g. chirp,step,rest")
    ap.add_argument("--arms", default="", help="receiver channels that are arms, e.g. 1 or 1,2 (default: auto)")
    ap.add_argument("--no-raw", action="store_true", help="do not save esp32_raw.bin")
    return ap.parse_args(argv)


def main(argv=None):
    global terminal_log, stop_requests
    args = parse_args(sys.argv[1:] if argv is None else argv)
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(1, 1 << 31)
    rng = random.Random(seed)
    weights = dict(MOTION_WEIGHTS)
    if args.only:
        wanted = [w.strip() for w in args.only.split(",") if w.strip()]
        bad = [w for w in wanted if w not in MOTION_WEIGHTS]
        if bad:
            sys.exit(f"unknown motion(s) {bad}; choose from {list(MOTION_WEIGHTS)}")
        weights = {k: (max(v, 1) if k in wanted else 0) for k, v in MOTION_WEIGHTS.items()}
    forced = tuple(int(c) for c in args.arms.split(",") if c.strip()) if args.arms else ARM_CHANNELS
    if forced and any(c not in (1, 2, 3) for c in forced):
        sys.exit("--arms takes receiver channels 1, 2, 3")
    if not any(weights.values()):
        sys.exit("every motion weight is 0")

    reader = AnglePacketReader()
    esp32_port = arduino = recorder = None
    run_dir = None
    metadata = {}
    motor_command_sent = False
    stop_reason = "program exit"
    cycle = 0
    run_t0 = None
    stats = {"segments": {}, "missing_frames": 0, "step_glitches": 0, "governor_pauses": 0}
    exit_code = 0
    try:
        log("START", f"PendulumExcite {SCRIPT_VERSION}, seed {seed}")
        esp32_port = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
        log("START", "serial ports open; waiting 3 s for the Arduino to reset")
        time.sleep(3)
        arduino.reset_input_buffer()
        esp32_port.reset_input_buffer()

        # ---------------- run folder (so even the calibration bytes are saved)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        run_dir = RUN_LOG_DIRECTORY / f"excite_{stamp}"
        run_dir.mkdir(parents=True, exist_ok=True)
        terminal_log = open(run_dir / "terminal.log", "w")
        recorder = Recorder(run_dir, SAVE_RAW_ESP32_BYTES and not args.no_raw)
        esp32 = TeePort(esp32_port, recorder.raw_bytes)
        try:
            shutil.copy2(Path(__file__).resolve(), run_dir / "PendulumExcite_used.py")
        except OSError:
            pass
        with open(run_dir / "columns.txt", "w") as f:
            for name, _, meaning in SAMPLE_COLUMNS:
                f.write(f"{name}: {meaning}\n")
        log("LOG", f"saving to {run_dir}")

        # ---------------- hanging calibration
        while True:
            log("READY", "Motor power OFF. Centre the cart. Let the arm(s) hang completely still.")
            input("Press Enter when the arm(s) have stopped swinging: ")
            reader.discard_pending(esp32)
            channels, cal, n_cal = calibrate_hanging(esp32, reader, forced)
            worst = max(cal[ch]["max_deviation_counts"] for ch in channels)
            for ch in (1, 2, 3):
                c = cal[ch]
                if "hanging_counts" in c:
                    log("CAL", f"channel {ch}: hanging={c['hanging_counts']:.1f} counts, noise std={c['std_counts']:.2f}, "
                               f"max dev={c['max_deviation_counts']:.0f}, valid {c['valid_fraction']:.0%}"
                               f"{'  <- arm' if ch in channels else ''}")
                else:
                    log("CAL", f"channel {ch}: no valid readings ({', '.join(c['statuses'])})")
            if worst <= MAX_HANGING_DEVIATION_COUNTS:
                break
            log("CAL", f"arm(s) moved {worst:.0f} counts during calibration (limit {MAX_HANGING_DEVIATION_COUNTS}); try again")
        refs = {ch: cal[ch]["hanging_counts"] for ch in channels}
        log("CAL", f"arm channels {channels} ({'single' if len(channels) == 1 else len(channels)} arm"
                   f"{'' if len(channels) == 1 else 's'})")

        metadata = {
            "script": "PendulumExcite.py", "script_version": SCRIPT_VERSION, "seed": seed,
            "argv": sys.argv, "host": socket.gethostname(), "python": sys.version, "numpy": np.__version__,
            "pyserial": getattr(serial, "__version__", "?"), "platform": platform.platform(),
            "started": datetime.now().astimezone().isoformat(),
            "arm_channels": channels, "hanging_calibration": cal, "calibration_frames": n_cal,
            "motion_weights": weights,
            "settings": {k: (str(v) if isinstance(v, Path) else v) for k, v in globals().items()
                         if k.isupper() and isinstance(v, (int, float, str, bool, tuple, dict, Path, type(None)))
                         and not k.startswith("_")},
            "conventions": {
                "angle_sign": "rel/abs angle = -(raw - hanging reference) * 360/16384 deg (same sign as the repo's "
                              "controllers); arm k absolute = sum of channel angles up to arm k",
                "cart_sign": "+ = Arduino step count up = negative timer value",
                "metres_per_step": M_PER_STEP,
                "arduino_timer": "step rate = 125000/(|timer|+1) steps/s; +/-65535 is the slowest (1.9 steps/s), "
                                 "used as a +/-1 step dither to hold position; -1 (0xFFFFFFFF) = stop + disable",
                "cycle": "one cycle per receiver frame (100 Hz): read frame -> compute -> send timer -> read 2-byte step reply",
            },
            "columns": {name: meaning for name, _, meaning in SAMPLE_COLUMNS},
        }
        (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=1, default=str))
        recorder.event(None, 0, "calibration", json.dumps({"channels": channels, "references": refs}))

        log("READY", "Turn motor power ON. Keep the arm(s) hanging. Keep hands clear of the track.")
        input("Press Enter to start (Ctrl+C at any time stops the cart and saves): ")
        signal.signal(signal.SIGINT, on_sigint)
        stop_requests = 0

        # ---------------- first command: the reply's step count is the cart's zero
        reader.discard_pending(esp32)
        motor_command_sent = True
        send_motor_command(arduino, -65535)
        t_sent = time.monotonic()
        steps_raw = struct.unpack(">h", read_exact(arduino, 2, 0.3))[0]
        arduino.reset_input_buffer()
        t_reply = time.monotonic()
        steps_total, last_steps_raw = 0, steps_raw
        run_t0 = t_reply
        recorder.event(0.0, 0, "motor_on", f"first step reply {steps_raw} = zero")
        log("RUN", f"Arduino replied (step count {steps_raw} = cart zero). Running until Ctrl+C.")

        # ---------------- state
        deck = []
        seg_index = 0
        seg = None
        cart_m = 0.0
        v_sent = 0.0
        v_cmd_prev = 0.0
        hold_x = 0.0
        last_frame_t = None
        unwrapped = {ch: 0.0 for ch in channels}
        prev_rel = {}
        prev_abs, prev_rx_ms = None, None
        missing_since = None
        ok_since = time.monotonic()
        governor = False
        stopping = False
        f_nat = NATURAL_FREQ_GUESS_HZ
        periods = []
        last_cross_t, swing_peak_since_cross = None, 0.0
        prev_arm1 = prev_arm1_t = None
        last_move_t = 0.0
        recent_swing = []
        next_status = time.monotonic()
        status_cycle, status_t = 0, time.monotonic()
        max_loop_ms = 0.0
        last_rate = {}

        def finish_segment(t_now, reason):
            if seg is None:
                return
            ts = t_now - seg.t_start
            recorder.segment((seg.index, seg.kind, f"{seg.t_start:.4f}", f"{t_now:.4f}", f"{ts:.4f}",
                              "" if seg.open_ended else f"{seg.duration:.3f}", reason,
                              f"{seg.path[0]:.6f}", f"{seg.path[-1]:.6f}", f"{seg.peaks[0]:.6f}",
                              f"{seg.peaks[1]:.5f}", f"{seg.peaks[2]:.4f}", f"{seg.max_swing:.2f}",
                              json.dumps(seg.params, default=float)))
            stats["segments"][seg.kind] = stats["segments"].get(seg.kind, 0) + 1

        def start_segment(kind, t_now, path, params, open_ended=False):
            nonlocal seg, seg_index
            seg_index += 1
            seg = Segment(seg_index, kind, path, params, t_now, open_ended)
            recorder.event(t_now, cycle, "segment_start", f"{seg_index} {kind} {json.dumps(params, default=float)[:300]}")
            log("SEG", f"#{seg_index} {kind}" + ("" if open_ended else f" {seg.duration:.1f}s") +
                f" peak |x|={seg.peaks[0] * 1000:.0f}mm |v|={seg.peaks[1]:.2f}m/s |a|={seg.peaks[2]:.1f}m/s2")

        def next_motion(t_now, x0):
            nonlocal deck
            if seg is None:
                n = int(BASELINE_REST_S / T) + 1
                return start_segment("baseline", t_now, np.full(n, x0), {"x0": x0, "duration_s": BASELINE_REST_S})
            if not deck:
                deck = [k for k, w in weights.items() for _ in range(w)]
                rng.shuffle(deck)
            kind = deck[-1]
            if abs(x0) > 0.8 * PLAN_LIMIT_M or (kind in OSCILLATING and abs(x0) > PLAN_LIMIT_M / 3):
                path, params, _ = plan_motion("recenter", x0, rng, f_nat)
                return start_segment("recenter", t_now, path, params)
            deck.pop()
            path, params, _ = plan_motion(kind, x0, rng, f_nat)
            return start_segment(kind, t_now, path, params)

        # ================================================================ loop
        while True:
            cycle += 1
            t_read0 = time.monotonic()
            try:
                packet = reader.read_latest(esp32, timeout_s=0.03)
            except AngleProtocolError as exc:
                packet = None
                if missing_since is None:
                    recorder.event(t_read0 - run_t0, cycle, "frame_missing", str(exc))
            t_frame = time.monotonic()
            t_run = t_frame - run_t0
            sensor_read_ms = (t_frame - t_read0) * 1000
            period_ms = None if last_frame_t is None else (t_frame - last_frame_t) * 1000
            last_frame_t = t_frame

            # ---------------- angles
            rel = {1: None, 2: None, 3: None}
            frame_ok = packet is not None and all(packet.statuses[ch - 1] == VALID for ch in channels)
            if packet is not None:
                for ch in channels:
                    if packet.statuses[ch - 1] == VALID:
                        r = -signed_count_delta(packet.angles[ch - 1], refs[ch]) * 360.0 / ENCODER_COUNTS
                        rel[ch] = r
                        if ch in prev_rel:
                            unwrapped[ch] += wrap_deg(r - prev_rel[ch])
                        else:
                            unwrapped[ch] = r
                        prev_rel[ch] = r
            arm_abs = [None, None, None]
            rates = [None, None, None]
            if frame_ok:
                acc = 0.0
                for k, ch in enumerate(channels):
                    acc += unwrapped[ch]
                    arm_abs[k] = acc
                if prev_abs is not None and prev_rx_ms is not None:
                    dt = ((packet.receiver_ms - prev_rx_ms) & 0xFFFFFFFF) / 1000.0
                    if 0 < dt < 0.2:
                        for k in range(len(channels)):
                            rates[k] = (arm_abs[k] - prev_abs[k]) / dt
                prev_abs, prev_rx_ms = list(arm_abs), packet.receiver_ms
                missing_since = None
                swing = max(abs(wrap_deg(a)) for a in arm_abs[:len(channels)])
            else:
                stats["missing_frames"] += 1
                if missing_since is None:
                    missing_since = t_frame
                    ok_since = None
                swing = None
            if frame_ok and ok_since is None:
                ok_since = t_frame
            recent_swing.append((t_frame, swing if swing is not None else 0.0))
            while recent_swing and recent_swing[0][0] < t_frame - 1.0:
                recent_swing.pop(0)
            swing_amp = max(s for _, s in recent_swing) if recent_swing else 0.0

            # ---------------- cart position now
            # the Arduino counts the steps at the moment a command arrives, so cart_m is the
            # position at the last send time
            cart_est = cart_m + v_sent * max(0.0, t_frame - t_sent)
            if abs(cart_est) > ABORT_LIMIT_M or abs(cart_m) > ABORT_LIMIT_M:
                stop_reason = f"cart at {cart_m:+.3f} m passed ABORT_LIMIT_M={ABORT_LIMIT_M}"
                recorder.event(t_run, cycle, "abort_limit", stop_reason)
                raise RuntimeError(stop_reason)

            # ---------------- choose / switch segments
            if seg is not None:
                seg.max_swing = max(seg.max_swing, swing or 0.0)
            if stop_requests and not stopping:
                stopping = True
                why = "Ctrl+C" if stop_reason == "program exit" else stop_reason
                recorder.event(t_run, cycle, "stop_requested", f"{why}; braking to a stop")
                log("STOP", f"{why}: braking to a stop (Ctrl+C again = motor off now)")
                finish_segment(t_run, "stop_requested")
                start_segment("stop", t_run, brake_path(cart_est, v_cmd_prev), {"x0": cart_est, "v0": v_cmd_prev},
                              open_ended=True)
            elif not stopping:
                missing_for = 0.0 if missing_since is None else t_frame - missing_since
                if missing_for > MISSING_STOP_S:
                    stop_reason = f"angles missing for {missing_for:.1f} s"
                    stop_requests = max(stop_requests, 1)
                    recorder.event(t_run, cycle, "angles_missing_stop", stop_reason)
                    log("FAULT", f"{stop_reason}; stopping")
                elif not governor and ((swing is not None and swing > MAX_SWING_DEG) or missing_for > MISSING_PAUSE_S):
                    governor = True
                    stats["governor_pauses"] += 1
                    why = (f"swing {swing:.1f} deg > MAX_SWING_DEG" if swing is not None
                           else f"angles missing {missing_for:.2f} s")
                    recorder.event(t_run, cycle, "governor_on", why)
                    log("SAFE", f"{why}: holding the cart")
                    finish_segment(t_run, "governor")
                    start_segment("governor_hold", t_run, brake_path(cart_est, v_cmd_prev), {"x0": cart_est, "v0": v_cmd_prev,
                                  "why": why}, open_ended=True)
                elif governor:
                    held = t_run - seg.t_start
                    calm = (ok_since is not None and t_frame - ok_since > 0.5 and swing_amp < RESUME_SWING_DEG)
                    if (calm and held > 1.0) or held > GOVERNOR_MAX_HOLD_S:
                        governor = False
                        recorder.event(t_run, cycle, "governor_off", f"held {held:.1f} s, swing {swing_amp:.1f} deg")
                        log("SAFE", f"resuming after {held:.1f} s (swing {swing_amp:.1f} deg)")
                        finish_segment(t_run, "resumed")
                        next_motion(t_run, float(seg.path[-1]))
                elif seg is None or seg.done(t_run - seg.t_start):
                    x0 = 0.0 if seg is None else float(seg.path[-1])
                    finish_segment(t_run, "done")
                    next_motion(t_run, x0)

            # ---------------- reference and command
            ts = t_run - seg.t_start
            x_ref, v_ref, a_ref = seg.at(ts)
            x_next, _, _ = seg.at(ts + T)
            moving = abs(x_next - x_ref) > 1e-7
            if moving:
                last_move_t = t_run
            v_ff = (x_next - x_ref) / T
            v_fb = TRACK_GAIN_PER_S * (x_ref - cart_est)
            v_want = v_ff + v_fb
            v = v_want
            limiters = []
            if abs(v) > SPEED_MAX_M_S:
                v = math.copysign(SPEED_MAX_M_S, v)
                limiters.append("speed")
            dv_max = ACCEL_MAX_M_S2 * T
            if abs(v - v_cmd_prev) > dv_max:
                v = v_cmd_prev + math.copysign(dv_max, v - v_cmd_prev)
                limiters.append("accel")
            room = SOFT_LIMIT_M - math.copysign(cart_est, v) if v else SOFT_LIMIT_M
            v_soft = math.sqrt(2 * ACCEL_MAX_M_S2 * max(room - abs(v) * T, 0.0))
            if abs(v) > v_soft:
                v = math.copysign(v_soft, v)
                limiters.append("soft")
            if abs(v) >= SPEED_DEADBAND_M_S:
                hold_x = cart_est + v * T
            timer, step_rate, v_sent_new, dither = timer_for_speed(v, cart_est - hold_x)
            a_cmd = (v - v_cmd_prev) / T

            # ---------------- send, read the step count
            send_motor_command(arduino, timer)
            t_prev_sent, t_sent = t_sent, time.monotonic()
            try:
                reply = read_exact(arduino, 2, 0.1)
            except TimeoutError:
                stop_reason = ("Arduino stopped replying: a limit switch or the stop button was triggered "
                               "(or the USB link dropped)")
                recorder.event(t_run, cycle, "arduino_no_reply", stop_reason)
                raise
            t_reply = time.monotonic()
            arduino.reset_input_buffer()
            v_sent_old, v_sent = v_sent, v_sent_new
            v_cmd_prev = v
            steps_raw = struct.unpack(">h", reply)[0]
            d_steps = ((steps_raw - last_steps_raw + 32768) & 0xFFFF) - 32768
            last_steps_raw = steps_raw
            expected = abs(v_sent_old) / M_PER_STEP * (t_sent - t_prev_sent)
            glitch = abs(d_steps) > 2 * expected + 30
            if glitch:
                # the stepper only moves as commanded: a jump this big is the counter (INT3) counting
                # noise, so the position carries on from the commanded speed instead
                stats["step_glitches"] += 1
                trusted = int(round(v_sent_old * (t_sent - t_prev_sent) / M_PER_STEP))
                recorder.event(t_run, cycle, "step_glitch", f"count jumped {d_steps:+d} (now {steps_raw}), "
                               f"expected about {trusted:+d}; check the D5 -> D18 jumper (StepCountCheck.py)")
                if stats["step_glitches"] <= 5:
                    log("FAULT", f"step count jumped {d_steps:+d} in one cycle (now {steps_raw}); ignored")
                if stats["step_glitches"] > MAX_STEP_GLITCHES:
                    stop_reason = (f"{stats['step_glitches']} step-count glitches: the cart position cannot be "
                                   "trusted. Check the D5 -> D18 jumper with StepCountCheck.py")
                    stop_requests = max(stop_requests, 1)
                d_steps = trusted
            steps_total += d_steps
            cart_m = steps_total * M_PER_STEP

            # ---------------- free-swing frequency (arm 1, while the cart is still)
            a1 = arm_abs[0]
            if a1 is not None:
                a1w = wrap_deg(a1)
                swing_peak_since_cross = max(swing_peak_since_cross, abs(a1w))
                free = (t_run - last_move_t) > 1.5 and not governor
                t_a1 = packet.receiver_ms / 1000.0
                if prev_arm1 is not None and prev_arm1 < 0 <= a1w and a1w - prev_arm1 < 90:
                    # crossing time interpolated between the two frames (frames are 10 ms apart)
                    t_c = prev_arm1_t + (t_a1 - prev_arm1_t) * (-prev_arm1) / (a1w - prev_arm1)
                    if free and last_cross_t is not None and 2.0 < swing_peak_since_cross < 45.0:
                        per = t_c - last_cross_t
                        if 0.2 < per < 5.0:
                            periods.append(per)
                            periods = periods[-41:]
                            if len(periods) >= 5:
                                f_nat = 1.0 / float(np.median(periods))
                    last_cross_t = t_c if free else None
                    swing_peak_since_cross = 0.0
                prev_arm1, prev_arm1_t = a1w, t_a1

            t_end = time.monotonic()
            loop_ms = (t_end - t_frame) * 1000
            max_loop_ms = max(max_loop_ms, loop_ms)
            st = packet.statuses if packet is not None else (None, None, None)
            ages = packet.ages_ms if packet is not None else (None, None, None)
            raws = packet.angles if packet is not None else (None, None, None)
            recorder.sample((
                t_run, t_frame, time.time() - (t_end - t_frame), cycle, period_ms, sensor_read_ms,
                (t_sent - t_frame) * 1000, (t_reply - t_sent) * 1000, loop_ms,
                seg.index, seg.kind, ts, "move" if moving else "free", int(governor), int(stopping), int(frame_ok),
                None if packet is None else packet.sequence, None if packet is None else packet.receiver_ms,
                *(None if s is None else STATUS_NAMES[s] for s in st),
                *(None if a is None or a == UNKNOWN_AGE else a for a in ages),
                *raws,
                *(rel[c] for c in (1, 2, 3)),
                *(unwrapped[c] if (c in channels and c in prev_rel) else None for c in (1, 2, 3)),
                *arm_abs, *rates, swing_amp, f_nat,
                x_ref, v_ref, a_ref, v_ff, v_fb, v_want, v, a_cmd, "|".join(limiters),
                timer, step_rate, v_sent_new, hold_x,
                steps_raw, d_steps, steps_total, cart_m, cart_est, x_ref - cart_est, int(glitch),
                reader.crc_errors, reader.discarded_bytes, reader.sequence_gaps, reader.superseded_frames,
                stats["missing_frames"], stats["step_glitches"], recorder.dropped,
            ))

            if t_end >= next_status:
                rate_hz = (cycle - status_cycle) / max(t_end - status_t, 1e-6)
                angs = " ".join(f"arm{k + 1}={wrap_deg(a):+6.1f}" for k, a in enumerate(arm_abs[:len(channels)])
                                if a is not None) + " deg" if frame_ok else "angles MISSING"
                log("RUN", f"t={t_run:7.1f}s #{seg.index} {seg.kind:<13} {ts:5.1f}s cart={cart_m * 1000:+6.1f}mm "
                           f"v={v * 1000:+6.0f}mm/s {angs} swing={swing_amp:5.1f} f0={f_nat:.3f}Hz "
                           f"{rate_hz:4.0f}Hz loop<={max_loop_ms:4.1f}ms miss={stats['missing_frames']} "
                           f"glitch={stats['step_glitches']}")
                next_status = t_end + STATUS_INTERVAL_S
                status_cycle, status_t, max_loop_ms = cycle, t_end, 0.0

            if stopping and ts > 0.3 and abs(v) < 0.002 and abs(x_ref - seg.path[-1]) < 1e-9:
                if stop_reason == "program exit":
                    stop_reason = "Ctrl+C"
                finish_segment(t_run, "stopped")
                seg = None
                break
            if stopping and ts > 3.0:
                stop_reason = stop_reason if stop_reason != "program exit" else "Ctrl+C (brake timeout)"
                finish_segment(t_run, "stopped")
                seg = None
                break
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C (immediate)" if motor_command_sent else "Ctrl+C before start"
        log("STOP", stop_reason)
    except Exception as exc:
        if stop_reason == "program exit":
            stop_reason = f"{type(exc).__name__}: {exc}"
        exit_code = 1
        if isinstance(exc, TimeoutError) and "limit switch" in stop_reason:
            log("STOP", stop_reason)
            log("STOP", "Move the cart off the switch / release the stop button. Rerunning resets the Arduino.")
        else:
            log("FAULT", f"cycle {cycle}: {stop_reason}")
            traceback.print_exc()
            if terminal_log is not None:
                traceback.print_exc(file=terminal_log)
        if recorder is not None:
            recorder.event(None if run_t0 is None else time.monotonic() - run_t0, cycle, "fault", stop_reason)
    finally:
        signal.signal(signal.SIGINT, signal.default_int_handler)
        try:
            if arduino is not None and arduino.is_open:
                if motor_command_sent:
                    try:
                        send_motor_command(arduino, -1)      # 0xFFFFFFFF: timer off, motor disabled
                        arduino.flush()
                        log("STOP", "sent the Arduino stop command (motor disabled)")
                    except (serial.SerialException, OSError) as exc:
                        log("FAULT", f"could not send the stop command: {exc}")
                arduino.close()
            if esp32_port is not None and esp32_port.is_open:
                esp32_port.close()
        finally:
            if recorder is not None:
                if 'seg' in locals() and seg is not None:
                    try:
                        finish_segment(time.monotonic() - run_t0, "stopped: " + stop_reason)
                    except Exception:
                        pass
                recorder.event(None if run_t0 is None else time.monotonic() - run_t0, cycle, "stop", stop_reason)
                recorder.close()
                duration = 0.0 if run_t0 is None else time.monotonic() - run_t0
                metadata["summary"] = {"stop_reason": stop_reason, "cycles": cycle, "duration_s": duration,
                                       "ended": datetime.now().astimezone().isoformat(),
                                       "natural_freq_hz": locals().get("f_nat"),
                                       "free_swing_periods_s": locals().get("periods"),
                                       "crc_errors": reader.crc_errors, "sequence_gaps": reader.sequence_gaps,
                                       "receiver_restarts": reader.receiver_restarts, "log_dropped": recorder.dropped,
                                       **stats}
                (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=1, default=str))
                size = sum(p.stat().st_size for p in run_dir.iterdir()) / 1e6
                log("STOP", f"{cycle} cycles, {duration:.0f} s, motions {stats['segments']}")
                log("STOP", f"saved {run_dir} ({size:.1f} MB)")
            if terminal_log is not None:
                terminal_log.close()
                terminal_log = None
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
