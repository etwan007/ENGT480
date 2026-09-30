"""Nonblocking, per-cycle CSV logging for the balancing controller."""

import csv
import queue
import sys
import threading
import time


COLUMNS = (
    "record_type", "elapsed_s", "cycle", "esp32_packet", "raw_angle_1",
    "raw_angle_2", "raw_angle_3", "upright_correction_counts",
    "measured_angle_rad", "measured_angle_deg", "cart_used_m",
    "cart_steps", "cart_m", "pred_angle_rad", "pred_angle_rate_rad_s",
    "pred_cart_m", "pred_cart_velocity_m_s", "est_angle_rad",
    "est_angle_rate_rad_s", "est_cart_m", "est_cart_velocity_m_s",
    "previous_force_n", "force_command_n", "ramp_fraction",
    "step_frequency_hz", "motor_timer_command", "sensor_read_ms",
    "position_read_ms", "loop_ms", "cycle_period_ms", "message",
)

DOUBLE_COLUMNS = (
    "record_type", "elapsed_s", "cycle", "esp32_packet",
    "raw_angle_1", "raw_angle_2", "raw_angle_3",
    "reference_upright_1_counts", "reference_aligned_2_counts",
    "measured_angle_1_rad", "measured_angle_1_deg",
    "measured_angle_2_rad", "measured_angle_2_deg",
    "joint_angle_2_rad", "joint_angle_2_deg", "cart_used_m",
    "cart_steps", "cart_m",
    "pred_theta1_rad", "pred_theta1_rate_rad_s",
    "pred_theta2_rad", "pred_theta2_rate_rad_s",
    "pred_cart_m", "pred_cart_velocity_m_s",
    "est_theta1_rad", "est_theta1_rate_rad_s",
    "est_theta2_rad", "est_theta2_rate_rad_s",
    "est_cart_m", "est_cart_velocity_m_s",
    "previous_force_n", "force_command_n",
    "cart_velocity_command_m_s", "ramp_fraction",
    "step_frequency_hz", "motor_timer_command",
    "reference_upright_1_next_counts",
    "sensor_read_ms", "position_read_ms", "loop_ms",
    "cycle_period_ms", "message",
)


class RunCsvLogger:
    """Queue immutable snapshots; only the writer thread touches the disk in a run."""

    def __init__(self, path, queue_capacity=20000, columns=COLUMNS):
        self.path = path
        self.columns = tuple(columns)
        self.queue = queue.Queue(maxsize=queue_capacity)
        self.dropped_samples = 0
        self.error = None
        self._stopping = threading.Event()
        self._thread = None
        self._file = None
        self._reason = "program exit"

    def start(self):
        # Fail before the motor starts if the CSV cannot be created.
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x", newline="", encoding="utf-8", buffering=1024 * 1024)
        try:
            csv.writer(self._file).writerow(self.columns)
        except BaseException:
            self._file.close()
            raise
        self._thread = threading.Thread(target=self._write, name="balance-csv-writer", daemon=True)
        self._thread.start()

    def sample(self, values):
        """Return immediately, including when storage is slower than the controller."""
        try:
            self.queue.put_nowait(values)
        except queue.Full:
            self.dropped_samples += 1

    def event(self, kind, message, elapsed_s=""):
        row = [""] * len(self.columns)
        row[0] = kind
        row[1] = elapsed_s
        row[-1] = message
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped_samples += 1

    def stop(self, reason):
        self._reason = reason
        self._stopping.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            return "CSV writer did not finish within 10 seconds; file may be incomplete"
        if self.error is not None:
            return f"CSV writer failed: {self.error}"
        if self.dropped_samples:
            return f"CSV complete, but {self.dropped_samples} samples were dropped because the queue filled"
        return None

    def _write(self):
        try:
            writer = csv.writer(self._file)
            next_flush = time.monotonic() + 1
            while not self._stopping.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=0.1)
                except queue.Empty:
                    row = None
                if row is not None:
                    writer.writerow(row)
                now = time.monotonic()
                if now >= next_flush:
                    self._file.flush()
                    next_flush = now + 1
            end_row = [""] * len(self.columns)
            end_row[0] = "stop"
            end_row[-1] = f"{self._reason}; dropped_samples={self.dropped_samples}"
            writer.writerow(end_row)
            self._file.flush()
        except BaseException as exc:
            self.error = exc
            print(f"CSV writer failed for {self.path}: {exc}", file=sys.stderr, flush=True)
        finally:
            self._file.close()
