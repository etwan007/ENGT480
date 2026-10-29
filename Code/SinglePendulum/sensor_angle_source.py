"""Synchronous bridge from the async SensorClient to the control loop."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import threading
import time

from sensor_client import SensorClient, SensorReading


DEFAULT_SENSOR_URI = "ws://127.0.0.1:8765/ws"
SENSOR_NAME = "sensor_1"


@dataclass(frozen=True, slots=True)
class SensorAngleSample:
    angle: float | None
    rad_s: float | None
    age_us: int
    seq: int | None
    timestamp_us: int | None
    received_monotonic: float


class SensorAngleSource:
    """Run SensorClient on a private asyncio thread and expose fresh snapshots."""

    def __init__(self, uri=DEFAULT_SENSOR_URI, sensor_name=SENSOR_NAME):
        self.uri = uri
        self.sensor_name = sensor_name
        self._lock = threading.Lock()
        self._sample: SensorAngleSample | None = None
        self._ready = threading.Event()
        self._stopped = threading.Event()
        self._stop_requested = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._error: BaseException | None = None
        self._connected = False
        self._last_error: str | None = None
        self._seq: int | None = None
        self._timestamp_us: int | None = None

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    def start(self, timeout=15.0, first_reading_timeout=5.0) -> None:
        if self._thread is not None:
            raise RuntimeError("sensor source has already been started")
        self._thread = threading.Thread(
            target=self._thread_main, name="sensor-client-loop", daemon=True
        )
        self._thread.start()
        if not self._ready.wait(timeout):
            self.close()
            raise TimeoutError(f"sensor server did not start measurement within {timeout:.1f}s")
        if self._error is not None:
            error = self._error
            self.close()
            raise RuntimeError(f"could not start sensor client at {self.uri}: {error}") from error

        deadline = time.monotonic() + first_reading_timeout
        while time.monotonic() < deadline:
            sample = self.latest()
            if sample is not None and sample.angle is not None and sample.age_us <= 100_000:
                return
            if self._stopped.is_set():
                break
            time.sleep(0.002)
        detail = self._startup_reading_status()
        self.close()
        raise TimeoutError(f"sensor source did not provide a fresh reading: {detail}")

    def _startup_reading_status(self) -> str:
        with self._lock:
            sample = self._sample
            connected = self._connected
            last_error = self._last_error
            seq = self._seq
            timestamp_us = self._timestamp_us
        seq_text = "none" if seq is None else str(seq)
        timestamp_text = "none" if timestamp_us is None else str(timestamp_us)
        if sample is None:
            state = f"server has not supplied sensor_1; seq={seq_text}, timestamp_us={timestamp_text}"
        else:
            age_us = sample.age_us + int(
                max(0.0, time.monotonic() - sample.received_monotonic) * 1_000_000
            )
            if sample.angle is None:
                state = f"sensor_1 angle is null; age_us={age_us}"
            elif age_us > 100_000:
                state = f"sensor_1 reading is stale; age_us={age_us}"
            else:
                state = f"sensor_1 reading was not valid; age_us={age_us}"
        error_text = "none" if not last_error else last_error
        return f"{state}; connected={connected}; last_error={error_text}"

    def latest(self, *, max_age_us: int | None = None) -> SensorAngleSample | None:
        with self._lock:
            sample = self._sample
        if sample is None:
            return None
        age_us = sample.age_us + int(max(0.0, time.monotonic() - sample.received_monotonic) * 1_000_000)
        if max_age_us is not None and age_us > max_age_us:
            return None
        return SensorAngleSample(
            sample.angle, sample.rad_s, age_us, sample.seq, sample.timestamp_us,
            sample.received_monotonic,
        )

    def close(self, timeout=7.0) -> None:
        if self._thread is None:
            return
        self._stop_requested.set()
        if self._thread.is_alive():
            self._stopped.wait(timeout)
            self._thread.join(timeout=0.1)

    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve())
        except BaseException as error:
            self._error = error
            self._ready.set()
        finally:
            loop.close()
            self._stopped.set()

    async def _serve(self) -> None:
        client = SensorClient(uri=self.uri)
        try:
            try:
                await asyncio.wait_for(client.subscribe(self.sensor_name), timeout=10.0)
            except TimeoutError as error:
                detail = client.last_error or "no WebSocket connection was established"
                raise TimeoutError(
                    f"connection timed out after 10s; last transport error: {detail}. "
                    "Check that the Sensor Server is running and listening at this address"
                ) from error
            try:
                await asyncio.wait_for(client.start(), timeout=10.0)
            except TimeoutError as error:
                detail = client.last_error or "the server did not acknowledge the start request"
                raise TimeoutError(
                    f"start request timed out after 10s: {detail}"
                ) from error
            self._ready.set()
            last_signature = None
            while not self._stop_requested.is_set():
                reading: SensorReading | None = client.sensors.get(self.sensor_name)
                connected = client.connected
                last_error = client.last_error
                with self._lock:
                    self._connected = connected
                    self._last_error = last_error
                    self._seq = client.seq
                    self._timestamp_us = client.timestamp_us
                if reading is not None:
                    signature = (
                        reading.angle, reading.rad_s, reading.age_us,
                        client.seq, client.timestamp_us,
                    )
                    if signature != last_signature:
                        snapshot = SensorAngleSample(
                            reading.angle, reading.rad_s, reading.age_us,
                            client.seq, client.timestamp_us, time.monotonic(),
                        )
                        with self._lock:
                            self._sample = snapshot
                        last_signature = signature
                await asyncio.sleep(0.001)
        except BaseException as error:
            self._error = error
            with self._lock:
                self._last_error = str(error)
            self._ready.set()
        finally:
            try:
                await client.close()
            finally:
                with self._lock:
                    self._connected = False
                self._stopped.set()
