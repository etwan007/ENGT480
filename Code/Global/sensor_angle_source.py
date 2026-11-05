"""Synchronous bridge from the async SensorClient to the control loop."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import threading
import time
from collections.abc import Sequence

from Global.sensor_client import SensorClient, SensorReading


DEFAULT_SENSOR_URI = "ws://127.0.0.1:8765/ws"
DEFAULT_SENSORS = ("sensor_1",)


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

    def __init__(self, uri=DEFAULT_SENSOR_URI, sensors: Sequence[str] | str = DEFAULT_SENSORS):
        if isinstance(sensors, str):
            sensors = (sensors,)
        if not sensors or any(not isinstance(name, str) or not name for name in sensors):
            raise ValueError("sensors must contain one or more non-empty names")
        self.uri = uri
        self.sensors = tuple(dict.fromkeys(sensors))
        self._lock = threading.Lock()
        self._samples: dict[str, SensorAngleSample] = {}
        self._ready = threading.Event()
        self._stopped = threading.Event()
        self._stop_requested = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._error: BaseException | None = None
        self._connected = False
        self._last_error: str | None = None

    @property
    def sensor_name(self) -> str:
        """Return the only configured sensor name for single-sensor callers."""
        if len(self.sensors) != 1:
            raise ValueError("sensor_name is only available when one sensor is configured")
        return self.sensors[0]

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    def start(self, timeout=15.0, first_reading_timeout=5.0, settle_time=0.0) -> None:
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

        if settle_time > 0:
            time.sleep(settle_time)

        deadline = time.monotonic() + first_reading_timeout
        while time.monotonic() < deadline:
            if self._has_fresh_readings():
                return
            if self._stopped.is_set():
                break
            time.sleep(0.002)
        detail = self._startup_reading_status()
        self.close()
        raise TimeoutError(f"sensor source did not provide a fresh reading: {detail}")

    def _startup_reading_status(self) -> str:
        with self._lock:
            samples = dict(self._samples)
            connected = self._connected
            last_error = self._last_error
        statuses = []
        names = samples.keys() if self.sensors == ("*",) else self.sensors
        for name in names:
            sample = samples.get(name)
            if sample is None:
                statuses.append(f"server has not supplied {name}")
                continue
            age_us = sample.age_us + int(max(
                0.0, time.monotonic() - sample.received_monotonic
            ) * 1_000_000)
            if sample.angle is None:
                statuses.append(f"{name} angle is null; age_us={age_us}")
            elif age_us > 100_000:
                statuses.append(f"{name} reading is stale; age_us={age_us}")
        state = "; ".join(statuses) if statuses else "no fresh sensor readings"
        error_text = "none" if not last_error else last_error
        return f"{state}; connected={connected}; last_error={error_text}"

    def _has_fresh_readings(self) -> bool:
        names = self.sensors
        if names == ("*",):
            with self._lock:
                names = tuple(self._samples)
        return bool(names) and all(
            (sample := self.latest(name, max_age_us=100_000)) is not None
            and sample.angle is not None
            for name in names
        )

    def latest(
        self, sensor_name: str | None = None, *, max_age_us: int | None = None
    ) -> SensorAngleSample | None:
        if sensor_name is None:
            sensor_name = self.sensor_name
        with self._lock:
            sample = self._samples.get(sensor_name)
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
        client = SensorClient(uri=self.uri, sensors=self.sensors)
        try:
            try:
                await asyncio.wait_for(client.wait_connected(), timeout=10.0)
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
            last_signatures: dict[str, tuple] = {}
            while not self._stop_requested.is_set():
                connected = client.connected
                last_error = client.last_error
                with self._lock:
                    self._connected = connected
                    self._last_error = last_error
                for sensor_name, reading in client.sensors.items():
                    if not isinstance(reading, SensorReading):
                        continue
                    signature = (
                        reading.angle, reading.rad_s, reading.age_us,
                        client.seq, client.timestamp_us,
                    )
                    if signature != last_signatures.get(sensor_name):
                        snapshot = SensorAngleSample(
                            reading.angle, reading.rad_s, reading.age_us,
                            client.seq, client.timestamp_us, time.monotonic(),
                        )
                        with self._lock:
                            self._samples[sensor_name] = snapshot
                        last_signatures[sensor_name] = signature
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
