"""Async client for the local Sensor Server WebSocket API."""

from __future__ import annotations

import asyncio
import json
from collections import deque
from dataclasses import dataclass
from typing import Any, Sequence

from websockets.asyncio.client import connect


@dataclass(frozen=True, slots=True)
class SensorReading:
    """Most recently received value for one sensor."""

    angle: float | None
    rad_s: float | None
    age_us: int


class SensorApiError(RuntimeError):
    """The server rejected a request or sent an invalid update."""


class SensorClient:
    """Maintain sensor values and control measurement through WebSocket.

    Subscription, start, or stop opens the connection as needed. Updates are
    read in a background task, and transport failures trigger reconnection.
    """

    def __init__(
        self,
        uri: str = "ws://127.0.0.1:8765/ws",
        sensors: Sequence[str] | str | None = None,
        *,
        reconnect_min_seconds: float = 0.25,
        reconnect_max_seconds: float = 5.0,
    ) -> None:
        if not uri.startswith(("ws://", "wss://")):
            raise ValueError("uri must start with ws:// or wss://")
        if reconnect_min_seconds <= 0 or reconnect_max_seconds < reconnect_min_seconds:
            raise ValueError("invalid reconnect delay range")

        self.uri = uri
        self.requested_sensors = self._validate_sensors(sensors) if sensors is not None else None
        self.reconnect_min_seconds = reconnect_min_seconds
        self.reconnect_max_seconds = reconnect_max_seconds

        self.sensors: dict[str, SensorReading] = {}
        self.seq: int | None = None
        self.timestamp_us: int | None = None
        self.connected = False
        self.last_error: str | None = None

        self._task: asyncio.Task[None] | None = None
        self._websocket: Any = None
        self._connected_event = asyncio.Event()
        self._send_lock = asyncio.Lock()
        self._subscription_generation = 1 if self.requested_sensors is not None else 0
        self._sent_subscription_generation = 0
        self._measuring_requested = False
        self._closing = False
        self._control_waiters: deque[tuple[str, asyncio.Future[None]]] = deque()

    async def subscribe(self, sensors: Sequence[str] | str) -> None:
        """Subscribe to sensor names, or ``"*"`` for every configured sensor."""
        self.requested_sensors = self._validate_sensors(sensors)
        self._subscription_generation += 1
        self.sensors.clear()
        await self._ensure_running()
        await self.wait_connected()
        await self._send_subscription_if_needed()

    async def start(self) -> None:
        """Start 1 kHz active measurement and wait for the server acknowledgement."""
        await self._set_measurement_mode(True)

    async def stop(self) -> None:
        """Stop active measurement, return sensors to 1 Hz idle, and await acknowledgement."""
        await self._set_measurement_mode(False)

    async def wait_connected(self, timeout: float | None = None) -> None:
        """Wait until the WebSocket connects and configured requests are sent."""
        if self._task is None:
            raise RuntimeError("call subscribe(), start(), or stop() before wait_connected()")

        task = self._task

        async def wait_for_connection() -> None:
            event_task = asyncio.create_task(self._connected_event.wait())
            try:
                done, _ = await asyncio.wait(
                    (event_task, task), return_when=asyncio.FIRST_COMPLETED
                )
                if event_task in done and self._connected_event.is_set():
                    return
                if self.last_error:
                    raise SensorApiError(self.last_error)
                raise RuntimeError("sensor client stopped before connecting")
            finally:
                event_task.cancel()
                await asyncio.gather(event_task, return_exceptions=True)

        waiter = wait_for_connection()
        if timeout is None:
            await waiter
        else:
            await asyncio.wait_for(waiter, timeout)

    async def close(self) -> None:
        """Request idle mode, then close the WebSocket and stop background reception."""
        task = self._task
        if task is None:
            return
        self._closing = True
        try:
            if not task.done() and self.connected:
                await asyncio.wait_for(self.stop(), timeout=5.0)
        except Exception as error:
            self.last_error = str(error)
        finally:
            if self._task is task:
                self._task = None
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self.connected = False
            self._connected_event.clear()

    async def _ensure_running(self) -> None:
        if self._closing:
            if self._task is None or self._task.done():
                raise RuntimeError("sensor client is closed")
            return
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="sensor-websocket-client")

    async def _send_subscription_if_needed(self) -> None:
        async with self._send_lock:
            websocket = self._websocket
            if (
                websocket is None
                or not self.connected
                or self.requested_sensors is None
                or self._sent_subscription_generation == self._subscription_generation
            ):
                return
            await websocket.send(
                json.dumps({"type": "subscribe", "sensors": list(self.requested_sensors)})
            )
            self._sent_subscription_generation = self._subscription_generation

    async def _set_measurement_mode(self, measuring: bool) -> None:
        await self._ensure_running()
        await self.wait_connected()
        status = "started" if measuring else "stopped"
        request_type = "start" if measuring else "stop"
        loop = asyncio.get_running_loop()
        waiter = loop.create_future()

        async with self._send_lock:
            websocket = self._websocket
            if websocket is None or not self.connected:
                raise ConnectionError("WebSocket disconnected before control request was sent")
            self._measuring_requested = measuring
            pending = (status, waiter)
            self._control_waiters.append(pending)
            try:
                await websocket.send(json.dumps({"type": request_type}))
            except Exception:
                self._remove_control_waiter(pending)
                raise

        try:
            await waiter
        finally:
            self._remove_control_waiter((status, waiter))

    async def _run(self) -> None:
        delay = self.reconnect_min_seconds
        while True:
            try:
                async with connect(self.uri, open_timeout=5) as websocket:
                    self._websocket = websocket
                    async with self._send_lock:
                        if self.requested_sensors is not None:
                            await websocket.send(
                                json.dumps(
                                    {
                                        "type": "subscribe",
                                        "sensors": list(self.requested_sensors),
                                    }
                                )
                            )
                            self._sent_subscription_generation = self._subscription_generation
                        if self._measuring_requested:
                            await websocket.send(json.dumps({"type": "start"}))
                    self.connected = True
                    self._connected_event.set()
                    self.last_error = None
                    delay = self.reconnect_min_seconds

                    async for message in websocket:
                        self._handle_message(message)

            except asyncio.CancelledError:
                raise
            except SensorApiError as error:
                self.last_error = str(error)
                self.connected = False
                self._connected_event.clear()
                return
            except Exception as error:
                self.last_error = str(error)
            finally:
                self.connected = False
                self._connected_event.clear()
                self._websocket = None
                self._fail_control_waiters()

            if self._closing:
                return
            await asyncio.sleep(delay)
            delay = min(delay * 2, self.reconnect_max_seconds)

    def _handle_message(self, raw_message: Any) -> None:
        try:
            message = json.loads(raw_message)
        except (TypeError, json.JSONDecodeError) as error:
            raise SensorApiError(f"invalid JSON from server: {error}") from error
        if not isinstance(message, dict):
            raise SensorApiError("server message must be a JSON object")

        if message.get("type") == "error":
            raise SensorApiError(
                f"server error {message.get('code', 'unknown')}: "
                f"{message.get('message', 'request rejected')}"
            )
        if message.get("type") == "control":
            self._acknowledge_control(message.get("status"))
            return
        if message.get("type") != "sensor_data" or message.get("event") != "update":
            return

        seq = message.get("seq")
        timestamp_us = message.get("timestamp_us")
        sensor_values = message.get("sensor")
        if not _is_nonnegative_int(seq) or not _is_nonnegative_int(timestamp_us):
            raise SensorApiError("update seq and timestamp_us must be non-negative integers")
        if not isinstance(sensor_values, dict):
            raise SensorApiError("update sensor field must be an object")

        parsed: dict[str, SensorReading] = {}
        for name, value in sensor_values.items():
            if not isinstance(name, str) or not isinstance(value, dict):
                raise SensorApiError("each sensor entry must be an object keyed by name")
            angle = value.get("angle")
            rad_s = value.get("rad_s")
            age_us = value.get("age_us")
            if angle is not None and (
                isinstance(angle, bool) or not isinstance(angle, (int, float))
            ):
                raise SensorApiError(f"sensor {name!r} angle must be a number or null")
            if not _is_nonnegative_int(age_us):
                raise SensorApiError(f"sensor {name!r} age_us must be a non-negative integer")
            if rad_s is not None and (
                isinstance(rad_s, bool)
                or not isinstance(rad_s, (int, float))
            ):
                raise SensorApiError(f"sensor {name!r} rad_s must be a number or null")
            parsed[name] = SensorReading(
                angle=None if angle is None else float(angle),
                rad_s=None if rad_s is None else float(rad_s),
                age_us=age_us,
            )

        self.sensors.update(parsed)
        self.seq = seq
        self.timestamp_us = timestamp_us

    def _acknowledge_control(self, status: Any) -> None:
        for expected_status, waiter in tuple(self._control_waiters):
            if expected_status == status and not waiter.done():
                waiter.set_result(None)
                self._control_waiters.remove((expected_status, waiter))
                return

    def _fail_control_waiters(self) -> None:
        while self._control_waiters:
            _, waiter = self._control_waiters.popleft()
            if not waiter.done():
                waiter.set_exception(ConnectionError("WebSocket closed before control acknowledgement"))

    def _remove_control_waiter(self, pending: tuple[str, asyncio.Future[None]]) -> None:
        try:
            self._control_waiters.remove(pending)
        except ValueError:
            pass

    @staticmethod
    def _validate_sensors(sensors: Sequence[str] | str) -> tuple[str, ...]:
        if isinstance(sensors, str):
            sensors = (sensors,)
        if not sensors or any(not isinstance(name, str) or not name for name in sensors):
            raise ValueError("sensors must contain one or more non-empty names")
        return tuple(dict.fromkeys(sensors))


def _is_nonnegative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
