---
title: Sensor Client API
description: Async Python client for subscribing to Sensor Server readings.
sidebar_position: 5
---

# Sensor Client API

`Global.sensor_client` provides an asynchronous WebSocket client for Sensor Server. Use it from an `asyncio` event loop. The server message schema is documented separately in [Sensor Server data format](./sensor-data.md).

## Connect and read

```python
import asyncio

from Global.sensor_client import SensorClient


async def main():
    client = SensorClient(
        uri="ws://127.0.0.1:8765/ws",
        sensors=("sensor_1", "sensor_2"),
    )
    await client.start()
    await client.wait_connected(timeout=5)
    reading = client.sensors.get("sensor_1")
    if reading is not None:
        print(reading.angle, reading.rad_s, reading.age_us)
    await client.close()


asyncio.run(main())
```

`SensorClient` accepts one sensor name, a sequence of names, or `"*"` for all configured sensors. The default endpoint is `ws://127.0.0.1:8765/ws`.

## Methods

| Method | Behavior |
| --- | --- |
| `subscribe(sensors)` | Connects as needed and sends a subscription. The server does not acknowledge subscriptions; check `last_error` for rejection. |
| `start()` | Requests active measurement (1000 Hz target) and waits for the server's `started` acknowledgement. This confirms the API accepted the request, not that every physical sensor reached that rate. |
| `stop()` | Requests idle measurement (1 Hz target) and waits for the `stopped` acknowledgement. |
| `wait_connected(timeout=None)` | Waits for the socket and constructor-configured subscription. Raises `asyncio.TimeoutError` on timeout and `RuntimeError` if no connection has been started. |
| `close()` | Stops measurement when connected, waits up to five seconds for acknowledgement, closes the socket, and disables reconnection. It preserves cached readings. |

Calling `subscribe()`, `start()`, or `stop()` starts the connection when needed. After an unexpected disconnect, the client reconnects with exponential backoff and restores the subscription. If active measurement was requested, it requests `start` again after reconnecting. A deliberate `close()` does not reconnect.

## Readings and status

| Property | Type | Meaning |
| --- | --- | --- |
| `client.sensors` | `dict[str, SensorReading]` | Latest readings by sensor name. Full and incremental updates are merged; omitted sensors retain their cached values. |
| `client.connected` | `bool` | Socket is connected; this does not guarantee a new sample has arrived. |
| `client.seq` | `int | None` | Latest update sequence, reset on a new WebSocket connection. |
| `client.timestamp_us` | `int | None` | Server monotonic timestamp in microseconds, not Unix time. |
| `client.last_error` | `str | None` | Latest connection, API, or malformed-update error. |
| `client.requested_sensors` | `tuple[str, ...] | None` | Deduplicated sensor names, or `None` before selecting a subscription. |

`SensorReading` exposes `angle` in radians, `rad_s` in radians per second, and `age_us`. Angle or velocity can be `None` when the server cannot provide a valid value. See the [data format guide](./sensor-data.md) for units and timestamp semantics.

## Errors

`SensorApiError` is detected by the background receiver and saved in `client.last_error`; it is not synchronously raised by `subscribe()`. `start()` and `stop()` can raise `ConnectionError` if the socket closes before the control acknowledgement arrives. `close()` saves stop errors and timeouts in `last_error`.
