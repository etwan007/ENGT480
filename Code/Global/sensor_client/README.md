# Sensor Client API

Async client for the Sensor Server WebSocket API. Call `subscribe()` to connect and receive sensor data. Call `start()` to request active 1,000 Hz measurement, `stop()` to return sensors to idle 1 Hz, and `close()` to stop measurement before disconnecting.

## `SensorClient`

| Call | Purpose and return value |
| --- | --- |
| `SensorClient(uri="ws://127.0.0.1:8765/ws", sensors=None, *, reconnect_min_seconds=0.25, reconnect_max_seconds=5.0)` | Creates a client. `uri` is the WebSocket URL; optional `sensors` is one name, a sequence of names, or `"*"` for all configured sensors. Reconnect arguments set retry delay bounds in seconds. Returns a `SensorClient`; invalid arguments raise `ValueError`. |
| `await client.subscribe(sensors)` | Opens the connection as needed and subscribes to one sensor name, a sequence of names, or `"*"`. Returns `None` after the request is sent; update messages populate `client.sensors`. The server has no subscription-success acknowledgement, so check `last_error` if it rejects the request. |
| `await client.start()` | Requests that the server switch configured sensors to the 1000 Hz active sampling target. Returns `None` after the API acknowledges with `started`; this is not confirmation that each physical sensor received the command. After an unexpected disconnect, the client repeats this request when it reconnects if measurement was active. |
| `await client.stop()` | Requests that the server switch sensors to 1 Hz idle sampling while clock synchronization continues. Returns `None` after the API acknowledges with `stopped`; this is not confirmation that each physical sensor received the command. |
| `await client.wait_connected(timeout=None)` | Waits until the socket connects and any constructor-configured subscription is sent. Returns `None`; `timeout` is seconds. Raises `asyncio.TimeoutError` on timeout or `RuntimeError` if called before starting a connection. |
| `await client.close()` | If connected, sends `stop` and waits up to five seconds for its acknowledgement, then cancels background reception and closes the WebSocket without reconnecting. If disconnected, closes immediately; the server's controller-disconnect handling returns sensors to idle. Returns `None` and preserves cached readings. A stop error or timeout is saved in `last_error`. Safe if no connection was started; a connected client is closed permanently. |

The caller must run these methods inside an asyncio event loop. Calling `subscribe()`, `start()`, or `stop()` opens the background connection as needed. After an unexpected transport disconnect, the client reconnects with exponential backoff and resends the previous subscription. If the last requested mode was active measurement, it sends `start` again after reconnecting; otherwise the server remains in idle mode. Calling `close()` is intentional and disables reconnection.

## `SensorClient` properties

| Property | Type | Meaning |
| --- | --- | --- |
| `client.sensors` | `dict[str, SensorReading]` | Latest values keyed by sensor name. Full and incremental updates are merged; sensors omitted from an update remain cached. |
| `client.connected` | `bool` | True while the WebSocket is connected; false while disconnected or reconnecting. Does not guarantee a new sample has arrived. |
| `client.seq` | `int \| None` | Sequence number of the latest update; `None` before the first update. Restarts on a new WebSocket connection. |
| `client.timestamp_us` | `int \| None` | Server monotonic timestamp of the latest update, in microseconds; `None` before the first update. Not Unix time. |
| `client.last_error` | `str \| None` | Most recent connection or API error; cleared after a successful connection and subscription. |
| `client.requested_sensors` | `tuple[str, ...] \| None` | Deduplicated subscribed names, or `None` before a subscription is selected. |

## `SensorReading` fields

| Field | Type | Meaning |
| --- | --- | --- |
| `reading.angle` | `float \| None` | Latest angle in radians, normalized to $[-\pi, \pi)$ ; zero and positive direction follow the sensor's physical orientation, with no zero-offset calibration. `None` indicates an invalid I2C reading. |
| `reading.rad_s` | `float \| None` | Latest signed angular velocity in radians per second. Its magnitude is not limited to $\pi$; `None` means the server cannot calculate it yet. |
| `reading.age_us` | `int` | Age of the angle reading, in microseconds, when the server sent the update. |

## Errors

`SensorApiError` - an API or malformed-update error detected by the background receiver. The receiver stops and records its message in `client.last_error`; it is not synchronously raised by `subscribe()`. `start()` and `stop()` may raise `ConnectionError` if the connection closes before the control acknowledgement arrives.
