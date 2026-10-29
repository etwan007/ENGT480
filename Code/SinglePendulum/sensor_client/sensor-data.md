# Sensor Server WebSocket API

This document defines the external API that the Sensor Server uses to provide local clients with the latest sensor state. Its JSON message format is independent of the V3 binary protocol between ESP32 nodes and the server. A future UDS transport can reuse the same message structure.

## Connection

- WebSocket listen address: `server.websocket_bind_address` in `config.toml` under `[server]`, formatted as `IPv4:port`.
- Default: `127.0.0.1:8765`.
- WebSocket path: `/ws`. With default settings, clients connect to `ws://127.0.0.1:8765/ws`.
- Set the address to a specific local IPv4 address or `0.0.0.0` to listen on all IPv4 interfaces. Addresses other than `127.0.0.1` may expose the API to other hosts reachable through that interface.
- Each WebSocket frame carries exactly one JSON message. JSON is encoded on a single line with no newline character; WebSocket frames provide message boundaries.

## Subscription

After connecting, a client sends a subscription request. Names in `sensors` are the logical sensor names from the server configuration. The client can list one or more names; `"*"` means all configured sensors.

```json
{ "type": "subscribe", "sensors": ["sensor_1", "sensor_2"] }
```

After a successful subscription, the server immediately sends a full snapshot with `event` set to `"update"`. The snapshot contains every subscribed sensor for which a reading is available. A sensor that has not produced a reading yet is omitted from the `sensor` object. Whenever a subscribed sensor receives a new reading, the server sends an incremental update. The `sensor` object in an incremental frame contains only the sensor or sensors that changed. Clients should merge each incremental object into their local state.

Each client has a bounded queue for pending updates so a slow client cannot block sensor packet reception. A client that falls behind until its queue fills is disconnected; it should reconnect and subscribe again to receive a fresh full snapshot.

If a subscription includes an unconfigured sensor name, the server returns an error and does not establish the subscription:

```json
{
    "type": "error",
    "code": "unknown_sensor",
    "message": "unknown sensor: sensor_9"
}

```

## Measurement control

The server starts in idle mode. Sensors remain registered and synchronized, sample and send one angle per second. Send a control message on any WebSocket connection to change the global measurement mode:

```json
{"type":"start"}
```

The server replies `{"type":"control","status":"started"}`, resets the API `timestamp_us` origin, and commands every configured sensor to sample at the 1,000 Hz target. High-rate incremental angle updates then resume.

```json
{"type":"stop"}
```

The server replies `{"type":"control","status":"stopped"}` and commands every configured sensor back to one angle sample per second. Clock synchronization continues at 1 Hz. Stopping does not reset the API timestamp origin. The next `start` resets it again.

If no sensor packet arrives for 100 ms, the server sends a full snapshot to subscribers at 10 Hz until sensor packets resume. This refreshes `age_us` and keeps clients synchronized when a sensor or transport is temporarily silent.

The server sends WebSocket Ping frames periodically and expects Pong responses. If the client that most recently sent `start` or `stop` disconnects or stops responding, the server detects the lost control connection, sends `stop` to the sensors, and returns to idle mode.

## Update messages

Full snapshots and incremental updates use the same structure. `event` is `"update"` for both the initial full snapshot and subsequent updates. The first update after a subscription is the full snapshot; later updates are incremental.

```json
{
    "event": "update",
    "type": "sensor_data",
    "seq": 1024,
    "timestamp_us": 1234567890123,
    "sensor": {
        "sensor_1": { "angle": 0.12, "rad_s": 0.8, "age_us": 150 },
        "sensor_2": { "angle": -0.34, "rad_s": -0.2, "age_us": 220 },
        "sensor_3": { "angle": 0.56, "rad_s": null, "age_us": 180 },
        "sensor_4": { "angle": 1.24, "rad_s": 1.1, "age_us": 90 }
    }
}

```

| Field | Type | Definition |
| --- | --- | --- |
| `event` | string | Update event name. Always `"update"`. |
| `type` | string | Message type. Always `"sensor_data"`. |
| `seq` | unsigned 64-bit integer | Update message sequence on the current WebSocket connection. Starts at 1 and increments for each update sent. |
| `timestamp_us` | unsigned 64-bit integer | Timestamp in the server's monotonic clock, in microseconds. It wraps at the unsigned 64-bit maximum and is not Unix time. |
| `sensor` | object | Mapping from logical sensor names to their latest state. A full snapshot contains all subscribed sensors with available readings; an incremental message contains only sensors updated by that message. |
| `sensor.<name>.angle` | number or null | Angle in radians, normalized to `$[-\pi, \pi)$`. The MT6701 single-turn angle is wrapped into this range; zero and positive direction follow the sensor's physical orientation. No zero-offset calibration is applied. `null` indicates an invalid I2C reading. |
| `sensor.<name>.rad_s` | number or null | Signed angular velocity in radians per second, calculated from consecutive valid angles using device sample timestamps and the shortest signed angular difference across `$2\pi$`. Its magnitude is not limited to `$\pi$`; for example, `10.0` and `-20.0` are valid. It is `null` until two valid samples are available or when the sample interval is ambiguous. |
| `sensor.<name>.age_us` | unsigned 64-bit integer | Age of the reading when the message is sent, in microseconds: the server's current monotonic time minus the time it most recently received that reading. It wraps at the unsigned 64-bit maximum. |

All incrementing numeric values wrap at their field width: after the maximum value, the next value is zero. Clients should compare them using modular arithmetic. `seq` is the WebSocket update message sequence, not a sensor firmware sample sequence. Per-sensor sample sequence numbers are outside the scope of this API. For a full snapshot, `seq` identifies the snapshot message; incremental frames then increment it in send order. The sequence restarts at 1 after reconnecting.

## Errors

If the request's `type` is missing or is not `"subscribe"`, `"start"`, or `"stop"`, `sensors` is missing or empty on a subscription, or a sensor name is invalid, the server sends a JSON message with `type` set to `"error"`. The error message format is `{"type":"error","code":"<code>","message":"<description>"}`. The client can correct the request and send it again.

## Transport reuse

The API message definitions depend only on JSON objects; they do not depend on WebSocket control frames, line delimiters, or newline characters. A future UDS transport can reuse the same subscription and update JSON content and define message framing for UDS separately.
