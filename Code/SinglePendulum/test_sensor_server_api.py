"""Probe the Sensor Server WebSocket API without using sensor_client."""

import asyncio
import json
import time

from websockets.asyncio.client import connect


URI = "ws://127.0.0.1:8765/ws"
SENSOR_NAME = "sensor_1"
MONITOR_SECONDS = 8.0


async def receive_message(websocket, timeout=None):
    raw = await asyncio.wait_for(websocket.recv(), timeout=timeout)
    message = json.loads(raw)
    if message.get("type") == "error":
        raise RuntimeError(f"server error: {message}")
    return message


def report_update(message):
    if message.get("type") != "sensor_data" or message.get("event") != "update":
        return None
    value = message.get("sensor", {}).get(SENSOR_NAME)
    if value is None:
        return None
    return message.get("seq"), value


async def main():
    async with connect(URI, open_timeout=5) as websocket:
        await websocket.send(json.dumps({"type": "subscribe", "sensors": [SENSOR_NAME]}))
        print(f"Subscribed to {SENSOR_NAME}; requesting active measurement.")
        await websocket.send(json.dumps({"type": "start"}))

        latest = None
        start_deadline = time.monotonic() + 10.0
        while time.monotonic() < start_deadline:
            try:
                message = await receive_message(websocket, timeout=1.0)
            except TimeoutError:
                continue
            update = report_update(message)
            if update is not None:
                latest = update
            if message.get("type") == "control" and message.get("status") == "started":
                print("Received server start acknowledgment; reading the stream directly.")
                break
        else:
            raise TimeoutError("server did not acknowledge the start request")

        interval_start = time.monotonic()
        interval_seq = latest[0] if latest else None
        end_time = interval_start + MONITOR_SECONDS

        while time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            try:
                message = await receive_message(websocket, timeout=min(1.0, remaining))
            except TimeoutError:
                message = None

            if message is not None:
                update = report_update(message)
                if update is not None:
                    latest = update

            now = time.monotonic()
            elapsed = now - interval_start
            if elapsed >= 1.0:
                seq = latest[0] if latest else None
                if seq is None or interval_seq is None:
                    rate = 0.0
                else:
                    rate = ((seq - interval_seq) & ((1 << 64) - 1)) / elapsed

                if latest is None:
                    print(f"update_rate={rate:.1f}/s no {SENSOR_NAME} reading")
                else:
                    _, value = latest
                    age_ms = value["age_us"] / 1000
                    print(
                        f"update_rate={rate:.1f}/s angle={value.get('angle')!r} "
                        f"rad_s={value.get('rad_s')!r} age={age_ms:.1f} ms"
                    )
                interval_start = now
                interval_seq = seq

        await websocket.send(json.dumps({"type": "stop"}))
        stop_deadline = time.monotonic() + 5.0
        while time.monotonic() < stop_deadline:
            try:
                message = await receive_message(websocket, timeout=1.0)
            except TimeoutError:
                continue
            if message.get("type") == "control" and message.get("status") == "stopped":
                print("Received server stop acknowledgment.")
                break


if __name__ == "__main__":
    asyncio.run(main())
