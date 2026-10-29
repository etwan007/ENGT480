"""Probe the Sensor Server WebSocket API without using sensor_client."""

import asyncio
import json
import time

from websockets.asyncio.client import connect


URI = "ws://127.0.0.1:8765/ws"
SENSOR_SELECTION = "*"
MONITOR_SECONDS = 8.0


async def receive_message(websocket, timeout=None):
    raw = await asyncio.wait_for(websocket.recv(), timeout=timeout)
    message = json.loads(raw)
    if message.get("type") == "error":
        raise RuntimeError(f"server error: {message}")
    return message


def merge_update(message, readings, update_counts):
    if message.get("type") != "sensor_data" or message.get("event") != "update":
        return None
    received_at = time.monotonic()
    for name, value in message.get("sensor", {}).items():
        readings[name] = (value, received_at)
        if isinstance(value.get("age_us"), int) and value["age_us"] <= 80_000:
            update_counts[name] = update_counts.get(name, 0) + 1
    return message.get("seq")


def print_sensor_payload(message):
    if message.get("type") == "sensor_data" and message.get("event") == "update":
        payload = message.get("sensor")
        if isinstance(payload, dict) and payload:
            print("raw sensor data:", json.dumps(payload, ensure_ascii=False))


async def main():
    async with connect(URI, open_timeout=5) as websocket:
        await websocket.send(json.dumps({"type": "subscribe", "sensors": [SENSOR_SELECTION]}))
        print("Subscribed to all configured sensors; requesting active measurement.")
        await websocket.send(json.dumps({"type": "start"}))

        readings = {}
        update_counts = {}
        latest_seq = None
        last_raw_print = 0.0
        start_deadline = time.monotonic() + 10.0
        while time.monotonic() < start_deadline:
            try:
                message = await receive_message(websocket, timeout=1.0)
            except TimeoutError:
                continue
            now = time.monotonic()
            if now - last_raw_print >= 0.1:
                print_sensor_payload(message)
                last_raw_print = now
            update_seq = merge_update(message, readings, update_counts)
            if update_seq is not None:
                latest_seq = update_seq
            if message.get("type") == "control" and message.get("status") == "started":
                print("Received server start acknowledgment; reading the stream directly.")
                break
        else:
            raise TimeoutError("server did not acknowledge the start request")

        interval_start = time.monotonic()
        interval_seq = latest_seq
        update_counts.clear()
        end_time = interval_start + MONITOR_SECONDS

        while time.monotonic() < end_time:
            remaining = end_time - time.monotonic()
            try:
                message = await receive_message(websocket, timeout=min(1.0, remaining))
            except TimeoutError:
                message = None

            if message is not None:
                now = time.monotonic()
                if now - last_raw_print >= 0.1:
                    print_sensor_payload(message)
                    last_raw_print = now
                update_seq = merge_update(message, readings, update_counts)
                if update_seq is not None:
                    latest_seq = update_seq

            now = time.monotonic()
            elapsed = now - interval_start
            if elapsed >= 1.0:
                seq = latest_seq
                if seq is None or interval_seq is None:
                    rate = 0.0
                else:
                    rate = ((seq - interval_seq) & ((1 << 64) - 1)) / elapsed

                if not readings:
                    print(f"ws_update_rate={rate:.1f}/s total; no sensor readings yet")
                else:
                    details = "; ".join(
                        f"{name}: rate={update_counts.get(name, 0) / elapsed:.1f}/s, "
                        f"angle={value.get('angle')!r}, rad_s={value.get('rad_s')!r}, "
                        f"age={(value['age_us'] + int((now - received_at) * 1_000_000)) / 1000:.1f} ms"
                        for name, (value, received_at) in sorted(readings.items())
                    )
                    print(f"ws_update_rate={rate:.1f}/s total; {details}")
                update_counts.clear()
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
