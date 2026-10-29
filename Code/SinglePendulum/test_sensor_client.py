"""Quickly inspect sensor_1 updates from the Sensor Server."""

import asyncio
import time

from sensor_client import SensorClient


SENSOR_NAME = "sensor_1"
MONITOR_SECONDS = 8


async def main() -> None:
    client = SensorClient()
    try:
        await client.subscribe(SENSOR_NAME)
        await client.start()
        print("Server acknowledged start; monitoring sensor updates.")

        previous_seq = client.seq
        previous_time = time.monotonic()
        end_time = previous_time + MONITOR_SECONDS

        while time.monotonic() < end_time:
            await asyncio.sleep(1.0)
            now = time.monotonic()
            seq = client.seq
            reading = client.sensors.get(SENSOR_NAME)

            if seq is None or previous_seq is None:
                update_rate = 0.0
            else:
                update_rate = ((seq - previous_seq) & ((1 << 64) - 1)) / (now - previous_time)

            if reading is None:
                print(f"connected={client.connected} seq={seq} no reading yet")
            else:
                print(
                    f"connected={client.connected} seq={seq} "
                    f"update_rate={update_rate:.1f}/s "
                    f"angle={reading.angle!r} rad_s={reading.rad_s!r} "
                    f"age={reading.age_us / 1000:.1f} ms"
                )

            previous_seq = seq
            previous_time = now
    finally:
        await client.close()
        print("Sensor client stopped.")


if __name__ == "__main__":
    asyncio.run(main())
