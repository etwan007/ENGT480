import serial
import struct
import time

ESP32_PORT = "/dev/ttyUSB0"
BAUD_RATE = 921600

MAX_ANGLE = 16383
MISSING = 65535


def looks_like_packet(values):
    # Each slot must be either a real 14-bit angle or the missing-data marker.
    return all(value <= MAX_ANGLE or value == MISSING for value in values)


def main():
    esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.02)

    try:
        time.sleep(0.25)
        esp32.reset_input_buffer()  # Only once, after opening the port.

        stream = bytearray()
        last_print = 0.0

        print("Reading encoder data. Press Ctrl+C to stop.")

        while True:
            stream.extend(esp32.read(64))

            while len(stream) >= 6:
                angles = struct.unpack(">HHH", stream[:6])

                # A good packet has at least one live encoder value.
                if looks_like_packet(angles) and any(
                    value != MISSING for value in angles
                ):
                    now = time.monotonic()

                    # Print at 10 times/second so terminal output does not
                    # make the serial read fall behind.
                    if now - last_print >= 0.1:
                        print(
                            f"angle1={angles[0]:5d}  "
                            f"angle2={angles[1]:5d}  "
                            f"angle3={angles[2]:5d}"
                        )
                        last_print = now

                    # This was a valid six-byte boundary.
                    del stream[:6]
                    break

                # Shift one byte and try again until a valid boundary is found.
                del stream[0]

            # Avoid retaining junk indefinitely if the stream is corrupted.
            if len(stream) > 64:
                del stream[:-5]

    except KeyboardInterrupt:
        print("\nTest stopped.")

    finally:
        esp32.close()


if __name__ == "__main__":
    main()
