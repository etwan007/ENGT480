import serial
import time

PORT = "/dev/ttyUSB0"
ACTIVE_SENDERS = 1  # Change to 3 when all three XIAOs are running.

previous_seq = None
last_summary = 0.0

with serial.Serial(PORT, 921600, timeout=1) as master:
    print("Listening. Press Ctrl+C to stop.")

    while True:
        line = master.readline()
        if not line:
            print("NO SERIAL DATA for 1 second")
            continue

        try:
            payload, checksum_text = line.rstrip(b"\r\n").rsplit(b"*", 1)

            checksum = 2166136261
            for byte in payload:
                checksum = ((checksum ^ byte) * 16777619) & 0xFFFFFFFF

            if checksum != int(checksum_text, 16):
                print("SERIAL CHECKSUM FAIL:", repr(line))
                continue

            fields = payload.decode("ascii").split(",")
            if len(fields) != 11 or fields[0] != "D":
                raise ValueError("Unexpected record format")

            values = list(map(int, fields[1:]))
            sequence = values[0]

        except (ValueError, UnicodeError):
            print("MALFORMED SERIAL RECORD:", repr(line))
            continue

        if previous_seq is not None:
            expected = (previous_seq + 1) & 0xFFFFFFFF
            if sequence != expected:
                print(f"SEQUENCE GAP/RESET: {previous_seq} -> {sequence}")

        previous_seq = sequence
        summaries = []
        alerts = []

        for sender in range(ACTIVE_SENDERS):
            start = 1 + 3 * sender
            received, age, outgoing = values[start:start + 3]
            label = f"XIAO {sender + 1}"

            summaries.append(
                f"{label}: RX={received} age={age}ms OUT={outgoing}"
            )

            if age == -999999:
                alerts.append(f"{label}: NO PACKET RECEIVED YET")
            else:
                if age < 0:
                    alerts.append(f"{label}: MASTER TIMESTAMP RACE")
                elif age > 50:
                    alerts.append(f"{label}: RECEIVE GAP >50ms")

                if received == 65535:
                    alerts.append(f"{label}: MASTER RECEIVED ERROR VALUE 65535")
                elif received > 16383:
                    alerts.append(f"{label}: MASTER RECEIVED OUT-OF-RANGE VALUE")
                elif received == 16383:
                    alerts.append(f"{label}: MASTER ALREADY RECEIVED 16383")

        now = time.monotonic()
        if alerts or now - last_summary >= 1:
            print(f"seq={sequence} | " + " | ".join(summaries))
            for alert in alerts:
                print("  " + alert)
            last_summary = now
