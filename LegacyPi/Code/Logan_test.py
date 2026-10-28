import serial
import struct
esp32 = serial.Serial(
	'/dev/ttyUSB0',
	115200,
	timeout=0.2,
)

esp32.reset_input_buffer()
buffer = bytearray()

while True:
	buffer.extend(esp32.read(64))

	while True:
		start = buffer.find(b'\xA5\x5A')

		if start == -1:
                # Preserve a possible first header byte
			buffer = buffer[-1:]
			break

            # Discard bytes before the header
		del buffer[:start]

		if len(buffer) < 8:
			break

		packet = bytes(buffer[:8])
		del buffer[:8]

		a1, a2, a3 = struct.unpack(
			'>HHH', packet[2:]
		)

		if all(v <= 16383 or v == 65535 for v in (a1, a2, a3)):
			print(a1, a2, a3)
		else:
			print("BAD PACKET:", packet.hex(' '))
