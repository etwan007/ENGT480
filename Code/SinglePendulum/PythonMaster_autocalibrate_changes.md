# Changes from `PythonMaster.py` to `PythonMaster_autocalibrate.py`

This document compares the two files in `Code/SinglePendulum/`. The modified script is named `PythonMaster_autocalibrate.py` in this repository; a copy on the Raspberry Pi may be named `PythonMaster_auto.py`.

## What happens during a run now

1. Python opens the ESP32 serial port (`/dev/ttyUSB0`) and Arduino serial port (`/dev/ttyACM0`), waits three seconds for Arduino startup, and clears old incoming bytes. Opening the Arduino port and the three-second wait were already in the original script; the new script prints each startup stage.
2. With motor power **off**, center the cart and let pendulum 1 hang motionless. Press Enter. Python reads 50 angle packets, spaced 20 ms apart, and estimates the raw count for the hanging position.
3. Python adds half a revolution (8,192 counts) to estimate the upright count. It prints both counts. This measured value replaces the original fixed `correction = 10687.7`.
4. Turn motor power **on**, hold pendulum 1 upright, and press Enter. Python reads 10 more angles, takes their median, and checks that the measured angle is within **12 degrees** of its estimated upright. If it is outside that range, the script exits before sending an Arduino motor command.
5. Python sends the original initial command (`-65535`), reads the initial cart position, and starts the same state estimator and motor control calculation as before. It prints a status line at most twice per second.
6. Once both ports are open and the exit handler is registered, Ctrl+C or a Python error makes the script attempt to send the Arduino stop command, then close both serial ports.

## Exact changes

| Area | Original `PythonMaster.py` | New `PythonMaster_autocalibrate.py` |
| --- | --- | --- |
| Upright reference | Uses the fixed raw encoder value `10687.7`. | Measures the hanging value at each startup and calculates `(hanging + 8192) % 16384` as the estimated upright value. |
| Hanging check | None. | Uses a circular mean so readings near raw count 0 wrap correctly. Rejects calibration if any of the 50 readings is more than 80 counts (about 1.76 degrees) from that mean. Prints first/last values, number of distinct values, duration, and largest deviation. |
| Upright check | Reads one angle and immediately continues. | Takes the median of 10 angle readings and refuses to send the first motor command if the result is more than 12 degrees from estimated upright (about 546 counts). Prints the first/last raw readings and the measured angle. |
| Angle conversion | Subtracts the correction directly, divides by `16383`, and has a `>17000` filter that reuses the previous angle. | Computes the shortest signed difference around the 16,384-count circle and divides by `16384`. The reported angle is between -180 and +180 degrees relative to estimated upright. The old `>17000` filter and `angleLast` state are removed. |
| ESP32 angle data | Reads six bytes in a loop that can wait indefinitely. | Requires a complete six-byte packet within 0.1 seconds. Rejects values outside the 14-bit range and stops with an explicit error if angle 1 is `65535` (`0xFFFF`, meaning missing). Angles 2 and 3 may be missing in the single-pendulum setup. |
| Arduino position data | Reads two bytes in a loop that can wait indefinitely. | Requires two bytes within 0.1 seconds. Saves the raw step count for logging, then uses the same conversion to meters: `steps * 0.638175 / 6400`. |
| Arduino writing | Clears the Arduino output buffer before each control command. | Removes that output-buffer clear, which could discard a command waiting to be transmitted. Adds a 0.2-second serial write timeout. The command format remains a signed, four-byte, little-endian integer. |
| Console output | Mainly prints `Serial started` and a few error messages. | Prints elapsed time and a stage label (`START`, `READY`, `CAL`, `CHECK`, `RUN`, `SENSOR`, `FAULT`, or `STOP`). Run lines include cycle number, raw angle, tilt, raw cart steps, cart position, estimated cart speed, requested force, step frequency, timer command, observed loop rate, and longest cycle since the last status line. |
| Errors | Some missing data could leave Python waiting indefinitely; failures had little context. | Serial timeouts and invalid/missing angle data raise an error. The exception handler prints the current stage, cycle number, and Python traceback. |
| Shutdown | Ctrl+C calls `arduino.write(0xFFFFFFFF)`, which is not a valid byte payload for pySerial, then closes only the Arduino port. | If a motor command has been sent, the exit handler writes `struct.pack('<I', 0xFFFFFFFF)`, flushes it, and closes both the Arduino and ESP32 ports. Before the first motor command, it closes the ports without sending a stop command. |

## What stayed the same

- The physical model, parameter values, 100 Hz sample period, LQR gains, Kalman estimator, force ramp, force-to-step-frequency conversion, and initial `-65535` command are unchanged.
- The ESP32 still sends three 16-bit angle fields in one six-byte packet at 100 Hz. The script controls the cart using angle 1. The Arduino still receives a four-byte motor command and replies with a two-byte cart step count.
- The script still treats the Arduino's step count as cart position. This is a commanded-step count, not independent confirmation that the cart moved the same distance.

## What the change does not solve

- The hanging-angle calculation assumes upright is exactly half a revolution from hanging. It gives a starting estimate, not a precise mechanical calibration. A small upright error can still cause cart drift.
- `65535` for angle 1 can come from a failed encoder read at transmitter 1 or from the receiver not hearing that transmitter for over 50 ms. This serial packet does not identify which cause occurred.
- There is no software cart travel limit or Arduino watchdog in this Python change. A Pi crash or power loss cannot run Python's exit handler. The hardware limit switches remain the final travel protection.
- An Arduino limit-switch hit enters a permanent halt in `ArduinoSlave.ino`; it needs an Arduino reset before another run. The Python stop command disables the Arduino motor output, and the existing Arduino code does not re-enable its driver until setup runs again. This change does not remove the normal reset-and-center procedure between runs.

## How to read the console

- `CAL hanging=... estimated upright=...`: the two raw encoder references used for this run.
- `CHECK angle=...`: how close the held pendulum is to the estimated upright position before motor commands begin.
- `RUN tilt=... cart=...`: measured pendulum angle and reported cart position during control.
- `FAULT stage=reading ESP32 angle ... sent 65535`: angle 1 was marked missing by the ESP32 path; this is different from an Arduino limit-switch event.
- `SENSOR ANGLE1_MISSING`: printed immediately when a complete six-byte packet contains `65535` in angle 1, before Python raises the fault. It records local wall-clock time, run stage and cycle, complete-packet number, all six packet bytes, all three raw angle values, the previous valid angle, time since the previous valid **Pi read**, the last Arduino step count, and the last motor command. The previous-read time is not the wireless packet age. The script stops on the first missing angle; this log does not distinguish an encoder read failure from a wireless gap.
- `SENSOR ESP32_PACKET_TIMEOUT`: the Pi did not read a complete six-byte ESP32 packet within 0.1 seconds. The log includes the number of complete packets read before the timeout and the most recent angle, cart, and motor-command values. This is distinct from a complete packet containing `65535`.
- `FAULT stage=waiting for Arduino cart position`: Python sent a command but did not receive the expected two-byte Arduino reply within 0.1 seconds. A limit-switch halt is one possible cause.
- `STOP`: Python attempted its shutdown sequence. This message does not prove that the Arduino received or acted on the stop command if the Arduino had already halted or the serial connection had failed.
