# Receiver protocol v1 and missing-reading diagnostics

The ESP32 receiver and current Python controllers must be deployed together.
The older six-byte serial format is not supported by the new readers. With an
old receiver, the new calibrated controllers fail during setup before sending a
motor command. Use `PythonMaster_autocalibrate.py` for the single pendulum. For the double,
`PythonMasterControl2.py` now launches `PythonMaster2_auto.py`,
so either entry point uses the framed reader and startup calibration. Manual
offsets and logging switches live in the calibrated script.
`CommunicationTest.py`, `PythonMaster.py`, and historical/backup controllers
(including `DoublePendulum/Archive/Test/`) still use the old format and must not be
used with this receiver firmware.

## Deployment

1. With motor power off, open `ESP32_Code/Receivermaster/Receivermaster.ino` in
   Arduino IDE. Keep `angle_protocol.h` in the same sketch folder. Compile and
   upload for the actual receiver board, using Arduino ESP32 core 3.x (the
   receive callback uses the 3.x API, as did the preceding receiver).
2. Copy the updated controller folder to the Pi, including `angle_protocol.py`,
   `angle_packet_fallback.py`, `run_csv_logger.py`, and, for the single
   controller, `live_offset_keys.py`. Keep the existing double-controller
   `control_matrices2.txt` beside its script.
3. With motor power off, run `Code/SinglePendulum/AngleSensorDiagnostic.py`.
   Confirm the active channel reports `valid` with a changing raw angle and
   the inactive channels report `never_received` (or `wireless_timeout` if
   their transmitters were previously powered on).
4. Use the calibrated controller for the corresponding pendulum. Calibration
   and upright checks still precede the initial motor command. The single
   controller still stops on two consecutive unusable angle cycles; the double
   controller retains its configurable limit (one permitted miss by default).

TX1 and TX2 use the same three-byte ESP-NOW format and do not need reflashing
for this update. A fresh radio packet containing `0xFFFF` already reports an
encoder transaction failure in their existing firmware. TX1 now has an optional diagnostic firmware update: its local USB output
records encoder duration/failures, send-call errors, and delivery callbacks. See
[TX1 diagnostics](Transmitters/TX1/DIAGNOSTICS.md) for flashing, capture, and
interpretation. These counters are local to TX1 and are not added to protocol
v1 or the Pi CSV. TX2 retains its existing firmware.

## What changed

The receiver copies angle, arrival time, and a separate `seen` flag under the
same critical-section lock used by its receive callback. It reads the clock
AFTER that snapshot. This prevents a new receive timestamp from being compared
against an older clock value and underflowing into a false timeout. A timestamp
of zero is valid when `seen` is true. Unsigned subtraction still handles millis
rollover for ordinary short intervals.

The receiver sends at most one frame per 10 ms scheduling slot, skipping missed
slots after a stall. It writes a complete frame in one serial write. There is
no ASCII debugging text mixed into the binary serial stream.

The Python reader finds headers, validates the CRC and channel fields, and
retains partial frames. It returns the newest validated complete frame from the
available input; superseded frames are counted. It processes at most 4096 input
bytes per read and keeps less than one incomplete frame between calls. A larger
backlog is treated as an unusable reading rather than used as current data.
Prompt boundaries drain pending input before calibration/upright checks.

## Wire layout

USB serial remains 921600 baud. Each frame is exactly 37 bytes. All multi-byte
integers are big-endian; the packed native-endian ESP-NOW radio payload is
unchanged.

| Offset | Length | Field |
| --- | --- | --- |
| 0 | 4 | Magic bytes `A5 5A C3 3C` |
| 4 | 1 | Version, `1` |
| 5 | 1 | Payload length, `29` |
| 6 | 4 | Frame sequence, unsigned 32-bit, wraps naturally |
| 10 | 4 | Receiver uptime in ms, unsigned 32-bit |
| 14 | 7 | Channel 1: raw angle (2), status (1), radio age ms (4) |
| 21 | 7 | Channel 2: same layout |
| 28 | 7 | Channel 3: same layout |
| 35 | 2 | CRC-16/CCITT-FALSE of bytes 4 through 34 |

CRC uses polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final
XOR. The check value for ASCII `123456789` is `0x29B1`. The magic is excluded.

| Status | Name | Meaning |
| --- | --- | --- |
| 0 | `valid` | Angle 0–16383 and radio age at most 50 ms |
| 1 | `encoder_read_failed` | Fresh radio packet contained `0xFFFF` |
| 2 | `wireless_timeout` | Last radio packet is older than 50 ms |
| 3 | `never_received` | No accepted packet from this sender since receiver boot |

Unavailable angles are always `0xFFFF`. The age for `never_received` is
`0xFFFFFFFF`, displayed as unknown and stored as a blank CSV cell. Timeout takes
precedence over the last packet's encoder status: it means that communication
has become stale, regardless of whether the previous encoder read succeeded.
Malformed radio lengths, sender IDs, and out-of-range angles are rejected.

## Logs and CSV

A missing channel now reports, for example:

```text
ANGLE1_MISSING ... sequence=89325 angle1 reason=encoder_read_failed radio_age_ms=2 ...
ANGLE1_MISSING ... sequence=89325 angle1 reason=wireless_timeout radio_age_ms=57 ...
```

`ms_since_last_valid_Pi_read` is still a separate host-side time, not radio age.
A framing/CRC deadline failure is reported as `no_valid_frame`; a backlog error
is `serial_backlog`. These are serial transport failures, not receiver channel
statuses. On such a failure, the channel status/age cells are blank rather than
repeating the previous packet's values. The fault/fallback message records the
transport error.

CSV schema version 2 adds `esp32_sequence`, `receiver_uptime_ms`,
`angle_status_1`–`angle_status_3`, `radio_age_1_ms`–`radio_age_3_ms`, and cumulative
`serial_crc_errors`, `serial_discarded_bytes`, `serial_sequence_gaps`, and
`serial_superseded_frames`. Sequence gaps include frames lost or rejected on
the serial path; they do not directly count ESP-NOW losses. Superseded frames
are valid frames deliberately skipped while selecting the newest sample.
The existing visualizer reads columns by name and supports both CSV versions.

## Verification

From the single-controller folder, run `python3 -m unittest discover -p 'test_*.py'`.
From the double-controller folder, run `python3 -m unittest discover -s Tests -t . -p 'test_*.py'`.
The protocol tests compile `tests/receiver_protocol_test.cpp` with a host C++
compiler when available. That test includes the actual receiver sketch with
mock ESP32 APIs, injects a receive update between clock capture and snapshot,
checks timeout boundaries, boot-time zero, rollover, scheduling, and malformed
radio rejection. Python decodes a fixture emitted by the actual C++ encoder.
Other tests cover every frame split, inserted/deleted/corrupted bytes, bounded
backlogs, statuses, CSV fields, and the controller fallback/stop paths.
A host simulation is not a hardware validation or a substitute for compiling
and uploading for the actual board.
