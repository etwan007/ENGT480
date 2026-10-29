# Balancing run CSV and visualizer

In this repository, the controllers live in `Code/SinglePendulum/` and `Code/DoublePendulum/`; shared configuration and helpers live in `Code/Global/`. The visualizer and synthetic examples live beside this guide in `Code/RunReview/`.

## Record a run

`Code/SinglePendulum/PythonMaster.py` (single pendulum) and
`Code/DoublePendulum/PythonMaster2_auto.py` (double pendulum) have these settings near the top:

- `SAVE_RUN_CSV = True` records the run. Set it to `False` to disable CSV logging.
- `RUN_LOG_DIRECTORY` controls where files are saved. By default, the script creates a `run_logs` folder beside itself.
- `SHOW_BALANCING_LOGS = False` hides routine terminal output during the run while keeping CSV recording on. This also avoids terminal printing in the control loop.

After hanging-angle calibration, while motor power is still off, the script creates a uniquely named CSV. Single-pendulum files start `balance_`; double-pendulum files start `balance_double_`. This keeps file creation out of the motor-on startup path. The CSV contains the upright check, run configuration, an initial motor event, one `sample` row per completed control cycle, and a final `stop` row. Ctrl+C stops the motor and closes the CSV. Faults are recorded as `fault` events when the logger is running.

The loop only makes an in-memory snapshot and queues it without waiting. A separate thread formats and writes CSV rows. If storage cannot keep up and the 20,000-row queue fills, new samples are dropped instead of delaying motor control. The final `stop` row reports `dropped_samples=N`; gaps are also visible in the `cycle` column. There is still a small cost to collecting a snapshot, so strict zero overhead is not possible.

## Adjust the single-pendulum offset while balancing

Run `PythonMaster_auto.py` in a focused interactive terminal (a Pi terminal or SSH works). After the upright check, **Up/Right** increases the manual upright offset and **Down/Left** decreases it, with no Enter required. Each arrow press changes it by **0.05°**; holding an arrow repeats at the terminal's key-repeat rate. Change `LIVE_OFFSET_STEP_DEG` near the top of the script to choose another increment.

Positive offsets reduce the reported pendulum angle. Adjustments start from `UPRIGHT_OFFSET_DEG`, apply on the next control cycle, and last only for the current run. The regular status line shows `offset=...deg` when balancing output is enabled. The CSV records each change as a `manual_offset` event and includes the updated upright correction on each sample, even with terminal logs hidden. Ctrl+C still stops the run, and terminal input settings are restored on exit. Without an interactive terminal, balancing continues with the configured offset and arrow control is unavailable.

## View a run

Open `balancing_visualizer.html` in a browser, then choose or drag in the CSV. It works locally without an internet connection or a server. Use Play, the timeline slider, or the step buttons to review the motion. It detects either file format. For double-pendulum runs, it draws both arms using their absolute angles and plots both tilt angles together. The other plots show cart position, motor command, and any recorded column you select. The lower panel shows every CSV value for the selected sample and the recorded start, stop, and fault events.

The animation uses the reported cart position and measured angles. The cart scale is fitted to the recorded range. For the double pendulum, the second arm uses its absolute angle to vertical, so the two links join correctly. It is a replay of measured data, not a physics simulation.

## What is in each sample

- Timing: elapsed time since the motor-on Enter prompt, cycle number, sensor update sequence and timestamp, sensor and position read times, loop calculation time, and the actual period since the previous cycle started.
- Single-pendulum measurements: `sensor_1` angle in radians, angular velocity in radians per second, reading age in microseconds, calibrated angle in radians and degrees, cart step count, and cart position. `cart_used_m` is the position from the preceding Arduino response that the estimator used this cycle; `cart_m` is the new response after the motor command.
- Double-pendulum measurements: all three raw ESP32 angles, both calibrated angles, cart step count, and cart position.
- Controller state: the four predicted states, four corrected estimated states, previous and current force commands, force ramp fraction, requested step frequency, and integer motor timer command.
- Calibration and setup: the upright correction on every sample, plus model parameters and gain matrices in the `config` event.

Double-pendulum samples use a wider header with both measured absolute angles, the second joint angle, all three raw ESP32 channels, six predicted and six estimated states, the cart velocity command, and the arm 1 encoder reference before and after its centering adjustment. The `config` event includes the controller and estimator matrices.

The single-pendulum Sensor Server log uses schema version 3; its `config` event records the sensor name, WebSocket address, and angle sign. The CSV uses a fixed header. Event rows have `record_type` values such as `start`, `config`, `check`, `motor_start`, `fault`, and `stop`; cycle data has `record_type=sample`. Blank cells on event rows are expected.

For either controller, a `sensor_fallback` event marks a cycle that substituted a predicted angle because a required sensor reading was unusable or stale. The sample's `message` column records the same fallback. For the double pendulum, the controller still uses whichever of angle channels 1 and 2 was valid; two consecutive cycles with an unusable required channel stop the motor. During startup calibration and the upright check, unusable readings are retried within a small limit, and repeated failures abort before balancing.

## Receiver protocol and dropout reasons

The double-pendulum controller and legacy ESP32-based tools require receiver
protocol v1. Deploy the receiver and Python updates together; see
[protocol and deployment instructions](../Wireless/ANGLE_PROTOCOL.md).
Those CSV logs record each channel's status (`valid`,
`encoder_read_failed`, `wireless_timeout`, or `never_received`) and radio age,
plus receiver sequence/uptime and serial parsing counters. The `message` field
records transport failures and the reason for prediction fallback. Both older
and newer CSV files can be opened in the visualizer.


Shared Python configuration and utilities now live in `Code/Global/`.
Deploy the entire `Global` folder as a sibling of `SinglePendulum` and
`DoublePendulum`; see [shared directory guide](../Global/README.md).
