# Double pendulum with receiver protocol v1

Run `python3 PythonMaster2_auto.py` from this folder on the Pi.
This is the main calibrated controller. The repository also retains
`PythonMasterControl2.py` as a compatibility launcher for the same controller.
Configure offsets, missing-reading limits, and CSV switches near the top of
`PythonMaster2_auto.py`.

Deploy `PythonMaster2_auto.py` and its runtime helpers to the Pi, including `angle_protocol.py`,
`angle_packet_fallback.py`, `run_csv_logger.py`, and `control_matrices2.txt`.
The receiver must run the matching [v1 firmware](../Wireless/ANGLE_PROTOCOL.md).
USB serial remains 921600 baud; the Arduino motor link remains 115200 baud.
The receiver sends 37-byte frames containing all three angle channels,
sequence, uptime, channel status, wireless age, and CRC. Only channels 1 and 2
are required for double-pendulum operation; channel 3 may be absent.

The previous fixed `correction` values in `PythonMasterControl2.py` are
replaced by startup calibration. With motor power off, center the cart and let
both arms hang motionless for the first prompt. Then turn motor power on and
hold both arms upright for the second prompt. Both required channels must
pass calibration and the upright check before any motor command is sent.
Fine-tune with `ARM1_OFFSET_DEG` and `JOINT2_OFFSET_DEG` in the calibrated
script. Positive offsets reduce the reported angle; arm 1's offset also
changes arm 2's absolute angle.

One unusable control-cycle reading is permitted by default and uses an
estimator prediction. A second consecutive unusable reading stops the motor,
even if the missing channel alternates. CRC/framing failures and wireless or
encoder failures all count toward this limit. Receiver reasons and ages are
recorded in the CSV along with serial error counters. Frames are resynchronized
and checked before angles are used. Motor mapping, model matrices, and gains
come from the existing calibrated controller.

Historical files are preserved in `Archive/Test/`; its controller still uses
the old six-byte format. See [archive notes](Archive/README.md). Do not run
archived controllers with the v1 receiver.

Automated checks live in `Tests/`. They stay in the repository for development
and are not required for balancing on the Pi. `PythonMasterMath2.py` and
`symbolic_model2.pkl` are model-generation tools; balancing reads only the
existing `control_matrices2.txt`. Recorded runs remain in `run_logs/`.

Hardware-free verification (NumPy installed):

```sh
python3 -m unittest discover -s Tests -t . -p 'test_*.py'
```

These checks simulate serial input; they do not establish balancing performance
on the physical double pendulum.

## Repository and Pi cleanup — October 16, 2026

`PythonMaster2_auto.py` is the main script for running the double pendulum on
both the repository checkout and the Pi. Its controller contents were preserved
when the repository's `PythonMasterControl2_autocalibrate.py` was renamed.
The repository's `PythonMasterControl2.py` is a compatibility launcher.

Five automated test modules were moved into `Tests/`, with their imports and
receiver fixture paths updated. The historical `Test/` snapshot was moved intact
to `Archive/Test/`. The archive README explains each file and records its
original SHA-256 checksum. All 37 automated checks passed after the move.

On the Pi, the old standalone `PythonMasterControl2.py` and
`PythonMaster2_autotest.py` were also moved into `Archive/`, alongside the
historical `Test/` snapshot. The Pi has its own `Archive/README.md` documenting
all six archived files. All 28 existing files were verified unchanged after
their moves, including all 13 recorded CSV runs. The main controller, runtime
helpers, current model tools, and active matrices stayed in place. These Pi
filesystem moves were performed separately from the repository changes; the
Pi-only archived scripts and recorded runs have not been copied into this repo.
