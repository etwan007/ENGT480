# Double-pendulum automated tests

Run from `Code/DoublePendulum`:

```sh
python3 -m unittest discover -s Tests -t . -p 'test_*.py'
```

Or from the repository root:

```sh
python3 -m unittest discover -s Code/DoublePendulum/Tests -t Code/DoublePendulum -p 'test_*.py'
```

The five test modules cover framed angle packets, missing-reading handling,
CSV recording, controller logging, and startup/stopping integration. Serial
devices are simulated. NumPy is required; a C++ compiler enables the actual
receiver encoder compatibility check. These checks do not command hardware.

The older `Test` folder was a historical controller/model snapshot, not this
automated suite. It is preserved separately under `../Archive/Test/`.
