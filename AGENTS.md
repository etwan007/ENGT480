# Triple Inverted Pendulum project

This repository contains software, simulation models, hardware documentation, and tools for the triple inverted pendulum project.

## Repository layout

```text
ENGT480/
|-- Code/
|   |-- hardware_info.toml       # Active hardware connection configuration
|   |-- SinglePendulum/          # Single-pendulum controller and its Python environment
|   |-- DoublePendulum/          # Double-pendulum controller, model tools, and checks
|   |-- Global/                  # Shared Python libraries used by controllers
|   |-- Wireless/                # Legacy wireless firmware and protocol; no longer updated
|   |-- ArduinoSlave/            # Arduino motor-control firmware
|   |-- Old_Code/                # Retained historical code
|   `-- RunReview/               # Run-log visualizer and example data
|-- engt-docs/                   # project documentation
|-- MATLAB Simulation/           # MATLAB scripts and Simulink model files
|-- Tools/                       # Project utilities, including Pi automation
```

## Code areas

- `Code/SinglePendulum/` contains the active single-pendulum Python controller. Its `pyproject.toml` and `uv.lock` define its environment. The root `Code/hardware_info.toml` selects the hardware connections.
- `Code/DoublePendulum/` contains the active double-pendulum controller, model-generation scripts, controller matrices, automated checks, and an `Archive/` of historical implementations.
- `Code/Global/` contains shared controller utilities and libraries, including physical parameters, CSV logging, and the asynchronous Sensor Client API and synchronous bridge. Use the Sensor Client API to interact with Sensor Server; keep reusable Python code here when it applies to multiple setups.
- `Code/Wireless/` contains legacy wireless sensor firmware, protocol documentation, and checks. It is no longer actively updated; use Sensor Client API calls to operate Sensor Server for future work.
- `Code/ArduinoSlave/` contains the Arduino firmware that receives motor commands.
- `Code/RunReview/` contains browser-based tools for inspecting run CSV files and example runs.
- `Code/Old_Code/` preserves earlier code; treat it as historical unless a task explicitly targets it.

## Other project resources

- `MATLAB Simulation/` holds MATLAB and Simulink models for controller design and simulation.
- `Wiring/` holds the project's wiring diagram.
- `engt-docs/` documentation for this project.
- `Tools/` Tool for build firmware and sync data on Raspberry Pi

## Python package management

The project uses `uv` to manage Python dependencies and environments. Python dependencies are declared in each project's `pyproject.toml`; use its `uv.lock` when present to reproduce the locked environment. Run `uv` commands from the corresponding project directory.

## Documentation

This project keep all documentation in `engt-docs`.