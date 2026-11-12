---
title: Single-pendulum controller
description: Set up and launch the single-pendulum controller.
sidebar_position: 2
---

# Single-pendulum controller

The active single-pendulum controller is in `Code/SinglePendulum/`. Its `pyproject.toml` and `uv.lock` define the Python environment.

## Requirements

- Python 3
- [`uv`](https://docs.astral.sh/uv/)
- Git

## Set up the environment

From `Code/SinglePendulum/`, install the locked dependencies:

```sh
uv sync --locked
```

## Run the controller

From the same directory, start the controller with:

```sh
uv run PythonMaster.py
```

## Check the Sensor Client connection

The repository includes a Sensor Client test script:

:::note

Before starting the test, make sure the sensor is connected properly. Otherwise, wireless performance may be compromised due to I2C read errors.

:::

```sh
uv run test_sensor_client.py
```

For the API contract and reading fields, see the [Sensor Client API](./sensor-client.md) and [Sensor Server data format](./sensor-data.md).
