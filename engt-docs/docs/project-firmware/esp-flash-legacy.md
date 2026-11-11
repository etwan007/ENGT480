---
title: Flash the Legacy ESP32-C3 Transmitters
description: Build and flash the ENGT 480 legacy TX1 and TX2 sensor firmware.
sidebar_label: Legacy ESP32-C3 Flashing
---

# Flash the Legacy ESP32-C3 Transmitters

This procedure builds and flashes the legacy wireless transmitter firmware used by the ENGT 480 angle-sensor system. It covers the two transmitter configurations, **TX1** and **TX2**, which run on Seeed XIAO ESP32-C3 boards.

The firmware source is in [`Code/Wireless/Transmitters/`](https://github.com/etwan007/ENGT480/tree/main/Code/Wireless/Transmitters). The repository scripts build both transmitter images and flash one selected sensor at a time.

:::warning[Legacy firmware]

This page documents the retained ESP-NOW transmitter path, which sends the established three-byte angle packet. New software that communicates with Sensor Server should use the shared Sensor Client API. The receiver's newer serial framing does not change the transmitter radio packet, so that receiver update alone does not require reflashing TX1 or TX2. See the [receiver protocol notes](https://github.com/etwan007/ENGT480/blob/main/Code/Wireless/ANGLE_PROTOCOL.md) before changing receiver or controller firmware.

:::

## Hardware and mapping

| Flash target | Firmware configuration | ESP-NOW sender ID | Local port setting |
| --- | --- | ---: | --- |
| Sensor 1 | `config.tx1.hpp` | `0` | `sensor_1` |
| Sensor 2 | `config.tx2.hpp` | `1` | `sensor_2` |

Each transmitter is a Seeed XIAO ESP32-C3 connected to its angle encoder. Connect the board to the programming computer over USB. Identify each board's serial device before flashing; do not assume that a port number stays assigned to the same board after reconnecting it.

## Requirements

- A Linux shell with Bash, `uv`, and USB serial access to the boards.
- The repository checkout containing `Tools/mcu-build/` and `Tools/mcu-flash/`.
- A USB data cable for the XIAO ESP32-C3.
- The appropriate transmitter connected by USB, with its serial device path known.

Install the build and flashing tools using the repository setup script:

```bash
./Tools/mcu-build/setup-pio.sh
```

The script installs PlatformIO and esptool as `uv` tools. Confirm that PlatformIO is available:

```bash
pio --version
```

The flash script can use the `esptool` executable, the legacy `esptool.py` executable, or the esptool package installed in PlatformIO's core directory.

## Build firmware

From the repository root, run:

```bash
./Tools/mcu-build/build-tx-legacy.sh
```

The script builds both configurations for the `seeed_xiao_esp32c3` PlatformIO environment. It places the resulting binary images in:

```text
.build/tx1/
.build/tx2/
```

Build the firmware before flashing. The flash script uses the images already present in these directories; it does not build firmware itself.

## Configure serial ports

Create the local configuration file `Tools/mcu-flash/.flash_env`. This file is local to the machine and should contain the full serial device path for each board:

```bash
sensor_1="/dev/serial/by-id/<sensor-1-device>"
sensor_2="/dev/serial/by-id/<sensor-2-device>"
```

Replace each placeholder with the actual device path. On Linux, stable paths under `/dev/serial/by-id/` are preferable to transient names such as `/dev/ttyUSB0`. The port must identify a serial device, not a directory. If your system exposes devices elsewhere, use their full device paths instead.

Before flashing, check the configured paths and make sure no serial monitor or other program is using the selected device. The invoking user must have permission to open it.

## Flash a transmitter

Connect only the transmitter you intend to program, then run the command for its mapped target:

```bash
# Sensor 1 / TX1
./Tools/mcu-flash/flash-tx-legacy.sh sensor_1

# Sensor 2 / TX2
./Tools/mcu-flash/flash-tx-legacy.sh sensor_2
```

The script validates the selected serial path and checks for a complete set of firmware images. If `firmware.factory.bin` exists for the selected target, it is written at address `0x0`. Otherwise, the script writes the bootloader, partition table, and application at `0x0`, `0x8000`, and `0x10000`, respectively. It targets the ESP32-C3 explicitly.

:::danger Protect the rig during programming
Turn motor power off before connecting or flashing a sensor board. Keep the controller stopped while programming. Flash one board at a time and verify which sensor is connected before selecting `sensor_1` or `sensor_2`.
:::

## Verify the flash

Wait for esptool to finish and report that writing and verification completed without an error. If the script exits with an error, resolve it before disconnecting the board or trying the other target.

After flashing:

1. Disconnect and reconnect the USB cable so the board restarts.
2. Confirm that the transmitter is powered and that the receiver reports fresh readings for the expected channel.
3. If readings do not arrive, check the selected sensor mapping, encoder wiring, receiver channel, and radio configuration before reflashing.

The transmitter build script creates separate TX1 and TX2 firmware images. Flashing one target does not program the other board.

## Troubleshooting

| Message or symptom | Check |
| --- | --- |
| Flash configuration not found | Create `Tools/mcu-flash/.flash_env` with `sensor_1` and `sensor_2` device paths. |
| No serial port is configured | Set the variable for the selected target in `.flash_env`. |
| Serial device does not exist | Reconnect the board and update the path to its current device. |
| Serial path is a directory | Use the full device path, not the `/dev/serial/by-id/` directory itself. |
| No complete firmware image set | Run `./Tools/mcu-build/build-tx-legacy.sh` and confirm `.build/tx1/` or `.build/tx2/` contains firmware images. |
| esptool was not found | Run `./Tools/mcu-build/setup-pio.sh`, or install esptool in a way that makes it available to the flash script. |
| Port permission denied or busy | Close programs using the port and grant the current user access to the serial device. |
| Flash cannot connect to the chip | Check the USB data cable and connection. Confirm the board is an ESP32-C3 and retry with the board attached directly to the computer. |

## Firmware and protocol references

- [Transmitter source and PlatformIO configuration](https://github.com/etwan007/ENGT480/tree/main/Code/Wireless/Transmitters)
- [Legacy build script](https://github.com/etwan007/ENGT480/blob/main/Tools/mcu-build/build-tx-legacy.sh)
- [Legacy flash script](https://github.com/etwan007/ENGT480/blob/main/Tools/mcu-flash/flash-tx-legacy.sh)
- [Receiver protocol and deployment notes](https://github.com/etwan007/ENGT480/blob/main/Code/Wireless/ANGLE_PROTOCOL.md)
