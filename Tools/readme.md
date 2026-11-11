# Tool

## MCU Firmware Build

### Setup

Setup env

```bash
./Tools/mcu-build/setup-pio.sh
```

### Build ESP TX (Legacy) Firmware

Build ESP TX (legacy) Firmware

```bash
./Tools/mcu-build/build-tx-legacy.sh
```

## MCU Firmware Flash

### env config

`Tools/mcu-flash/.flash_env`

```bash
sensor_1="/dev/serial/by-id/sensor_1"
sensor_2="/dev/serial/by-id/sensor_2"
```

### Flash Sensor (legacy)

Flash ESP TX (legacy) Firmware

Flash Sensor 1

```bash
./Tools/mcu-flash/flash-tx-legacy.sh sensor_1
```

Flash Sensor 2

```bash
./Tools/mcu-flash/flash-tx-legacy.sh sensor_1
```
