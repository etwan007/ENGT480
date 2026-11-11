#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
env_file="$script_dir/.flash_env"

if [[ ! -f "$env_file" ]]; then
    echo "Error: Flash configuration not found: $env_file" >&2
    exit 1
fi

# The flash environment is a shell file containing sensor device paths.
# shellcheck source=/dev/null
source "$env_file"

case "${1:-}" in
    sensor_1)
        sensor_port="${sensor_1:-}"
        transmitter_id="tx1"
        ;;
    sensor_2)
        sensor_port="${sensor_2:-}"
        transmitter_id="tx2"
        ;;
    *)
        echo "Usage: $0 sensor_1|sensor_2" >&2
        exit 2
        ;;
esac

if [[ -z "$sensor_port" ]]; then
    echo "Error: No serial port is configured for $1 in $env_file" >&2
    exit 1
fi

if [[ -d "$sensor_port" ]]; then
    echo "Error: $sensor_port is a directory. Set $1 in $env_file to the full serial device path." >&2
    exit 1
fi

if [[ ! -e "$sensor_port" ]]; then
    echo "Error: Serial device does not exist: $sensor_port" >&2
    exit 1
fi

firmware_dir="$repo_root/.build/$transmitter_id"
factory_image="$firmware_dir/firmware.factory.bin"
bootloader_image="$firmware_dir/bootloader.bin"
partitions_image="$firmware_dir/partitions.bin"
application_image="$firmware_dir/firmware.bin"

if [[ -f "$factory_image" ]]; then
    flash_mode="factory"
elif [[ -f "$bootloader_image" && -f "$partitions_image" && -f "$application_image" ]]; then
    flash_mode="separate"
else
    echo "Error: No complete firmware image set found in $firmware_dir" >&2
    echo "Expected firmware.factory.bin or bootloader.bin, partitions.bin, and firmware.bin." >&2
    exit 1
fi

pio_core_dir="${PLATFORMIO_CORE_DIR:-${HOME:-}/.platformio}"
pio_python="$pio_core_dir/penv/bin/python"
pio_esptool="$pio_core_dir/packages/tool-esptoolpy/esptool.py"

if command -v esptool >/dev/null 2>&1; then
    if [[ "$flash_mode" == "factory" ]]; then
        esptool --chip esp32c3 -p "$sensor_port" write-flash 0x0 "$factory_image"
    else
        esptool --chip esp32c3 -p "$sensor_port" write-flash \
            0x0 "$bootloader_image" \
            0x8000 "$partitions_image" \
            0x10000 "$application_image"
    fi
elif command -v esptool.py >/dev/null 2>&1; then
    if [[ "$flash_mode" == "factory" ]]; then
        esptool.py --chip esp32c3 -p "$sensor_port" write_flash 0x0 "$factory_image"
    else
        esptool.py --chip esp32c3 -p "$sensor_port" write_flash \
            0x0 "$bootloader_image" \
            0x8000 "$partitions_image" \
            0x10000 "$application_image"
    fi
elif [[ -x "$pio_python" && -f "$pio_esptool" ]]; then
    if [[ "$flash_mode" == "factory" ]]; then
        "$pio_python" "$pio_esptool" --chip esp32c3 -p "$sensor_port" write_flash 0x0 "$factory_image"
    else
        "$pio_python" "$pio_esptool" --chip esp32c3 -p "$sensor_port" write_flash \
            0x0 "$bootloader_image" \
            0x8000 "$partitions_image" \
            0x10000 "$application_image"
    fi
else
    echo "Error: esptool was not found. Install it or install the PlatformIO esptool package." >&2
    exit 1
fi
