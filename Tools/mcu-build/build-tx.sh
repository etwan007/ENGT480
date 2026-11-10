#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
project_dir="$repo_root/Code/Wireless/Transmitters"
build_dir="$repo_root/.build"
pio_environment="seeed_xiao_esp32c3"
pio_build_dir="$project_dir/.pio/build/$pio_environment"

if ! command -v pio >/dev/null 2>&1; then
    echo "Error: PlatformIO CLI (pio) is not installed or not on PATH." >&2
    exit 1
fi

mkdir -p "$build_dir"

build_transmitter() {
    local transmitter_id="$1"
    local config_file="$project_dir/src/config.$transmitter_id.hpp"

    if [[ ! -f "$config_file" ]]; then
        echo "Error: Configuration file not found: $config_file" >&2
        exit 1
    fi

    cp "$config_file" "$project_dir/src/config.hpp"
    (cd "$project_dir" && pio run --environment "$pio_environment")

    if [[ ! -f "$pio_build_dir/firmware.bin" ]]; then
        echo "Error: PlatformIO did not produce $pio_build_dir/firmware.bin" >&2
        exit 1
    fi

    local transmitter_build_dir="$build_dir/$transmitter_id"
    mkdir -p "$transmitter_build_dir"
    rm -f "$transmitter_build_dir"/*.bin

    # Keep the firmware and supporting binary images under the transmitter folder.
    for artifact in "$pio_build_dir"/*.bin; do
        [[ -f "$artifact" ]] || continue
        cp "$artifact" "$transmitter_build_dir/"
    done
}

build_transmitter tx1
build_transmitter tx2

echo "Firmware images are available in $build_dir."
