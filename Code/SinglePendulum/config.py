"""Load and validate the SinglePendulum hardware configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "hardware_info.toml"


class ConfigError(ValueError):
    """Raised when hardware_info.toml is missing or has invalid values."""


@dataclass(frozen=True, slots=True)
class AppConfig:
    motor_serial: str
    motor_baudrate: int
    sensor_ws_url: str


def _section(data: dict[str, Any], name: str, path: Path) -> dict[str, Any]:
    value = data.get(name)
    if not isinstance(value, dict):
        raise ConfigError(f"{path}: expected a [{name}] table")
    return value


def _required_string(section: dict[str, Any], key: str, table: str, path: Path) -> str:
    value = section.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f'{path}: [{table}].{key} must be a non-empty string')
    return value.strip()


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> AppConfig:
    """Read the hardware config; paths are independent of the current directory."""
    config_path = Path(path).expanduser().resolve()
    try:
        with config_path.open("rb") as config_file:
            data = tomllib.load(config_file)
    except FileNotFoundError as exc:
        raise ConfigError(f"hardware config file not found: {config_path}") from exc
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError(f"invalid TOML in {config_path}: {exc}") from exc

    motor = _section(data, "motor", config_path)
    sensor_ws = _section(data, "sensor_ws", config_path)
    serial_port = _required_string(motor, "serial", "motor", config_path)
    sensor_ws_url = _required_string(sensor_ws, "ws", "sensor_ws", config_path)

    baudrate = motor.get("baudrate", 115200)
    if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
        raise ConfigError(f"{config_path}: [motor].baudrate must be a positive integer")

    return AppConfig(
        motor_serial=serial_port,
        motor_baudrate=baudrate,
        sensor_ws_url=sensor_ws_url,
    )
