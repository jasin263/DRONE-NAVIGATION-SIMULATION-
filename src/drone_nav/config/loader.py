"""Configuration loader and validator for drone-nav."""

import json
from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from drone_nav.config.models import WarehouseConfig


class ConfigError(Exception):
    """Raised when configuration loading or validation fails."""

    pass


def load_config_raw(file_path: str | Path) -> dict[str, Any]:
    """Read a YAML or JSON file into a dictionary."""
    path = Path(file_path)
    if not path.exists():
        raise ConfigError(f"Configuration file not found: {path.resolve()}")

    try:
        with open(path, encoding="utf-8") as f:
            if path.suffix.lower() in [".yaml", ".yml"]:
                data = yaml.safe_load(f)
            elif path.suffix.lower() == ".json":
                data = json.load(f)
            else:
                # Try YAML first, then JSON
                content = f.read()
                try:
                    data = yaml.safe_load(content)
                except Exception:
                    data = json.loads(content)
    except Exception as exc:
        raise ConfigError(f"Failed to parse configuration file '{path}': {exc}") from exc

    if not isinstance(data, dict):
        raise ConfigError(
            f"Configuration root in '{path}' must be a dictionary/mapping, got {type(data).__name__}"
        )

    return data


def format_validation_error(error: ValidationError) -> str:
    """Format a Pydantic ValidationError into a readable, clear diagnostic message."""
    messages = []
    for err in error.errors():
        loc = " -> ".join(str(item) for item in err["loc"])
        msg = err["msg"]
        err_type = err["type"]
        messages.append(f"  * [{loc}]: {msg} (type: {err_type})")
    return "\n".join(messages)


def load_config(file_path: str | Path) -> WarehouseConfig:
    """Load and strictly validate warehouse configuration from YAML/JSON."""
    data = load_config_raw(file_path)
    try:
        return WarehouseConfig.model_validate(data)
    except ValidationError as exc:
        formatted = format_validation_error(exc)
        raise ConfigError(
            f"Configuration validation failed for '{file_path}':\n{formatted}"
        ) from exc


def get_json_schema() -> dict[str, Any]:
    """Generate the official JSON Schema from the Pydantic v2 WarehouseConfig model."""
    return WarehouseConfig.model_json_schema()


def save_json_schema(target_path: str | Path) -> Path:
    """Export the JSON schema to disk."""
    path = Path(target_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    schema = get_json_schema()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(schema, f, indent=2)
    return path
