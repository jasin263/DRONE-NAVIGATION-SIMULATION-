"""Unit tests for configuration loading and validation."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from drone_nav.config.loader import ConfigError, get_json_schema, load_config, load_config_raw
from drone_nav.config.models import WarehouseConfig

PRESETS_DIR = Path("configs/presets")


def test_presets_exist():
    assert (PRESETS_DIR / "small.yaml").exists()
    assert (PRESETS_DIR / "medium.yaml").exists()
    assert (PRESETS_DIR / "large.yaml").exists()


@pytest.mark.parametrize("preset_name", ["small", "medium", "large"])
def test_validate_all_presets(preset_name: str):
    cfg_path = PRESETS_DIR / f"{preset_name}.yaml"
    config = load_config(cfg_path)
    assert isinstance(config, WarehouseConfig)
    assert config.version == "1.0"
    assert config.site.id.startswith("WH-")
    assert len(config.layout.aisles) > 0
    assert len(config.layout.takeoff_landing_pads) > 0


def test_json_schema_export():
    schema = get_json_schema()
    assert isinstance(schema, dict)
    assert "properties" in schema
    assert "site" in schema["properties"]
    assert "layout" in schema["properties"]
    assert "drone" in schema["properties"]
    assert "localisation" in schema["properties"]
    assert "mission" in schema["properties"]
    assert "safety" in schema["properties"]
    assert "export" in schema["properties"]


def test_missing_file_raises_config_error():
    with pytest.raises(ConfigError) as exc_info:
        load_config("configs/presets/non_existent.yaml")
    assert "not found" in str(exc_info.value).lower()


def test_invalid_yaml_syntax(tmp_path: Path):
    bad_yaml = tmp_path / "bad.yaml"
    bad_yaml.write_text("site: [unbalanced brackets", encoding="utf-8")
    with pytest.raises(ConfigError) as exc_info:
        load_config(bad_yaml)
    assert "failed to parse" in str(exc_info.value).lower()


def test_reject_inverted_floor_bounds():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["site"]["floor_bounds"]["x_max"] = -5.0
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    err = str(exc_info.value)
    assert "x_max" in err


def test_reject_rack_level_count_mismatch():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    # Set levels to 4 while level_heights has 3 entries
    raw["layout"]["aisles"][0]["left_rack"]["levels"] = 4
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "level_heights" in str(exc_info.value)


def test_reject_non_increasing_level_heights():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["layout"]["aisles"][0]["left_rack"]["level_heights"] = [1.0, 0.8, 2.0]
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "strictly increasing" in str(exc_info.value)


def test_reject_one_way_with_bidirectional_direction():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["layout"]["aisles"][0]["one_way"] = True
    raw["layout"]["aisles"][0]["direction"] = "bidirectional"
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "bidirectional" in str(exc_info.value)


def test_reject_pad_outside_bounds():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    # Floor is 18m x 14m, put pad at (25m, 20m)
    raw["layout"]["takeoff_landing_pads"][0]["position"] = [25.0, 20.0, 0.0]
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "outside floor bounds" in str(exc_info.value)


def test_reject_aisle_outside_bounds():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["layout"]["aisles"][0]["x_end"] = 50.0
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "outside floor bounds" in str(exc_info.value)


def test_reject_critical_reserve_exceeding_safe():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["drone"]["battery"]["critical_reserve_pct"] = 25.0
    raw["drone"]["battery"]["safe_reserve_pct"] = 20.0
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "critical_reserve_pct must be less than safe_reserve_pct" in str(exc_info.value)


def test_reject_invalid_no_fly_zone():
    raw = load_config_raw(PRESETS_DIR / "small.yaml")
    raw["layout"]["no_fly_zones"][0]["polygon"] = [[0.0, 0.0], [1.0, 1.0]]  # only 2 points
    with pytest.raises(ValidationError) as exc_info:
        WarehouseConfig.model_validate(raw)
    assert "at least 3 vertices" in str(exc_info.value)
