"""Unit tests for DRIMS CSV import, export, and round-trip verification."""

from pathlib import Path

import pytest

from drone_nav.config.loader import load_config
from drone_nav.world.csv_importer import (
    REQUIRED_CSV_COLUMNS,
    CSVImportError,
    export_drims_csv,
    import_drims_csv,
)
from drone_nav.world.generator import generate_world_from_config


def test_export_and_import_roundtrip(tmp_path: Path):
    cfg = load_config("configs/presets/small.yaml")
    original_world = generate_world_from_config(cfg)
    csv_file = tmp_path / "small_world.csv"

    export_drims_csv(original_world, csv_file)
    assert csv_file.exists()

    # Re-import into new world
    imported_world = import_drims_csv(csv_file)
    assert len(imported_world.bins) == len(original_world.bins)
    assert imported_world.site_id == original_world.site_id

    # Check bin attributes match
    for bin_id, orig_b in original_world.bins.items():
        assert bin_id in imported_world.bins
        imp_b = imported_world.bins[bin_id]
        assert imp_b.label_code == orig_b.label_code
        assert imp_b.bounds.x_min == pytest.approx(orig_b.bounds.x_min, abs=1e-3)
        assert imp_b.bounds.y_min == pytest.approx(orig_b.bounds.y_min, abs=1e-3)
        assert imp_b.bounds.z_min == pytest.approx(orig_b.bounds.z_min, abs=1e-3)


def test_import_missing_file():
    with pytest.raises(CSVImportError) as exc_info:
        import_drims_csv("non_existent_file.csv")
    assert "not found" in str(exc_info.value)


def test_import_missing_columns(tmp_path: Path):
    bad_csv = tmp_path / "missing_cols.csv"
    bad_csv.write_text("site_id,zone,aisle\nWH01,Z01,A01\n", encoding="utf-8")
    with pytest.raises(CSVImportError) as exc_info:
        import_drims_csv(bad_csv)
    assert "missing required columns" in str(exc_info.value).lower()


def test_import_empty_file(tmp_path: Path):
    empty_csv = tmp_path / "empty.csv"
    empty_csv.write_text("", encoding="utf-8")
    with pytest.raises(CSVImportError) as exc_info:
        import_drims_csv(empty_csv)
    assert "empty or has no header" in str(exc_info.value).lower()


def test_import_invalid_numeric_row(tmp_path: Path):
    bad_row = tmp_path / "bad_row.csv"
    header = ",".join(REQUIRED_CSV_COLUMNS)
    row = "WH01,Z01,A01,R01,1,1,WH01-LABEL,not_a_float,2.0,1.0,2.0,0.5,1.5"
    bad_row.write_text(f"{header}\n{row}\n", encoding="utf-8")
    with pytest.raises(CSVImportError) as exc_info:
        import_drims_csv(bad_row)
    assert "invalid data" in str(exc_info.value).lower()


def test_import_inverted_bounds_volume(tmp_path: Path):
    bad_vol = tmp_path / "bad_vol.csv"
    header = ",".join(REQUIRED_CSV_COLUMNS)
    # x_max (1.0) <= x_min (2.0)
    row = "WH01,Z01,A01,R01,1,1,WH01-LABEL,2.0,1.0,1.0,2.0,0.5,1.5"
    bad_vol.write_text(f"{header}\n{row}\n", encoding="utf-8")
    with pytest.raises(CSVImportError) as exc_info:
        import_drims_csv(bad_vol)
    assert "non-positive volume" in str(exc_info.value).lower()
