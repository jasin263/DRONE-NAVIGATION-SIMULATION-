"""Test suite for client Steel Coil Warehouse preset, 3D world twin, and image catalog."""

from fastapi.testclient import TestClient
import pytest

from drone_nav.config.loader import load_config
from drone_nav.scanning.coil_catalog import SteelCoilCatalog
from drone_nav.viewer.app import app
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def client() -> TestClient:
    return TestClient(app)


def test_steel_coil_preset_loading():
    """Verify steel coil preset loads and validates against WarehouseConfig schema."""
    cfg = load_config("configs/presets/steel_coil_warehouse.yaml")
    assert cfg.site.id == "WH-STEEL-COIL"
    assert "Coil" in cfg.site.name
    assert len(cfg.layout.aisles) == 3
    assert cfg.site.ceiling_height == 9.5


def test_steel_coil_world_generation():
    """Verify world generator builds correct number of coils, aisles, and landmarks."""
    cfg = load_config("configs/presets/steel_coil_warehouse.yaml")
    world = generate_world_from_config(cfg)

    summary = world.get_summary()
    assert summary.site_id == "WH-STEEL-COIL"
    assert summary.total_aisles == 3
    assert summary.total_racks == 6
    assert summary.total_bins == 84
    assert len(world.static_obstacles) == 8  # Structural pillars B10, B12, C14, etc.

    # Check that bins received coil metadata from client photos
    bins_with_coils = [b for b in world.bins.values() if b.coil_img_id is not None]
    assert len(bins_with_coils) == 84
    first_bin = next(iter(world.bins.values()))
    assert first_bin.coil_supplier is not None
    assert first_bin.coil_weight_kg is not None
    assert first_bin.coil_weight_kg > 1000.0


def test_steel_coil_catalog():
    """Verify client coil catalog lookup and metadata integrity."""
    catalog = SteelCoilCatalog()
    assert len(catalog) >= 19
    rec = catalog.get_by_img_id("IMG_5805")
    assert rec is not None
    assert rec.coil_id == "M30953188"
    assert rec.mill_supplier == "ArcelorMittal Calvert"
    assert rec.floor_bay == "R75"
    assert rec.column_landmark == "B12"


def test_api_client_coils_endpoint(client: TestClient):
    """Verify /api/client-coils returns client image metadata."""
    res = client.get("/api/client-coils")
    assert res.status_code == 200
    data = res.json()
    assert "summary" in data
    assert "coils" in data
    assert len(data["coils"]) >= 19
    assert any(c["mill_supplier"] == "ArcelorMittal Calvert" for c in data["coils"])
    assert any(c["mill_supplier"] == "Cleveland-Cliffs" for c in data["coils"])


def test_api_world_steel_coil_preset(client: TestClient):
    """Verify /api/world?config=steel_coil_warehouse returns 3D twin data with coil attributes."""
    res = client.get("/api/world?config=steel_coil_warehouse")
    assert res.status_code == 200
    data = res.json()
    assert data["site_id"] == "WH-STEEL-COIL"
    assert len(data["bins"]) == 84
    assert "coil_img_id" in data["bins"][0]
    assert "coil_supplier" in data["bins"][0]
    assert "static_obstacles" in data
