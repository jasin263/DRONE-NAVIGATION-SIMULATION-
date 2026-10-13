"""Unit tests for world model generation, geometry, and marker placement."""

from pathlib import Path

from drone_nav.config.loader import load_config
from drone_nav.world.generator import generate_world_from_config
from drone_nav.world.models import World

PRESETS_DIR = Path("configs/presets")


def test_generate_small_world():
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)
    assert isinstance(world, World)
    assert world.site_id == "WH-SMALL"

    # Small preset: 3 aisles * 2 racks * 5 bays * 3 levels = 90 bins
    assert len(world.bins) == 90
    assert len(world.aisles) == 3
    assert len(world.racks) == 6
    assert len(world.pads) == 1
    assert len(world.charging_stations) == 1
    assert len(world.no_fly_zones) == 1
    assert len(world.people_zones) == 1
    assert len(world.static_obstacles) == 1

    summary = world.get_summary()
    assert summary.total_bins == 90
    assert summary.total_racks == 6
    assert summary.total_aisles == 3
    assert summary.floor_dimensions_m == (18.0, 14.0)


def test_generate_medium_world():
    cfg = load_config(PRESETS_DIR / "medium.yaml")
    world = generate_world_from_config(cfg)
    summary = world.get_summary()

    # Medium preset: 8 aisles * 2 racks * 8 bays * 4 levels = 512 bins (~500-bin requirement)
    assert summary.total_bins == 512
    assert summary.total_aisles == 8
    assert summary.total_racks == 16
    assert summary.total_bays == 128
    assert summary.total_markers == 80
    assert summary.total_pads == 2


def test_generate_large_world():
    cfg = load_config(PRESETS_DIR / "large.yaml")
    world = generate_world_from_config(cfg)
    summary = world.get_summary()

    # Large preset: 20 aisles * 2 racks * 16 bays * 4 levels * 2 slots = 5,120 bins (~5,000-bin requirement)
    assert summary.total_bins == 5120
    assert summary.total_aisles == 20
    assert summary.total_racks == 40
    assert summary.total_bays == 640
    assert summary.total_markers == 200
    assert summary.total_pads == 3


def test_bin_geometry_and_scan_poses():
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)

    # Check a specific bin
    bin_obj = next(iter(world.bins.values()))
    assert bin_obj.label_code.startswith("WH01-")
    assert bin_obj.bounds.x_max > bin_obj.bounds.x_min
    assert bin_obj.bounds.y_max > bin_obj.bounds.y_min
    assert bin_obj.bounds.z_max > bin_obj.bounds.z_min

    # Recommended scan pos should have standoff distance
    cx, cy, cz = bin_obj.center
    sx, sy, sz = bin_obj.recommended_scan_pos
    assert abs(sz - cz) < 1e-3
    assert abs(sy - cy) < 1e-3
    assert (
        abs(abs(sx - cx) - (cfg.drone.camera.standoff_distance_m + bin_obj.bounds.width / 2.0))
        < 0.1
    )


def test_marker_placement_rules():
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)

    # Rack has 5 bays (0, 1, 2, 3, 4).
    # Rule: rack_ends=True (bays 0 and 4), every_n_bays=2 (bays 0, 2, 4)
    # Total marked bays per rack = {0, 2, 4} -> 3 markers per rack
    # 6 racks * 3 markers = 18 markers
    assert len(world.markers) == 18
    r01_markers = [m for m in world.markers if m.rack_id == "R01"]
    assert len(r01_markers) == 3
    marked_bays = {m.bay_index for m in r01_markers}
    assert marked_bays == {0, 2, 4}


def test_get_bin_query():
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)

    first_id = next(iter(world.bins.keys()))
    found = world.get_bin(first_id)
    assert found is not None
    assert found.id == first_id

    missing = world.get_bin("NON-EXISTENT-BIN")
    assert missing is None
