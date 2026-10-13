"""Unit tests for optical camera model, label decoding, and coverage tracking."""

import math

import pytest

from drone_nav.config.loader import load_config
from drone_nav.planner.planner import MissionPlanner
from drone_nav.scanning.camera import CameraScanner
from drone_nav.scanning.reader import LabelReader
from drone_nav.scanning.service import ScanningMissionSimulator
from drone_nav.scanning.tracker import CoverageTracker
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def small_world_and_config():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    return world, config


def test_camera_scanner_visibility(small_world_and_config):
    world, config = small_world_and_config
    cam = CameraScanner(config)
    target_bin = list(world.bins.values())[0]

    # Standoff position facing bin face
    norm = target_bin.scan_face_normal
    target_yaw = math.atan2(-norm[1], -norm[0])
    scan_pos = target_bin.recommended_scan_pos

    is_vis, q, factors = cam.compute_scan_quality(scan_pos, target_yaw, 0.0, target_bin)
    assert is_vis is True
    assert 0.0 < q <= 1.0
    assert factors["dist_factor"] > 0.5
    assert factors["angle_factor"] > 0.8
    assert factors["motion_factor"] == 1.0


def test_camera_scanner_out_of_range(small_world_and_config):
    world, config = small_world_and_config
    cam = CameraScanner(config)
    target_bin = list(world.bins.values())[0]

    norm = target_bin.scan_face_normal
    target_yaw = math.atan2(-norm[1], -norm[0])

    # Position 10 metres away (way past standoff focus)
    far_pos = (target_bin.center[0] + 10.0, target_bin.center[1], target_bin.center[2])
    is_vis, q, _ = cam.compute_scan_quality(far_pos, target_yaw, 0.0, target_bin)
    assert is_vis is False
    assert q == 0.0


def test_camera_scanner_field_of_view_and_facing(small_world_and_config):
    world, config = small_world_and_config
    cam = CameraScanner(config)
    target_bin = list(world.bins.values())[0]

    norm = target_bin.scan_face_normal
    # Facing 180 degrees away from bin
    opposite_yaw = math.atan2(norm[1], norm[0])
    scan_pos = target_bin.recommended_scan_pos

    is_vis, q, _ = cam.compute_scan_quality(scan_pos, opposite_yaw, 0.0, target_bin)
    assert is_vis is False
    assert q == 0.0


def test_camera_motion_blur_degradation(small_world_and_config):
    world, config = small_world_and_config
    cam = CameraScanner(config)
    target_bin = list(world.bins.values())[0]

    norm = target_bin.scan_face_normal
    target_yaw = math.atan2(-norm[1], -norm[0])
    scan_pos = target_bin.recommended_scan_pos

    _, q_stationary, f_stat = cam.compute_scan_quality(scan_pos, target_yaw, 0.0, target_bin)
    _, q_moving, f_mov = cam.compute_scan_quality(scan_pos, target_yaw, 1.4, target_bin)

    assert f_mov["motion_factor"] < f_stat["motion_factor"]
    assert q_moving < q_stationary


def test_label_reader_decode_and_confidence(small_world_and_config):
    world, _ = small_world_and_config
    target_bin = list(world.bins.values())[0]
    reader = LabelReader(seed=42)

    # High quality scan
    res_high = reader.decode_label(target_bin, quality_score=0.90, bin_index=1)
    assert res_high.success is True
    assert res_high.confidence >= 0.80
    assert res_high.value == target_bin.label_code
    assert res_high.read_type in ["BARCODE", "QR"]
    assert res_high.image_ref is not None

    # Poor quality below threshold
    res_poor = reader.decode_label(target_bin, quality_score=0.10, bin_index=1)
    assert res_poor.success is False
    assert res_poor.failure_reason == "INSUFFICIENT_OPTICAL_QUALITY"
    assert res_poor.confidence == 0.0


def test_coverage_tracker(small_world_and_config):
    world, _ = small_world_and_config
    tracker = CoverageTracker(world)

    assert tracker.total_bins == len(world.bins)
    assert tracker.visited_bins == 0
    assert tracker.scanned_bins == 0
    assert tracker.successful_reads == 0
    assert tracker.coverage_pct == 0.0

    sample_bin = list(world.bins.values())[0]
    rec = tracker.record_scan_event(
        bin_id=sample_bin.id,
        read_success=True,
        confidence=0.95,
        read_type="BARCODE",
        symbology="CODE128",
        decoded_value=sample_bin.label_code,
        image_ref="test/img.jpg",
        bbox={"x": 100, "y": 100, "w": 200, "h": 100},
        flight_id=1,
        drone_pos=(1.0, 2.0, 3.0),
        timestamp_s=12.5,
    )

    assert rec.visited is True
    assert rec.scanned is True
    assert rec.read_success is True
    assert tracker.visited_bins == 1
    assert tracker.scanned_bins == 1
    assert tracker.successful_reads == 1
    assert tracker.read_success_rate == 1.0
    assert tracker.coverage_pct > 0.0


def test_scanning_mission_simulator(small_world_and_config):
    world, config = small_world_and_config
    planner = MissionPlanner(world, config)
    mission = planner.plan_mission("serpentine")

    simulator = ScanningMissionSimulator(world, config, seed=42)
    tracker, sim_results = simulator.simulate_mission_scanning(
        mission_plan=mission, flight_ids=[1], start_time_iso="2026-10-13T08:00:00Z"
    )

    assert len(sim_results) == 1
    assert tracker.total_bins == 90
    assert tracker.scanned_bins == 90
    assert tracker.coverage_pct == 100.0
    assert tracker.read_success_rate > 0.70
