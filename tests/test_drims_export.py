"""Unit tests for DRIMS v1 JSON schema export, validation, and idempotency."""

import json
from pathlib import Path

import pytest
from pydantic import TypeAdapter, ValidationError

from drone_nav.config.loader import load_config
from drone_nav.config.models import ExportMode
from drone_nav.export.exporter import DRIMSExporter
from drone_nav.export.schemas import (
    DRIMSMissionSummary,
    DRIMSScanEvent,
    DRIMSTelemetryPoint,
    GroundTruthEvent,
    export_drims_json_schemas,
)
from drone_nav.planner.planner import MissionPlanner
from drone_nav.scanning.service import ScanningMissionSimulator
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def small_world_and_config():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    return world, config


def test_drims_scan_event_schema_validation():
    valid_event = {
        "schemaVersion": "1.0",
        "missionId": "MIS-TEST-001",
        "eventId": "12345678-1234-5678-1234-567812345678",
        "droneId": "DRONE-ALPHA-01",
        "timestamp": "2026-10-13T08:00:00Z",
        "readType": "BARCODE",
        "symbology": "CODE128",
        "value": "WH01-A01-R01-L1-B01",
        "confidence": 0.94,
        "locationTag": "WH01-A01-R01-L1-B01",
        "position": {"x": 3.5, "y": 2.8, "z": 1.0, "frame": "warehouse_origin"},
        "imageRef": "Amazonbindata/img/00001.jpg",
        "bbox": {"x": 860, "y": 490, "w": 200, "h": 100},
    }

    event = DRIMSScanEvent.model_validate(valid_event)
    assert event.readType == "BARCODE"
    assert event.confidence == 0.94

    # Test rejection of forbidden extra fields
    with pytest.raises(ValidationError):
        DRIMSScanEvent.model_validate({**valid_event, "extra_illegal_field": "error"})


def test_export_drims_json_schemas(tmp_path: Path):
    scan_p, miss_p = export_drims_json_schemas(tmp_path)
    assert scan_p.exists()
    assert miss_p.exists()

    scan_schema = json.loads(scan_p.read_text(encoding="utf-8"))
    assert scan_schema["title"] == "DRIMSScanEvent"
    assert "schemaVersion" in scan_schema["properties"]

    miss_schema = json.loads(miss_p.read_text(encoding="utf-8"))
    assert miss_schema["title"] == "DRIMSMissionSummary"
    assert "totalFlights" in miss_schema["properties"]


def test_drims_exporter_full_package(small_world_and_config, tmp_path: Path):
    world, config = small_world_and_config
    planner = MissionPlanner(world, config)
    mission = planner.plan_mission("serpentine")

    simulator = ScanningMissionSimulator(world, config, seed=42)
    tracker, sim_results = simulator.simulate_mission_scanning(
        mission_plan=mission, flight_ids=[1], start_time_iso="2026-10-13T08:00:00Z"
    )

    exporter = DRIMSExporter(config=config, drone_id="DRONE-TEST-01")
    out_dir = tmp_path / "drims_export"
    res = exporter.export_mission(
        world=world,
        tracker=tracker,
        sim_results=sim_results,
        mission_id="MIS-TEST-PKG",
        output_dir=out_dir,
        start_time_iso="2026-10-13T08:00:00Z",
        end_time_iso="2026-10-13T08:30:00Z",
    )

    assert res["total_scans"] > 0
    assert (out_dir / "mission.json").exists()
    assert (out_dir / "scans.json").exists()
    assert (out_dir / "telemetry.json").exists()
    assert (out_dir / "ground_truth.json").exists()
    assert (out_dir / "complete").exists()

    # Validate mission.json
    m_data = json.loads((out_dir / "mission.json").read_text(encoding="utf-8"))
    DRIMSMissionSummary.model_validate(m_data)

    # Validate scans.json
    s_data = json.loads((out_dir / "scans.json").read_text(encoding="utf-8"))
    TypeAdapter(list[DRIMSScanEvent]).validate_python(s_data)

    # Validate telemetry.json
    t_data = json.loads((out_dir / "telemetry.json").read_text(encoding="utf-8"))
    TypeAdapter(list[DRIMSTelemetryPoint]).validate_python(t_data)

    # Validate ground_truth.json
    gt_data = json.loads((out_dir / "ground_truth.json").read_text(encoding="utf-8"))
    TypeAdapter(list[GroundTruthEvent]).validate_python(gt_data)

    # Validate complete marker
    c_data = json.loads((out_dir / "complete").read_text(encoding="utf-8"))
    assert c_data["status"] == "COMPLETED"
    assert c_data["mission_id"] == "MIS-TEST-PKG"
    assert c_data["scans_count"] == len(s_data)


def test_drims_exporter_idempotency(small_world_and_config, tmp_path: Path):
    world, config = small_world_and_config
    planner = MissionPlanner(world, config)
    mission = planner.plan_mission("serpentine")

    simulator = ScanningMissionSimulator(world, config, seed=42)
    tracker, sim_results = simulator.simulate_mission_scanning(
        mission_plan=mission, flight_ids=[1], start_time_iso="2026-10-13T08:00:00Z"
    )

    exporter = DRIMSExporter(config=config, drone_id="DRONE-IDEMPOTENT")

    dir1 = tmp_path / "export_1"
    dir2 = tmp_path / "export_2"

    res1 = exporter.export_mission(
        world=world,
        tracker=tracker,
        sim_results=sim_results,
        mission_id="MIS-IDEM-001",
        output_dir=dir1,
        start_time_iso="2026-10-13T08:00:00Z",
        end_time_iso="2026-10-13T08:30:00Z",
    )

    res2 = exporter.export_mission(
        world=world,
        tracker=tracker,
        sim_results=sim_results,
        mission_id="MIS-IDEM-001",
        output_dir=dir2,
        start_time_iso="2026-10-13T08:00:00Z",
        end_time_iso="2026-10-13T08:30:00Z",
    )

    # Scans hash equality
    assert res1["idempotency_hash"] == res2["idempotency_hash"]

    # File byte comparison
    b1 = (dir1 / "scans.json").read_bytes()
    b2 = (dir2 / "scans.json").read_bytes()
    assert b1 == b2

    # Check event IDs are identical
    scans1 = json.loads((dir1 / "scans.json").read_text(encoding="utf-8"))
    scans2 = json.loads((dir2 / "scans.json").read_text(encoding="utf-8"))
    for s1, s2 in zip(scans1, scans2):
        assert s1["eventId"] == s2["eventId"]


def test_drims_exporter_api_unreachable_fallback(small_world_and_config, tmp_path: Path):
    world, config = small_world_and_config
    config.export.mode = ExportMode.API
    config.export.drims_endpoint_url = "http://127.0.0.1:54321/api/v1/ingest"

    planner = MissionPlanner(world, config)
    mission = planner.plan_mission("serpentine")

    simulator = ScanningMissionSimulator(world, config, seed=42)
    tracker, sim_results = simulator.simulate_mission_scanning(
        mission_plan=mission, flight_ids=[1], start_time_iso="2026-10-13T08:00:00Z"
    )

    exporter = DRIMSExporter(config=config, drone_id="DRONE-API-TEST")
    out_dir = tmp_path / "api_export"
    res = exporter.export_mission(
        world=world,
        tracker=tracker,
        sim_results=sim_results,
        mission_id="MIS-API-FALLBACK",
        output_dir=out_dir,
        start_time_iso="2026-10-13T08:00:00Z",
        end_time_iso="2026-10-13T08:30:00Z",
    )

    assert "NOT STARTED" in res["api_status"]
    assert (out_dir / "complete").exists()
