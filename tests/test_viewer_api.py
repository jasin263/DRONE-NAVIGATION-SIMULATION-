"""Unit tests for FastAPI viewer application and REST endpoints."""

from pathlib import Path

from fastapi.testclient import TestClient

from drone_nav.viewer.app import app

client = TestClient(app)


def test_index_page_returns_html():
    response = client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]
    assert "drone-nav" in response.text
    assert "threeContainer" in response.text or "mapCanvas" in response.text


def test_api_presets():
    response = client.get("/api/presets")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    preset_ids = [p["id"] for p in data]
    assert "small" in preset_ids
    assert "medium" in preset_ids
    assert "large" in preset_ids


def test_api_schema():
    response = client.get("/api/schema")
    assert response.status_code == 200
    data = response.json()
    assert "properties" in data
    assert "site" in data["properties"]


def test_api_world_small_preset():
    response = client.get("/api/world?config=small")
    assert response.status_code == 200
    data = response.json()
    assert data["site_id"] == "WH-SMALL"
    assert data["summary"]["total_bins"] == 90
    assert len(data["aisles"]) == 3
    assert len(data["racks"]) == 6
    assert len(data["markers"]) == 18


def test_api_world_not_found():
    response = client.get("/api/world?config=nonexistent_preset_xyz")
    assert response.status_code == 404


def test_api_import_csv():
    csv_content = (
        "site_id,zone,aisle,rack,level,bin,label_code,x_min,x_max,y_min,y_max,z_min,z_max\n"
        "WH01,Z01,A01,R01,1,1,WH01-TAG1,1.0,2.0,1.0,2.0,0.5,1.5\n"
        "WH01,Z01,A01,R01,1,2,WH01-TAG2,1.0,2.0,2.0,3.0,0.5,1.5\n"
    )
    files = {"file": ("test.csv", csv_content.encode("utf-8"), "text/csv")}
    response = client.post("/api/import-csv", files=files)
    assert response.status_code == 200
    data = response.json()
    assert data["summary"]["total_bins"] == 2
    assert data["site_id"] == "WH01"


# ============================================================================
# Phase 7 Endpoints Tests
# ============================================================================


def test_api_v1_config_get():
    response = client.get("/api/v1/config?config=small")
    assert response.status_code == 200
    data = response.json()
    assert data["preset_id"] == "small"
    assert "yaml_text" in data
    assert "config" in data
    assert data["config"]["site"]["id"] == "WH-SMALL"


def test_api_v1_config_validate_valid():
    yaml_text = Path("configs/presets/small.yaml").read_text(encoding="utf-8")
    response = client.post("/api/v1/config/validate", json={"yaml_text": yaml_text})
    assert response.status_code == 200
    data = response.json()
    assert data["valid"] is True
    assert data["site_id"] == "WH-SMALL"


def test_api_v1_config_validate_invalid():
    bad_yaml = "site:\n  id: ''\nlayout:\n  aisles: []"
    response = client.post("/api/v1/config/validate", json={"yaml_text": bad_yaml})
    assert response.status_code == 200
    data = response.json()
    assert data["valid"] is False
    assert "error" in data


def test_api_v1_config_save():
    yaml_text = Path("configs/presets/small.yaml").read_text(encoding="utf-8")
    response = client.post(
        "/api/v1/config/save",
        json={"preset_name": "temp_test_preset", "yaml_text": yaml_text},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["saved"] is True
    assert data["preset_id"] == "temp_test_preset"
    # Clean up created file
    created = Path("configs/presets/temp_test_preset.yaml")
    if created.exists():
        created.unlink()


def test_api_v1_mission_plan():
    response = client.post("/api/v1/mission/plan?config=small&strategy=serpentine")
    assert response.status_code == 200
    data = response.json()
    assert data["site_id"] == "WH-SMALL"
    assert data["total_flights"] >= 1
    assert data["bins_scanned"] == 90
    assert len(data["flights"]) >= 1
    assert len(data["flights"][0]["waypoints"]) > 0


def test_api_v1_mission_run_and_replay():
    response = client.post(
        "/api/v1/mission/run?config=small&strategy=serpentine&seed=42&wind_speed=0.1"
    )
    assert response.status_code == 200
    data = response.json()
    mission_id = data["mission_id"]
    assert data["bins_targeted"] == 90
    assert data["bins_scanned"] > 0
    assert data["successful_reads"] > 0
    assert len(data["states"]) > 0
    assert len(data["scans"]) > 0

    # Test replay endpoint
    replay_resp = client.get(f"/api/v1/mission/replay/{mission_id}")
    assert replay_resp.status_code == 200
    replay_data = replay_resp.json()
    assert replay_data["mission_id"] == mission_id
    assert len(replay_data["states"]) > 0

    # Test scans endpoint
    scans_resp = client.get(f"/api/v1/mission/scans/{mission_id}")
    assert scans_resp.status_code == 200
    scans_data = scans_resp.json()
    assert scans_data["mission_id"] == mission_id
    assert scans_data["total_scans"] > 0

    # Test export download endpoint
    dl_resp = client.get(f"/api/v1/export/download/{mission_id}")
    assert dl_resp.status_code == 200
    assert dl_resp.headers["content-type"] == "application/zip"
    assert len(dl_resp.content) > 100


def test_api_v1_mission_compare():
    response = client.post("/api/v1/mission/compare?config=small")
    assert response.status_code == 200
    data = response.json()
    assert data["site_id"] == "WH-SMALL"
    assert len(data["strategies"]) >= 3
    strat_names = [s["strategy"] for s in data["strategies"]]
    assert "serpentine" in strat_names
    assert "nearest" in strat_names
    assert "minimum_time" in strat_names


def test_api_v1_safety_scenarios_and_run():
    # List scenarios
    list_resp = client.get("/api/v1/safety/scenarios")
    assert list_resp.status_code == 200
    scenarios = list_resp.json()
    assert len(scenarios) == 11

    # Run scenario 1
    run_resp = client.post("/api/v1/safety/run?config=small&scenario=1")
    assert run_resp.status_code == 200
    run_data = run_resp.json()
    assert run_data["site_id"] == "WH-SMALL"
    assert len(run_data["results"]) == 1
    assert run_data["results"][0]["passed"] is True


def test_api_v1_scenarios_sweep():
    response = client.post("/api/v1/scenarios/sweep?config=small&sweep_type=wind")
    assert response.status_code == 200
    data = response.json()
    assert data["sweep_name"] == "Wind Disturbance Sweep"
    assert len(data["results"]) > 0
    assert "<svg" in data["svg_chart"]
