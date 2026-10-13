"""Unit tests for FastAPI viewer application and REST endpoints."""

from fastapi.testclient import TestClient

from drone_nav.viewer.app import app

client = TestClient(app)


def test_index_page_returns_html():
    response = client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]
    assert "drone-nav" in response.text
    assert "mapCanvas" in response.text


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


def test_api_world_medium_preset():
    response = client.get("/api/world?config=medium")
    assert response.status_code == 200
    data = response.json()
    assert data["site_id"] == "WH-MEDIUM"
    assert data["summary"]["total_bins"] == 512
    assert len(data["aisles"]) == 8


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
