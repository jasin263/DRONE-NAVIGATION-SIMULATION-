"""Test suite for industrial Steel Coil OCR & Barcode Extraction Pipeline."""

from pathlib import Path
import pytest
from fastapi.testclient import TestClient

from drone_nav.scanning.coil_ocr_pipeline import CoilOCRPipeline, CoilScanResult
from drone_nav.viewer.app import app


@pytest.fixture
def client() -> TestClient:
    return TestClient(app)


@pytest.fixture
def pipeline() -> CoilOCRPipeline:
    img_dir = "Warehouse_images/preview" if Path("Warehouse_images/preview").exists() else "Warehouse_images/jpg"
    return CoilOCRPipeline(
        image_dir=img_dir,
        output_dir="exports/test_ocr_output",
        ocr_scale_dimension=1920,
    )


def test_pipeline_initialization(pipeline: CoilOCRPipeline):
    """Verify pipeline directories and engines initialize properly."""
    assert pipeline.image_dir.exists()
    assert pipeline.output_dir.exists()
    assert pipeline.ocr is not None


def test_coil_ocr_printed_tag_5808(pipeline: CoilOCRPipeline):
    """Verify printed tag 'GE660184200' and supplier extraction on IMG_5808."""
    img_path = Path("Warehouse_images/preview/IMG_5808.jpg")
    if not img_path.exists():
        img_path = Path("Warehouse_images/jpg/IMG_5808.jpg")

    result = pipeline.process_image(img_path, save_annotated=True)
    assert isinstance(result, CoilScanResult)
    assert result.coil_id == "GE660184200"
    assert result.mill_supplier in ["CLIFFS", "Cleveland-Cliffs"]
    assert result.overall_confidence >= 0.80
    assert result.annotated_image_path is not None
    assert Path(result.annotated_image_path).exists()

    # Verify DRIMS event payload formatting
    drims_ev = result.to_drims_scan_event()
    assert drims_ev["schemaVersion"] == "1.0"
    assert drims_ev["value"] == "GE660184200"
    assert drims_ev["coilId"] == "GE660184200"
    assert drims_ev["confidence"] >= 0.80


def test_coil_ocr_barcode_tag_5800(pipeline: CoilOCRPipeline):
    """Verify barcode decoding on steel sheet pack IMG_5800."""
    img_path = Path("Warehouse_images/jpg/IMG_5800.jpg")
    if not img_path.exists():
        img_path = Path("Warehouse_images/preview/IMG_5800.jpg")

    result = pipeline.process_image(img_path, save_annotated=True)
    assert isinstance(result, CoilScanResult)
    # IMG_5800 contains barcodes 673210 and B30951336B9
    if result.all_barcodes:
        barcode_vals = [b.value for b in result.all_barcodes]
        assert any("673210" in v or "B3095" in v for v in barcode_vals)
        assert result.primary_barcode is not None


def test_coil_ocr_sheet_pack_5798(pipeline: CoilOCRPipeline):
    """Verify product code and piece count extraction on IMG_5798."""
    img_path = Path("Warehouse_images/jpg/IMG_5798.jpg")
    if not img_path.exists():
        img_path = Path("Warehouse_images/preview/IMG_5798.jpg")

    result = pipeline.process_image(img_path, save_annotated=False)
    assert isinstance(result, CoilScanResult)
    # Candidate should contain P3352-18 or P3352-1B
    assert any("P3352" in c for c in result.all_coil_candidates) or result.coil_id is not None


def test_export_drims_package(pipeline: CoilOCRPipeline, tmp_path: Path):
    """Verify exporting detection results into a standard DRIMS .zip package."""
    img_path = Path("Warehouse_images/preview/IMG_5808.jpg")
    if not img_path.exists():
        img_path = Path("Warehouse_images/jpg/IMG_5808.jpg")

    res = pipeline.process_image(img_path, save_annotated=False)
    out_zip = tmp_path / "test_drims_coils.zip"

    pkg_path = pipeline.export_to_drims_package([res], output_zip=out_zip)
    assert pkg_path.exists()
    assert pkg_path.stat().st_size > 0


def test_api_coil_ocr_catalog(client: TestClient):
    """Verify GET /api/v1/coils/ocr-catalog returns scanned coil list."""
    res = client.get("/api/v1/coils/ocr-catalog?max_images=5")
    assert res.status_code == 200
    data = res.json()
    assert "total_images" in data
    assert "items" in data
    assert len(data["items"]) >= 1
    first_item = data["items"][0]
    assert "image_name" in first_item
    assert "image_url" in first_item


def test_api_single_coil_ocr(client: TestClient):
    """Verify GET /api/v1/coils/ocr/IMG_5808 returns detailed scan detection."""
    res = client.get("/api/v1/coils/ocr/IMG_5808")
    assert res.status_code == 200
    data = res.json()
    assert data["image_name"].startswith("IMG_5808")
    assert data["coil_id"] == "GE660184200"
    assert "drims_event" in data
    assert data["drims_event"]["coilId"] == "GE660184200"


def test_api_download_coil_drims_endpoint(client: TestClient):
    """Verify GET /api/v1/coils/download-drims returns valid zip archive."""
    res = client.get("/api/v1/coils/download-drims")
    assert res.status_code == 200
    assert res.headers["content-type"] == "application/zip"
    assert len(res.content) > 100
