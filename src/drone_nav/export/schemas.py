"""Pydantic v2 data contract schemas for DRIMS v1 ingestion."""

import json
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class DRIMSPosition(BaseModel):
    """3D coordinates in warehouse frame of reference."""

    x: float
    y: float
    z: float
    frame: str = "warehouse_origin"


class DRIMSBBox(BaseModel):
    """2D pixel coordinates and dimensions of detected label inside frame."""

    x: int
    y: int
    w: int
    h: int


class DRIMSScanEvent(BaseModel):
    """Individual barcode/QR scan detection record sent to DRIMS inventory app."""

    model_config = ConfigDict(extra="forbid")

    schemaVersion: str = Field(default="1.0", description="DRIMS data contract schema version")
    missionId: str = Field(description="UUID string of flight mission")
    eventId: str = Field(description="Deterministic UUID v5 per scan occurrence")
    droneId: str = Field(description="Autonomous inventory drone asset identifier")
    timestamp: str = Field(description="ISO 8601 UTC timestamp of scan")
    readType: Literal["BARCODE", "QR", "RFID", "IMAGE_COUNT"]
    symbology: str = Field(description="Barcode format: CODE128, QRCODE, DATAMATRIX")
    value: str = Field(description="Decoded label value")
    confidence: float = Field(ge=0.0, le=1.0, description="Read confidence score")
    locationTag: str = Field(description="Warehouse bin location code")
    position: DRIMSPosition
    imageRef: str | None = Field(default=None, description="Path to captured bin image")
    bbox: DRIMSBBox


class DRIMSMissionSummary(BaseModel):
    """Summary record exported at conclusion of inventory scanning flight."""

    model_config = ConfigDict(extra="forbid")

    schemaVersion: str = Field(default="1.0")
    missionId: str
    droneId: str
    siteId: str
    startTime: str
    endTime: str
    totalFlights: int = Field(ge=1)
    binsScanned: int = Field(ge=0)
    binsTotal: int = Field(ge=0)
    coveragePct: float = Field(ge=0.0, le=100.0)
    readSuccessRate: float = Field(ge=0.0, le=1.0)
    configSnapshot: dict[str, Any] = Field(default_factory=dict)


class DRIMSTelemetryPoint(BaseModel):
    """Sampled in-flight telemetry record for spatial audit."""

    timestamp: str
    t_s: float
    position: DRIMSPosition
    speed_mps: float
    battery_wh: float
    battery_pct: float
    action: str


class GroundTruthEvent(BaseModel):
    """Ground truth state for evaluating inventory scan accuracy and synthetic failures."""

    binId: str
    locationTag: str
    truePosition: dict[str, float]
    visited: bool
    scanned: bool
    readSuccess: bool
    failureReason: str | None = None


def export_drims_json_schemas(output_dir: Path | str) -> tuple[Path, Path]:
    """Export standard JSON Schemas from DRIMS Pydantic v2 models."""
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scan_schema_path = out_dir / "drims_scan_event.schema.json"
    mission_schema_path = out_dir / "drims_mission_summary.schema.json"

    scan_schema = DRIMSScanEvent.model_json_schema()
    mission_schema = DRIMSMissionSummary.model_json_schema()

    scan_schema_path.write_text(json.dumps(scan_schema, indent=2), encoding="utf-8")
    mission_schema_path.write_text(json.dumps(mission_schema, indent=2), encoding="utf-8")

    return scan_schema_path, mission_schema_path
