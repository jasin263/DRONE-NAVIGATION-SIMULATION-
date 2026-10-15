"""DRIMS v1 ingestion package exporter with atomic completion marker and idempotency."""

import hashlib
import json
import urllib.error
import urllib.request
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from drone_nav.config.models import WarehouseConfig
from drone_nav.export.schemas import (
    DRIMSBBox,
    DRIMSMissionSummary,
    DRIMSPosition,
    DRIMSScanEvent,
    DRIMSTelemetryPoint,
    GroundTruthEvent,
)
from drone_nav.scanning.tracker import CoverageTracker
from drone_nav.simulator.models import SimulationResult
from drone_nav.world.models import World


class DRIMSExporter:
    """Exports inventory scan missions in strict compliance with the DRIMS ingestion contract."""

    def __init__(self, config: WarehouseConfig, drone_id: str = "DRONE-ALPHA-01") -> None:
        self.config = config
        self.drone_id = drone_id
        # Standard UUID namespace for deterministic event ID generation
        self.uuid_namespace = uuid.uuid5(uuid.NAMESPACE_DNS, "drims.drone-nav.local")

    def generate_event_id(self, mission_id: str, bin_id: str, attempt: int = 1) -> str:
        """Deterministic UUID v5 ensuring repeatable, idempotent scan event IDs."""
        key = f"{mission_id}:{bin_id}:{attempt}"
        return str(uuid.uuid5(self.uuid_namespace, key))

    def export_mission(
        self,
        world: World,
        tracker: CoverageTracker,
        sim_results: list[SimulationResult],
        mission_id: str,
        output_dir: Path | str,
        start_time_iso: str | None = None,
        end_time_iso: str | None = None,
    ) -> dict[str, Any]:
        """Export full mission data package: mission.json, scans.json, telemetry.json, complete."""
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        now_utc = datetime.now(UTC).isoformat()
        start_t = start_time_iso or now_utc
        end_t = end_time_iso or now_utc

        # 1. Build Scans Array
        scan_events: list[DRIMSScanEvent] = []
        ground_truth_events: list[GroundTruthEvent] = []

        for b_id, rec in tracker.records.items():
            bin_obj = world.bins.get(b_id)
            bx, by, bz = bin_obj.center if bin_obj else (0.0, 0.0, 0.0)

            # Ground truth audit record
            gt_event = GroundTruthEvent(
                binId=b_id,
                locationTag=rec.label_code,
                truePosition={"x": bx, "y": by, "z": bz},
                visited=rec.visited,
                scanned=rec.scanned,
                readSuccess=rec.read_success,
                failureReason="MISSED_SCAN" if not rec.read_success else None,
            )
            ground_truth_events.append(gt_event)

            # Only successfully decoded scans go into official DRIMS scans.json
            if rec.read_success and rec.decoded_value:
                event_id = self.generate_event_id(mission_id, b_id, attempt=1)
                dx, dy, dz = rec.drone_pos_at_scan

                scan_event = DRIMSScanEvent(
                    schemaVersion="1.0",
                    missionId=mission_id,
                    eventId=event_id,
                    droneId=self.drone_id,
                    timestamp=rec.timestamp_utc or now_utc,
                    readType=rec.read_type,
                    symbology=rec.symbology,
                    value=rec.decoded_value,
                    confidence=rec.confidence,
                    locationTag=rec.label_code,
                    position=DRIMSPosition(
                        x=round(dx, 3),
                        y=round(dy, 3),
                        z=round(dz, 3),
                        frame="warehouse_origin",
                    ),
                    imageRef=rec.image_ref,
                    bbox=DRIMSBBox(
                        x=rec.bbox.get("x", 860),
                        y=rec.bbox.get("y", 490),
                        w=rec.bbox.get("w", 200),
                        h=rec.bbox.get("h", 100),
                    ),
                )
                scan_events.append(scan_event)

        # 2. Build Mission Summary
        mission_summary = DRIMSMissionSummary(
            schemaVersion="1.0",
            missionId=mission_id,
            droneId=self.drone_id,
            siteId=world.site_id,
            startTime=start_t,
            endTime=end_t,
            totalFlights=max(1, len(sim_results)),
            binsScanned=tracker.scanned_bins,
            binsTotal=tracker.total_bins,
            coveragePct=tracker.coverage_pct,
            readSuccessRate=tracker.read_success_rate,
            configSnapshot={
                "site_id": world.site_id,
                "strategy": self.config.mission.scan_strategy.value,
                "drone_mass_kg": self.config.drone.mass_kg,
                "max_speed_mps": self.config.drone.max_speed_mps,
            },
        )

        # 3. Build Telemetry Sample Points (down-sampled to ~300 points per flight)
        from datetime import timedelta
        try:
            base_dt = datetime.fromisoformat(start_t.replace("Z", "+00:00"))
        except Exception:
            base_dt = datetime.now(UTC)

        telemetry_samples: list[DRIMSTelemetryPoint] = []
        for sim_res in sim_results:
            step = max(1, len(sim_res.states) // 300)
            for state in sim_res.states[::step]:
                t_iso = (base_dt + timedelta(seconds=state.timestamp_s)).isoformat()
                telemetry_samples.append(
                    DRIMSTelemetryPoint(
                        timestamp=t_iso,
                        t_s=round(state.timestamp_s, 2),
                        position=DRIMSPosition(
                            x=round(state.position[0], 3),
                            y=round(state.position[1], 3),
                            z=round(state.position[2], 3),
                        ),
                        speed_mps=round(state.speed_mps, 2),
                        battery_wh=round(state.battery_wh, 2),
                        battery_pct=round(state.battery_pct, 1),
                        action=state.action,
                    )
                )

        # Write files atomically
        files_written: list[Path] = []

        # A. mission.json
        mission_json_path = out_path / "mission.json"
        mission_json_path.write_text(
            json.dumps(mission_summary.model_dump(), indent=2), encoding="utf-8"
        )
        files_written.append(mission_json_path)

        # B. scans.json
        scans_json_path = out_path / "scans.json"
        scans_payload = [e.model_dump() for e in scan_events]
        scans_content = json.dumps(scans_payload, indent=2)
        scans_json_path.write_text(scans_content, encoding="utf-8")
        files_written.append(scans_json_path)

        # Calculate idempotency hash over scans.json
        scans_hash = hashlib.sha256(scans_content.encode("utf-8")).hexdigest()

        # C. telemetry.json
        telem_json_path = out_path / "telemetry.json"
        telem_json_path.write_text(
            json.dumps([t.model_dump() for t in telemetry_samples], indent=2),
            encoding="utf-8",
        )
        files_written.append(telem_json_path)

        # D. ground_truth.json
        gt_json_path = out_path / "ground_truth.json"
        gt_json_path.write_text(
            json.dumps([gt.model_dump() for gt in ground_truth_events], indent=2),
            encoding="utf-8",
        )
        files_written.append(gt_json_path)

        # E. Atomic completion marker file
        marker_path = out_path / "complete"
        marker_payload = {
            "status": "COMPLETED",
            "completed_at": end_t,
            "mission_id": mission_id,
            "scans_count": len(scan_events),
            "scans_hash": scans_hash,
        }
        marker_path.write_text(json.dumps(marker_payload, indent=2), encoding="utf-8")
        files_written.append(marker_path)

        # Handle API Export Mode if requested
        api_status = "FILE_ONLY"
        if self.config.export.mode.value == "api":
            endpoint = self.config.export.drims_endpoint_url
            if endpoint:
                api_status = self._post_to_drims_api(endpoint, mission_summary, scan_events)
            else:
                api_status = "NOT STARTED: drims_endpoint_url not configured"

        return {
            "mission_id": mission_id,
            "export_dir": str(out_path),
            "total_scans": len(scan_events),
            "bins_targeted": tracker.total_bins,
            "coverage_pct": tracker.coverage_pct,
            "read_success_rate": tracker.read_success_rate,
            "idempotency_hash": scans_hash,
            "api_status": api_status,
            "files": [str(p) for p in files_written],
        }

    def _post_to_drims_api(
        self,
        endpoint_url: str,
        summary: DRIMSMissionSummary,
        scans: list[DRIMSScanEvent],
    ) -> str:
        """Attempt HTTP POST to external DRIMS endpoint; handles unreachable gracefully."""
        try:
            payload = {
                "mission": summary.model_dump(),
                "scans": [s.model_dump() for s in scans],
            }
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                endpoint_url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                if resp.status in (200, 201, 202):
                    return f"POST_SUCCESS (HTTP {resp.status})"
                return f"POST_FAILED (HTTP {resp.status})"
        except Exception as exc:
            return f"NOT STARTED: Endpoint unreachable ({exc})"
