"""Inventory scan coverage tracker and per-bin audit state container."""

from dataclasses import dataclass, field
from datetime import UTC, datetime

from drone_nav.world.models import World


@dataclass
class BinScanAuditRecord:
    """Detailed lifecycle scan history for an individual warehouse bin."""

    bin_id: str
    label_code: str
    target_pos: tuple[float, float, float]
    visited: bool = False
    scanned: bool = False
    read_success: bool = False
    confidence: float = 0.0
    read_type: str = "BARCODE"
    symbology: str = "CODE128"
    decoded_value: str | None = None
    image_ref: str | None = None
    bbox: dict[str, int] = field(default_factory=lambda: {"x": 0, "y": 0, "w": 0, "h": 0})
    timestamp_utc: str | None = None
    flight_id: int = 1
    drone_pos_at_scan: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # AI Defect & Inventory Discrepancy Detection fields
    discrepancy_type: str = "none"  # "none" | "qty_mismatch" | "empty_bin" | "package_damage" | "surface_defect"
    discrepancy_severity: str = "none"  # "none" | "low" | "medium" | "high"
    expected_qty: int = 0
    detected_qty: int = 0
    defect_label: str | None = None
    defect_confidence: float = 0.0


class CoverageTracker:
    """Maintains mission-wide bin inventory scan status and coverage rates."""

    def __init__(self, world: World) -> None:
        self.world = world
        self.records: dict[str, BinScanAuditRecord] = {}

        for b_id, b_obj in world.bins.items():
            self.records[b_id] = BinScanAuditRecord(
                bin_id=b_id,
                label_code=b_obj.label_code,
                target_pos=b_obj.center,
            )

    @property
    def total_bins(self) -> int:
        return len(self.records)

    @property
    def visited_bins(self) -> int:
        return sum(1 for r in self.records.values() if r.visited)

    @property
    def scanned_bins(self) -> int:
        return sum(1 for r in self.records.values() if r.scanned)

    @property
    def successful_reads(self) -> int:
        return sum(1 for r in self.records.values() if r.read_success)

    @property
    def coverage_pct(self) -> float:
        if self.total_bins == 0:
            return 0.0
        return round((self.visited_bins / self.total_bins) * 100.0, 2)

    @property
    def read_success_rate(self) -> float:
        if self.scanned_bins == 0:
            return 0.0
        return round(self.successful_reads / self.scanned_bins, 4)

    @property
    def total_discrepancies(self) -> int:
        return sum(1 for r in self.records.values() if r.discrepancy_type != "none")

    @property
    def discrepancy_records(self) -> list[BinScanAuditRecord]:
        return [r for r in self.records.values() if r.discrepancy_type != "none"]

    def record_scan_event(
        self,
        bin_id: str,
        read_success: bool,
        confidence: float,
        read_type: str,
        symbology: str,
        decoded_value: str | None,
        image_ref: str | None,
        bbox: dict[str, int],
        flight_id: int,
        drone_pos: tuple[float, float, float],
        timestamp_s: float,
        start_time_iso: str | None = None,
        discrepancy_type: str = "none",
        discrepancy_severity: str = "none",
        expected_qty: int = 0,
        detected_qty: int = 0,
        defect_label: str | None = None,
        defect_confidence: float = 0.0,
    ) -> BinScanAuditRecord:
        """Update audit record when a bin is scanned."""
        rec = self.records.get(bin_id)
        if not rec:
            # Create if not tracked
            rec = BinScanAuditRecord(
                bin_id=bin_id,
                label_code=bin_id,
                target_pos=drone_pos,
            )
            self.records[bin_id] = rec

        rec.visited = True
        rec.scanned = True
        rec.read_success = read_success
        rec.confidence = confidence
        rec.read_type = read_type
        rec.symbology = symbology
        rec.decoded_value = decoded_value
        rec.image_ref = image_ref
        rec.bbox = bbox
        rec.flight_id = flight_id
        rec.drone_pos_at_scan = drone_pos
        rec.discrepancy_type = discrepancy_type
        rec.discrepancy_severity = discrepancy_severity
        rec.expected_qty = expected_qty
        rec.detected_qty = detected_qty
        rec.defect_label = defect_label
        rec.defect_confidence = defect_confidence

        # Generate realistic ISO 8601 UTC timestamp
        if start_time_iso:
            try:
                base_dt = datetime.fromisoformat(start_time_iso.replace("Z", "+00:00"))
                from datetime import timedelta
                rec.timestamp_utc = (base_dt + timedelta(seconds=timestamp_s)).isoformat()
            except Exception:
                rec.timestamp_utc = datetime.now(UTC).isoformat()
        else:
            rec.timestamp_utc = datetime.now(UTC).isoformat()

        return rec
