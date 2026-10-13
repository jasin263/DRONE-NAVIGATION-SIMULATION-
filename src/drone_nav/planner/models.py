"""Data models for mission flight planning and waypoints."""

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any


class WaypointAction(StrEnum):
    TAKEOFF = "takeoff"
    FLY = "fly"
    SCAN = "scan"
    RETURN = "return"
    LAND = "land"
    CHARGE = "charge"


@dataclass
class Waypoint:
    """Individual 4D navigational waypoint with drone orientation and action."""
    id: str
    position: tuple[float, float, float]  # (x, y, z) in metres
    yaw_rad: float  # Heading orientation in radians [0, 2*pi)
    speed_mps: float
    action: WaypointAction = WaypointAction.FLY
    target_bin_id: str | None = None
    wait_time_s: float = 0.0  # Hover duration for scanning or inspection

    @property
    def x(self) -> float:
        return self.position[0]

    @property
    def y(self) -> float:
        return self.position[1]

    @property
    def z(self) -> float:
        return self.position[2]


@dataclass
class FlightPlan:
    """A single flight sortie between battery charging cycles."""
    flight_id: int
    departure_pad_id: str
    arrival_pad_id: str
    waypoints: list[Waypoint] = field(default_factory=list)
    total_distance_m: float = 0.0
    estimated_duration_s: float = 0.0
    estimated_energy_wh: float = 0.0
    scanned_bin_ids: list[str] = field(default_factory=list)

    @property
    def distance_m(self) -> float:
        return self.total_distance_m

    def to_dict(self) -> dict[str, Any]:
        return {
            "flight_id": self.flight_id,
            "departure_pad_id": self.departure_pad_id,
            "arrival_pad_id": self.arrival_pad_id,
            "total_distance_m": round(self.total_distance_m, 2),
            "estimated_duration_s": round(self.estimated_duration_s, 1),
            "estimated_energy_wh": round(self.estimated_energy_wh, 2),
            "scanned_bins_count": len(self.scanned_bin_ids),
            "waypoints": [
                {
                    "id": wp.id,
                    "position": list(wp.position),
                    "yaw_rad": round(wp.yaw_rad, 3),
                    "action": wp.action.value,
                    "target_bin_id": wp.target_bin_id,
                }
                for wp in self.waypoints
            ],
        }


@dataclass
class MissionPlan:
    """Complete multi-flight warehouse scanning mission."""
    mission_id: str
    site_id: str
    strategy: str
    flights: list[FlightPlan] = field(default_factory=list)
    bins_targeted: int = 0
    bins_scanned: int = 0
    unreachable_bins: list[str] = field(default_factory=list)

    @property
    def total_flights(self) -> int:
        return len(self.flights)

    @property
    def total_distance_m(self) -> float:
        return sum(f.total_distance_m for f in self.flights)

    @property
    def total_duration_s(self) -> float:
        return sum(f.estimated_duration_s for f in self.flights)

    @property
    def total_energy_wh(self) -> float:
        return sum(f.estimated_energy_wh for f in self.flights)

    @property
    def coverage_pct(self) -> float:
        if self.bins_targeted == 0:
            return 100.0
        return round((self.bins_scanned / self.bins_targeted) * 100.0, 2)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "site_id": self.site_id,
            "strategy": self.strategy,
            "total_flights": self.total_flights,
            "total_distance_m": round(self.total_distance_m, 2),
            "total_duration_s": round(self.total_duration_s, 1),
            "total_energy_wh": round(self.total_energy_wh, 2),
            "bins_targeted": self.bins_targeted,
            "bins_scanned": self.bins_scanned,
            "coverage_pct": self.coverage_pct,
            "unreachable_count": len(self.unreachable_bins),
            "flights": [f.to_dict() for f in self.flights],
        }


@dataclass
class StrategyComparisonRow:
    """Row in strategy comparison table."""
    strategy: str
    coverage_pct: float
    total_flights: int
    total_distance_m: float
    total_duration_s: float
    total_energy_wh: float
    avg_dist_per_bin_m: float
