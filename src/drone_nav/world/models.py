"""World model representing the physical warehouse structure.

All positions are 3D coordinates (x, y, z) in metres relative to the warehouse origin.
"""

from dataclasses import dataclass, field
from typing import Literal


@dataclass(frozen=True)
class BoundingBox3D:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @property
    def center(self) -> tuple[float, float, float]:
        return (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
            (self.z_min + self.z_max) / 2.0,
        )

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def length(self) -> float:
        return self.y_max - self.y_min

    @property
    def height(self) -> float:
        return self.z_max - self.z_min


@dataclass
class Bin:
    """Individual storage bin in a rack shelf."""

    id: str
    label_code: str
    site_id: str
    zone_id: str
    aisle_id: str
    rack_id: str
    bay_index: int
    level_index: int
    shelf_slot: int
    bounds: BoundingBox3D
    scan_face_normal: tuple[float, float, float]  # Vector pointing from bin toward aisle
    recommended_scan_pos: tuple[float, float, float]  # Position drone should hover to scan

    @property
    def center(self) -> tuple[float, float, float]:
        return self.bounds.center


@dataclass
class Rack:
    """Storage rack structure."""

    id: str
    aisle_id: str
    side: Literal["left", "right"]
    bays: int
    levels: int
    bounds: BoundingBox3D
    bins: list[Bin] = field(default_factory=list)


@dataclass
class Aisle:
    """Flight aisle corridor flanked by racks."""

    id: str
    name: str | None
    x_start: float
    y_start: float
    x_end: float
    y_end: float
    width: float
    one_way: bool
    direction: str  # forward, backward, bidirectional
    corridor_bounds: BoundingBox3D
    left_rack: Rack | None = None
    right_rack: Rack | None = None


@dataclass
class FiducialMarker:
    """Fiducial visual marker (e.g., AprilTag/ArUco) for localisation."""

    id: str
    tag_id: int
    position: tuple[float, float, float]
    normal: tuple[float, float, float]  # Facing direction
    size_m: float
    rack_id: str | None = None
    bay_index: int | None = None


@dataclass
class TakeoffLandingPad:
    id: str
    position: tuple[float, float, float]
    radius: float


@dataclass
class ChargingStation:
    id: str
    position: tuple[float, float, float]
    charge_rate_kw: float


@dataclass
class Door:
    id: str
    position: tuple[float, float, float]
    width: float
    height: float


@dataclass
class NoFlyZone:
    id: str
    name: str
    polygon: list[tuple[float, float]]
    z_min: float
    z_max: float


@dataclass
class PeopleZone:
    id: str
    name: str
    polygon: list[tuple[float, float]]
    max_speed_mps: float


@dataclass
class StaticObstacle:
    id: str
    type: str
    bounds: BoundingBox3D


@dataclass
class DynamicObstacle:
    id: str
    type: str
    path: list[tuple[float, float, float]]
    speed_mps: float
    schedule_start_s: float
    schedule_end_s: float


@dataclass
class WorldSummary:
    site_id: str
    site_name: str
    total_aisles: int
    total_racks: int
    total_bays: int
    total_bins: int
    total_markers: int
    total_pads: int
    total_charging_stations: int
    total_no_fly_zones: int
    total_people_zones: int
    floor_dimensions_m: tuple[float, float]
    ceiling_height_m: float


@dataclass
class World:
    """Complete in-memory warehouse world model."""

    site_id: str
    site_name: str
    floor_bounds: tuple[float, float, float, float]  # x_min, x_max, y_min, y_max
    ceiling_height: float
    aisles: list[Aisle] = field(default_factory=list)
    racks: list[Rack] = field(default_factory=list)
    bins: dict[str, Bin] = field(default_factory=dict)
    markers: list[FiducialMarker] = field(default_factory=list)
    pads: list[TakeoffLandingPad] = field(default_factory=list)
    charging_stations: list[ChargingStation] = field(default_factory=list)
    doors: list[Door] = field(default_factory=list)
    no_fly_zones: list[NoFlyZone] = field(default_factory=list)
    people_zones: list[PeopleZone] = field(default_factory=list)
    static_obstacles: list[StaticObstacle] = field(default_factory=list)
    dynamic_obstacles: list[DynamicObstacle] = field(default_factory=list)

    def get_bin(self, bin_id: str) -> Bin | None:
        return self.bins.get(bin_id)

    def get_summary(self) -> WorldSummary:
        total_bays = sum(rack.bays for rack in self.racks)
        dx = self.floor_bounds[1] - self.floor_bounds[0]
        dy = self.floor_bounds[3] - self.floor_bounds[2]
        return WorldSummary(
            site_id=self.site_id,
            site_name=self.site_name,
            total_aisles=len(self.aisles),
            total_racks=len(self.racks),
            total_bays=total_bays,
            total_bins=len(self.bins),
            total_markers=len(self.markers),
            total_pads=len(self.pads),
            total_charging_stations=len(self.charging_stations),
            total_no_fly_zones=len(self.no_fly_zones),
            total_people_zones=len(self.people_zones),
            floor_dimensions_m=(dx, dy),
            ceiling_height_m=self.ceiling_height,
        )
