"""Pydantic v2 configuration models for drone-nav.

Defines the complete schema for site, layout, obstacles, drone, localisation,
mission, safety, and DRIMS export settings.
"""

from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator


class CoordinateFrameConfig(BaseModel):
    """Warehouse coordinate frame definition."""

    origin: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0], description="[x, y, z] origin offset in metres"
    )
    x_axis: str = Field(default="east", description="Physical direction of positive X axis")
    y_axis: str = Field(default="north", description="Physical direction of positive Y axis")
    z_axis: str = Field(default="up", description="Physical direction of positive Z axis")
    z_up: bool = Field(default=True, description="True if positive Z points upward")


class FloorBounds(BaseModel):
    """2D floor bounding box in metres."""

    x_min: float = Field(default=0.0, ge=0.0)
    x_max: float = Field(gt=0.0, description="Warehouse width in metres")
    y_min: float = Field(default=0.0, ge=0.0)
    y_max: float = Field(gt=0.0, description="Warehouse length in metres")

    @model_validator(mode="after")
    def validate_bounds(self) -> "FloorBounds":
        if self.x_max <= self.x_min:
            raise ValueError(
                f"x_max ({self.x_max}) must be strictly greater than x_min ({self.x_min})"
            )
        if self.y_max <= self.y_min:
            raise ValueError(
                f"y_max ({self.y_max}) must be strictly greater than y_min ({self.y_min})"
            )
        return self


class SiteConfig(BaseModel):
    """Warehouse site metadata and boundary definitions."""

    id: str = Field(min_length=1, description="Site identifier, e.g. WH01")
    name: str = Field(min_length=1, description="Human readable warehouse name")
    units: Literal["metres"] = Field(default="metres")
    coordinate_frame: CoordinateFrameConfig = Field(default_factory=CoordinateFrameConfig)
    floor_bounds: FloorBounds
    ceiling_height: float = Field(gt=1.0, description="Ceiling height in metres")
    timezone: str = Field(default="UTC")


class RackSide(StrEnum):
    LEFT = "left"
    RIGHT = "right"


class RackConfig(BaseModel):
    """Rack specification flanking an aisle."""

    id: str = Field(min_length=1)
    bays: int = Field(gt=0, description="Number of bays along aisle")
    levels: int = Field(gt=0, description="Number of vertical storage levels")
    bay_width: float = Field(gt=0.0, description="Width of a single bay in metres")
    level_heights: list[float] = Field(description="Heights of beam levels from floor in metres")
    depth: float = Field(gt=0.0, description="Rack shelf depth in metres")
    bin_width: float = Field(gt=0.0, description="Individual bin width in metres")
    bin_height: float = Field(gt=0.0, description="Individual bin height in metres")
    bins_per_bay_level: int = Field(
        default=1, ge=1, description="Number of bins side-by-side per bay shelf"
    )

    @model_validator(mode="after")
    def validate_levels(self) -> "RackConfig":
        if len(self.level_heights) != self.levels:
            raise ValueError(
                f"Rack {self.id}: length of level_heights ({len(self.level_heights)}) "
                f"must match levels count ({self.levels})"
            )
        for i in range(1, len(self.level_heights)):
            if self.level_heights[i] <= self.level_heights[i - 1]:
                raise ValueError(f"Rack {self.id}: level_heights must be strictly increasing")
        return self


class AisleDirection(StrEnum):
    FORWARD = "forward"
    BACKWARD = "backward"
    BIDIRECTIONAL = "bidirectional"


class AisleConfig(BaseModel):
    """Aisle layout definition with flanking racks."""

    id: str = Field(min_length=1)
    name: str | None = None
    x_start: float
    y_start: float
    x_end: float
    y_end: float
    width: float = Field(gt=0.5, description="Clear flight corridor width in metres")
    one_way: bool = Field(default=False)
    direction: AisleDirection = Field(default=AisleDirection.BIDIRECTIONAL)
    left_rack: RackConfig | None = None
    right_rack: RackConfig | None = None

    @model_validator(mode="after")
    def validate_aisle(self) -> "AisleConfig":
        if self.one_way and self.direction == AisleDirection.BIDIRECTIONAL:
            raise ValueError(f"Aisle {self.id}: one_way is True but direction is bidirectional")
        return self


class DoorConfig(BaseModel):
    id: str
    position: list[float] = Field(min_length=3, max_length=3, description="[x, y, z]")
    width: float = Field(gt=0.0)
    height: float = Field(gt=0.0)


class ChargingStationConfig(BaseModel):
    id: str
    position: list[float] = Field(min_length=3, max_length=3, description="[x, y, z]")
    charge_rate_kw: float = Field(gt=0.0, default=0.5)


class TakeoffLandingPadConfig(BaseModel):
    id: str
    position: list[float] = Field(min_length=3, max_length=3, description="[x, y, z]")
    radius: float = Field(gt=0.0, default=0.5)


class NoFlyZoneConfig(BaseModel):
    id: str
    name: str
    polygon: list[list[float]] = Field(description="List of [x, y] coordinates forming polygon")
    z_min: float = Field(default=0.0)
    z_max: float = Field(gt=0.0)

    @field_validator("polygon")
    @classmethod
    def validate_polygon(cls, v: list[list[float]]) -> list[list[float]]:
        if len(v) < 3:
            raise ValueError("No-fly zone polygon must contain at least 3 vertices")
        for pt in v:
            if len(pt) < 2:
                raise ValueError("Polygon points must be [x, y]")
        return v


class PeopleZoneConfig(BaseModel):
    id: str
    name: str
    polygon: list[list[float]] = Field(description="List of [x, y] coordinates")
    max_speed_mps: float = Field(gt=0.0, le=2.0, description="Drone speed limit in people zone")


class ZoneConfig(BaseModel):
    id: str
    name: str
    bounds: FloorBounds


class LayoutConfig(BaseModel):
    """Warehouse interior layout configuration."""

    zones: list[ZoneConfig] = Field(default_factory=list)
    aisles: list[AisleConfig] = Field(min_length=1)
    bin_numbering_template: str = Field(
        default="WH{site_id}-A{aisle_id}-R{rack_id}-L{level}-B{bay}",
        description="Python str format template for bin label code",
    )
    doors: list[DoorConfig] = Field(default_factory=list)
    charging_stations: list[ChargingStationConfig] = Field(default_factory=list)
    takeoff_landing_pads: list[TakeoffLandingPadConfig] = Field(min_length=1)
    no_fly_zones: list[NoFlyZoneConfig] = Field(default_factory=list)
    people_zones: list[PeopleZoneConfig] = Field(default_factory=list)


class StaticObstacleConfig(BaseModel):
    id: str
    type: Literal["pallet", "pillar", "blocked_aisle", "box"] = "pallet"
    position: list[float] = Field(min_length=3, max_length=3, description="[x, y, z]")
    dimensions: list[float] = Field(min_length=3, max_length=3, description="[dx, dy, dz]")


class DynamicObstacleConfig(BaseModel):
    id: str
    type: Literal["walker", "forklift"] = "forklift"
    path: list[list[float]] = Field(min_length=2, description="List of [x, y, z] waypoints")
    speed_mps: float = Field(gt=0.0)
    schedule_start_s: float = Field(default=0.0, ge=0.0)
    schedule_end_s: float = Field(gt=0.0)


class ObstaclesConfig(BaseModel):
    """Obstacles specification in warehouse."""

    static: list[StaticObstacleConfig] = Field(default_factory=list)
    dynamic: list[DynamicObstacleConfig] = Field(default_factory=list)
    random_seed: int = Field(default=42)
    random_count: int = Field(default=0, ge=0)


class BatteryConfig(BaseModel):
    """Drone battery capacity and power consumption model."""

    capacity_wh: float = Field(gt=0.0, description="Total battery capacity in Watt-hours")
    hover_power_w: float = Field(gt=0.0, description="Power consumed while hovering in Watts")
    forward_power_w_per_mps: float = Field(
        ge=0.0, description="Additional power per m/s forward speed"
    )
    climb_power_w_per_mps: float = Field(ge=0.0, description="Additional power per m/s climb speed")
    safe_reserve_pct: float = Field(
        default=20.0, ge=5.0, le=50.0, description="Safe battery reserve %"
    )
    critical_reserve_pct: float = Field(
        default=10.0, ge=2.0, le=25.0, description="Emergency landing reserve %"
    )

    @model_validator(mode="after")
    def validate_reserves(self) -> "BatteryConfig":
        if self.critical_reserve_pct >= self.safe_reserve_pct:
            raise ValueError("critical_reserve_pct must be less than safe_reserve_pct")
        return self


class CameraConfig(BaseModel):
    """Scanner camera payload specification."""

    resolution_w: int = Field(default=1920, gt=0)
    resolution_h: int = Field(default=1080, gt=0)
    hfov_deg: float = Field(default=70.0, gt=10.0, lt=180.0)
    vfov_deg: float = Field(default=45.0, gt=10.0, lt=180.0)
    mount_angle_deg: float = Field(
        default=0.0, ge=-90.0, le=90.0, description="Pitch angle relative to drone forward"
    )
    standoff_distance_m: float = Field(
        default=1.0, gt=0.2, description="Target distance to rack face"
    )
    exposure_time_s: float = Field(default=0.005, gt=0.0)
    max_blur_speed_mps: float = Field(default=1.5, gt=0.0)


class SensorNoiseConfig(BaseModel):
    """Noise and bias parameters for simulated sensors."""

    imu_accel_bias: float = Field(default=0.02, ge=0.0)
    imu_accel_noise: float = Field(default=0.05, ge=0.0)
    imu_gyro_bias: float = Field(default=0.005, ge=0.0)
    imu_gyro_noise: float = Field(default=0.01, ge=0.0)
    marker_pos_noise: float = Field(default=0.02, ge=0.0)
    marker_miss_prob: float = Field(default=0.05, ge=0.0, lt=1.0)
    lidar_range_max_m: float | None = Field(default=10.0)
    lidar_noise_m: float | None = Field(default=0.02)


class DroneConfig(BaseModel):
    """Drone physical properties, limits, and payload."""

    mass_kg: float = Field(gt=0.1, le=25.0, description="Total takeoff mass in kg")
    max_speed_mps: float = Field(gt=0.1, le=10.0, description="Max horizontal flight speed")
    max_accel_mps2: float = Field(gt=0.1, le=5.0, description="Max linear acceleration")
    max_yaw_rate_radps: float = Field(gt=0.1, le=3.14, description="Max rotational yaw rate")
    battery: BatteryConfig
    camera: CameraConfig = Field(default_factory=CameraConfig)
    sensors: SensorNoiseConfig = Field(default_factory=SensorNoiseConfig)


class MarkerSpacingRule(BaseModel):
    rack_ends: bool = Field(default=True, description="Always place fiducials at rack start/end")
    every_n_bays: int = Field(default=2, ge=1, description="Spacing interval in bays")


class DriftModelConfig(BaseModel):
    walk_std_mps: float = Field(default=0.03, ge=0.0, description="Random walk standard deviation")


class EKFParamsConfig(BaseModel):
    process_noise_pos: float = Field(default=0.01, gt=0.0)
    process_noise_vel: float = Field(default=0.05, gt=0.0)
    meas_noise_marker: float = Field(default=0.03, gt=0.0)
    meas_noise_imu: float = Field(default=0.1, gt=0.0)


class LocalisationConfig(BaseModel):
    """Fiducial marker and sensor fusion configuration."""

    marker_size_m: float = Field(default=0.15, gt=0.01)
    marker_spacing: MarkerSpacingRule = Field(default_factory=MarkerSpacingRule)
    pose_noise: float = Field(default=0.02, ge=0.0)
    drift_model: DriftModelConfig = Field(default_factory=DriftModelConfig)
    fusion_filter: EKFParamsConfig = Field(default_factory=EKFParamsConfig)


class ScanStrategy(StrEnum):
    SERPENTINE = "serpentine"
    NEAREST = "nearest"
    MINIMUM_TIME = "minimum_time"


class ReturnToChargeRules(BaseModel):
    threshold_pct: float = Field(default=25.0, ge=10.0, le=50.0)
    pad_id: str | None = None


class MissionConfig(BaseModel):
    """Mission planning parameters."""

    target_zones: list[str] = Field(default_factory=list)
    target_bins: list[str] = Field(default_factory=list)
    priority: Literal["low", "normal", "high"] = "normal"
    scan_strategy: ScanStrategy = Field(default=ScanStrategy.SERPENTINE)
    max_flight_duration_s: float = Field(default=1200.0, gt=60.0)
    return_to_charge_rules: ReturnToChargeRules = Field(default_factory=ReturnToChargeRules)


class LinkLossBehaviour(StrEnum):
    HOVER_THEN_LAND = "hover_then_land"
    RETURN_HOME = "return_home"
    EMERGENCY_STOP = "emergency_stop"


class LocalisationLossBehaviour(StrEnum):
    HOVER_THEN_LAND = "hover_then_land"
    EMERGENCY_STOP = "emergency_stop"


class SafetyConfig(BaseModel):
    """Safety supervisor thresholds and fallback behaviours."""

    min_rack_clearance_m: float = Field(default=0.5, gt=0.1)
    geofence_margin_m: float = Field(default=0.5, gt=0.1)
    battery_reserve_pct: float = Field(default=20.0, ge=5.0)
    link_loss_behaviour: LinkLossBehaviour = Field(default=LinkLossBehaviour.HOVER_THEN_LAND)
    localisation_loss_behaviour: LocalisationLossBehaviour = Field(
        default=LocalisationLossBehaviour.HOVER_THEN_LAND
    )
    emergency_stop_decel_mps2: float = Field(default=4.0, gt=1.0)


class ExportMode(StrEnum):
    FILE = "file"
    API = "api"
    DRY_RUN = "dry_run"


class ExportConfig(BaseModel):
    """DRIMS ingestion export settings."""

    drims_endpoint_url: str | None = None
    auth_token: str | None = None
    site_id_mapping: str | None = None
    schema_version: str = Field(default="1.0")
    mode: ExportMode = Field(default=ExportMode.FILE)
    output_directory: str = Field(default="exports")


class WarehouseConfig(BaseModel):
    """Root configuration model for drone-nav."""

    version: str = Field(default="1.0")
    site: SiteConfig
    layout: LayoutConfig
    obstacles: ObstaclesConfig = Field(default_factory=ObstaclesConfig)
    drone: DroneConfig
    localisation: LocalisationConfig = Field(default_factory=LocalisationConfig)
    mission: MissionConfig = Field(default_factory=MissionConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    export: ExportConfig = Field(default_factory=ExportConfig)

    @model_validator(mode="after")
    def validate_global_consistency(self) -> "WarehouseConfig":
        # Check that takeoff pads are within site floor bounds
        fb = self.site.floor_bounds
        for pad in self.layout.takeoff_landing_pads:
            px, py = pad.position[0], pad.position[1]
            if not (fb.x_min <= px <= fb.x_max and fb.y_min <= py <= fb.y_max):
                raise ValueError(
                    f"Takeoff pad '{pad.id}' at ({px}, {py}) is outside floor bounds "
                    f"[{fb.x_min}, {fb.x_max}] x [{fb.y_min}, {fb.y_max}]"
                )
        # Check aisles are within floor bounds
        for aisle in self.layout.aisles:
            for x, y in [(aisle.x_start, aisle.y_start), (aisle.x_end, aisle.y_end)]:
                if not (fb.x_min <= x <= fb.x_max and fb.y_min <= y <= fb.y_max):
                    raise ValueError(
                        f"Aisle '{aisle.id}' endpoint ({x}, {y}) is outside floor bounds "
                        f"[{fb.x_min}, {fb.x_max}] x [{fb.y_min}, {fb.y_max}]"
                    )
        return self
