"""Safety supervisor data models, violation types, and scenario results."""

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any


class SafetyViolationType(StrEnum):
    STATIC_OBSTACLE_COLLISION = "static_obstacle_collision"
    DYNAMIC_OBSTACLE_CONFLICT = "dynamic_obstacle_conflict"
    PEOPLE_ZONE_OVERSPEED = "people_zone_overspeed"
    RACK_CLEARANCE_BREACH = "rack_clearance_breach"
    GEOFENCE_BREACH = "geofence_breach"
    NO_FLY_ZONE_BREACH = "no_fly_zone_breach"
    BATTERY_LOW_RTL = "battery_low_rtl"
    BATTERY_CRITICAL_LAND = "battery_critical_land"
    TELEMETRY_LINK_LOSS = "telemetry_link_loss"
    LOCALISATION_LOSS = "localisation_loss"
    EMERGENCY_STOP = "emergency_stop"
    WATCHDOG_TIMEOUT = "watchdog_timeout"


class SafetyAlertLevel(StrEnum):
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class FailsafeAction(StrEnum):
    NONE = "none"
    CLAMP_SPEED = "clamp_speed"
    REPLAN_PATH = "replan_path"
    HOLD_HOVER = "hold_hover"
    RETURN_HOME = "return_home"
    EMERGENCY_LAND = "emergency_land"
    EMERGENCY_STOP = "emergency_stop"
    WATCHDOG_FAILSAFE = "watchdog_failsafe"


@dataclass
class SafetyEvent:
    """An alert or corrective action taken by the safety supervisor."""

    timestamp_s: float
    alert_level: SafetyAlertLevel
    violation_type: SafetyViolationType
    message: str
    action_taken: FailsafeAction
    drone_position: tuple[float, float, float]
    extra_data: dict[str, Any] = field(default_factory=dict)


class FailureScenarioId(StrEnum):
    SCENARIO_1_STATIC_OBSTACLE = "SCENARIO_1_STATIC_OBSTACLE"
    SCENARIO_2_DYNAMIC_OBSTACLE = "SCENARIO_2_DYNAMIC_OBSTACLE"
    SCENARIO_3_PEOPLE_ZONE_SPEED = "SCENARIO_3_PEOPLE_ZONE_SPEED"
    SCENARIO_4_MIN_RACK_CLEARANCE = "SCENARIO_4_MIN_RACK_CLEARANCE"
    SCENARIO_5_GEOFENCE_BREACH = "SCENARIO_5_GEOFENCE_BREACH"
    SCENARIO_6_BATTERY_RETURN_HOME = "SCENARIO_6_BATTERY_RETURN_HOME"
    SCENARIO_7_CRITICAL_BATTERY_LAND = "SCENARIO_7_CRITICAL_BATTERY_LAND"
    SCENARIO_8_LINK_LOSS = "SCENARIO_8_LINK_LOSS"
    SCENARIO_9_LOCALISATION_LOSS = "SCENARIO_9_LOCALISATION_LOSS"
    SCENARIO_10_EMERGENCY_STOP = "SCENARIO_10_EMERGENCY_STOP"
    SCENARIO_11_SOFTWARE_CRASH_WATCHDOG = "SCENARIO_11_SOFTWARE_CRASH_WATCHDOG"


@dataclass
class ScenarioResult:
    """Outcome of executing a simulated failure injection test."""

    scenario_id: FailureScenarioId
    name: str
    description: str
    passed: bool
    trigger_time_s: float
    response_time_s: float
    action_taken: str
    details: str
    events_log: list[str] = field(default_factory=list)
