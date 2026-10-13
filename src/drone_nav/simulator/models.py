"""Kinematic simulation data structures and telemetry models."""

import hashlib
import json
from dataclasses import dataclass, field
from typing import Any


@dataclass
class DroneState:
    """Full 6-DOF kinematic state and battery telemetry at a specific time step."""
    timestamp_s: float
    position: tuple[float, float, float]  # (x, y, z) in metres
    velocity: tuple[float, float, float]  # (vx, vy, vz) in m/s
    acceleration: tuple[float, float, float]  # (ax, ay, az) in m/s^2
    attitude_rpy: tuple[float, float, float]  # (roll, pitch, yaw) in radians
    angular_velocity: tuple[float, float, float]  # (p, q, r) in rad/s
    battery_wh: float
    battery_pct: float
    waypoint_index: int
    action: str = "fly"
    target_bin_id: str | None = None

    @property
    def speed_mps(self) -> float:
        vx, vy, vz = self.velocity
        return float((vx**2 + vy**2 + vz**2) ** 0.5)

    def to_telemetry_dict(self) -> dict[str, Any]:
        """Convert state into standard JSON-serializable telemetry record."""
        return {
            "t_s": round(self.timestamp_s, 3),
            "pos": [round(v, 4) for v in self.position],
            "vel": [round(v, 3) for v in self.velocity],
            "acc": [round(v, 3) for v in self.acceleration],
            "rpy_rad": [round(v, 4) for v in self.attitude_rpy],
            "rpy_deg": [round(v * 57.2957795, 2) for v in self.attitude_rpy],
            "speed_mps": round(self.speed_mps, 3),
            "battery_wh": round(self.battery_wh, 3),
            "battery_pct": round(self.battery_pct, 2),
            "wp_idx": self.waypoint_index,
            "action": self.action,
            "target_bin": self.target_bin_id,
        }


@dataclass
class TelemetryRecord:
    """Convenience telemetry container for serialization and export."""
    records: list[dict[str, Any]] = field(default_factory=list)

    def to_jsonl(self) -> str:
        return "\n".join(json.dumps(r) for r in self.records)


@dataclass
class SimulationResult:
    """Comprehensive summary of a kinematic flight simulation execution."""
    mission_id: str
    flight_id: int
    states: list[DroneState]
    total_time_s: float
    total_distance_flown_m: float
    tracking_errors_m: list[float]
    mean_tracking_error_m: float
    p95_tracking_error_m: float
    max_tracking_error_m: float
    initial_battery_wh: float
    final_battery_wh: float
    energy_consumed_wh: float
    final_battery_pct: float
    reproducibility_hash: str
    planned_waypoints_count: int

    @property
    def sample_count(self) -> int:
        return len(self.states)

    def compute_hash(self) -> str:
        """Compute SHA-256 fingerprint over position and battery telemetry sequence."""
        h = hashlib.sha256()
        for s in self.states:
            pos_str = f"{s.position[0]:.4f},{s.position[1]:.4f},{s.position[2]:.4f}"
            bat_str = f"{s.battery_wh:.4f}"
            h.update(f"{pos_str}|{bat_str};".encode())
        return h.hexdigest()
