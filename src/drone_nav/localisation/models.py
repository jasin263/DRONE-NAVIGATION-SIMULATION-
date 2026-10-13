"""Data models for sensor fusion, marker observations, and localisation evaluation."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class MarkerObservation:
    """A simulated camera detection of a fiducial marker."""

    timestamp_s: float
    marker_id: str
    tag_id: int
    true_marker_pos: tuple[float, float, float]
    observed_pos: tuple[float, float, float]
    range_m: float
    bearing_deg: float


@dataclass
class LocalisationEstimate:
    """Estimated drone navigation state at a specific simulation timestep."""

    timestamp_s: float
    true_position: tuple[float, float, float]
    estimated_position: tuple[float, float, float]
    dead_reckoning_position: tuple[float, float, float]
    estimated_velocity: tuple[float, float, float]
    estimated_accel_bias: tuple[float, float, float]
    pos_uncertainty_3sigma_m: float
    position_error_m: float
    dead_reckoning_error_m: float
    marker_observed: bool = False
    observed_marker_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "t_s": round(self.timestamp_s, 3),
            "true_pos": [round(v, 4) for v in self.true_position],
            "est_pos": [round(v, 4) for v in self.estimated_position],
            "dr_pos": [round(v, 4) for v in self.dead_reckoning_position],
            "pos_err_m": round(self.position_error_m, 4),
            "dr_err_m": round(self.dead_reckoning_error_m, 4),
            "sigma3_m": round(self.pos_uncertainty_3sigma_m, 4),
            "marker_obs": self.marker_observed,
            "marker_id": self.observed_marker_id,
        }


@dataclass
class LocalisationResult:
    """Comprehensive evaluation metrics for an EKF fusion run."""

    estimates: list[LocalisationEstimate] = field(default_factory=list)
    observations: list[MarkerObservation] = field(default_factory=list)
    total_time_s: float = 0.0
    total_markers_in_map: int = 0
    total_marker_detections: int = 0
    mean_ekf_error_m: float = 0.0
    p95_ekf_error_m: float = 0.0
    max_ekf_error_m: float = 0.0
    mean_dr_error_m: float = 0.0
    max_dr_error_m: float = 0.0
    expected_drims_error_m: float = 0.0

    @property
    def sample_count(self) -> int:
        return len(self.estimates)


@dataclass
class SpacingSweepResult:
    """Comparative row in marker spacing density benchmark."""

    spacing_bays: int
    total_markers: int
    marker_detections: int
    mean_error_m: float
    p95_error_m: float
    max_error_m: float
    max_drift_m: float
    status: str = "PASS"
