"""Data models for batch scenario parameter sweeps and benchmark sweeps."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class SweepIterationResult:
    """Outcome of a single experiment run in a parameter sweep."""

    iteration: int
    params: dict[str, Any]
    distance_m: float
    flight_time_s: float
    energy_consumed_wh: float
    mean_tracking_error_cm: float
    p95_tracking_error_cm: float
    bins_scanned: int
    read_success_rate: float
    coverage_pct: float
    status: str = "COMPLETED"


@dataclass
class SweepReport:
    """Aggregated benchmark report over multiple sweep iterations."""

    sweep_name: str
    parameter_swept: str
    values_tested: list[Any]
    results: list[SweepIterationResult] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
