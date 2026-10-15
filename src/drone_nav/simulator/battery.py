"""Dynamic drone battery discharge simulator based on configurable power models."""

import math
from collections.abc import Sequence

from drone_nav.config.models import BatteryConfig


class BatteryDischargeSimulator:
    """Simulates real-time energy drain based on drone movement and hover states."""

    def __init__(self, config: BatteryConfig, initial_charge_pct: float = 100.0) -> None:
        self.config = config
        self.capacity_wh = config.capacity_wh
        self.remaining_wh = (initial_charge_pct / 100.0) * self.capacity_wh
        self.total_consumed_wh = 0.0

    @property
    def percentage(self) -> float:
        if self.capacity_wh <= 1e-6:
            return 0.0
        return max(0.0, min(100.0, (self.remaining_wh / self.capacity_wh) * 100.0))

    def step(
        self,
        velocity: tuple[float, float, float] | Sequence[float],
        acceleration: tuple[float, float, float] | Sequence[float],
        dt: float,
        payload_power_w: float = 12.0,
    ) -> float:
        """Compute instantaneous power draw and update remaining battery capacity."""
        vx, vy, vz = float(velocity[0]), float(velocity[1]), float(velocity[2])
        horiz_speed = math.hypot(vx, vy)
        climb_speed = max(0.0, vz)

        # Baseline hover power
        p_hover = self.config.hover_power_w

        # Forward translational aerodynamic cost
        p_forward = self.config.forward_power_w_per_mps * horiz_speed

        # Vertical climb gravitational potential power
        p_climb = self.config.climb_power_w_per_mps * climb_speed

        # Acceleration inertia power
        ax, ay, az = float(acceleration[0]), float(acceleration[1]), float(acceleration[2])
        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5
        p_accel = 5.0 * accel_mag

        total_power_w = p_hover + p_forward + p_climb + p_accel + payload_power_w

        delta_energy_wh = (total_power_w * dt) / 3600.0
        energy_before = self.remaining_wh
        self.remaining_wh = max(0.0, energy_before - delta_energy_wh)
        self.total_consumed_wh += energy_before - self.remaining_wh

        return float(total_power_w)

    def is_safe_reserve(self) -> bool:
        return self.percentage >= self.config.safe_reserve_pct

    def is_critical(self) -> bool:
        return self.percentage <= self.config.critical_reserve_pct
