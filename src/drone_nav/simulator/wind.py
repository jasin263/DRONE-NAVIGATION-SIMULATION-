"""Stochastic and steady-state wind disturbance model with deterministic seeding."""

import math

import numpy as np


class WindModel:
    """Simulates warehouse air currents, HVAC drafts, and random turbulence."""

    def __init__(
        self,
        base_wind: tuple[float, float, float] = (0.0, 0.0, 0.0),
        gust_intensity: float = 0.2,
        seed: int = 42,
    ) -> None:
        self.base_wind = np.array(base_wind, dtype=float)
        self.gust_intensity = gust_intensity
        self.current_wind = self.base_wind.copy()
        self.rng = np.random.RandomState(seed)
        self.theta = 0.5  # Mean-reversion rate
        self.sigma = gust_intensity  # Volatility

    def step(self, dt: float) -> tuple[float, float, float]:
        """Update wind disturbance state via Ornstein-Uhlenbeck process and return (wx, wy, wz)."""
        if self.gust_intensity <= 1e-6:
            return float(self.base_wind[0]), float(self.base_wind[1]), float(self.base_wind[2])

        noise = self.rng.normal(0.0, 1.0, size=3)
        drift = -self.theta * (self.current_wind - self.base_wind) * dt
        diffusion = self.sigma * math.sqrt(dt) * noise
        self.current_wind += drift + diffusion

        return float(self.current_wind[0]), float(self.current_wind[1]), float(self.current_wind[2])

    def compute_drag_acceleration(
        self, drone_velocity: tuple[float, float, float], drone_mass_kg: float = 2.0
    ) -> tuple[float, float, float]:
        """Calculate aerodynamic disturbance acceleration acting on the drone body."""
        v_drone = np.array(drone_velocity, dtype=float)
        # Relative air velocity
        v_rel = self.current_wind - v_drone
        # Linearized aerodynamic drag coefficient for quadrotor in indoor warehouse
        k_drag = 0.12 / drone_mass_kg
        a_drag = k_drag * v_rel
        return float(a_drag[0]), float(a_drag[1]), float(a_drag[2])
