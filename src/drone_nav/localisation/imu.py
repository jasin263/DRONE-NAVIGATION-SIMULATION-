"""IMU sensor simulation with bias drift, noise, and open-loop dead-reckoning."""

import math

import numpy as np

from drone_nav.config.models import WarehouseConfig


class IMUSimulator:
    """Simulates 6-axis MEMS inertial measurement unit with bias drift and noise."""

    def __init__(self, config: WarehouseConfig, seed: int = 42) -> None:
        self.config = config
        self.rng = np.random.RandomState(seed)

        s_cfg = config.drone.sensors
        drift_cfg = config.localisation.drift_model

        # Initial biases
        self.accel_bias = np.array([
            s_cfg.imu_accel_bias,
            s_cfg.imu_accel_bias * 0.8,
            s_cfg.imu_accel_bias * 0.5,
        ], dtype=float)
        self.gyro_bias = np.array([
            s_cfg.imu_gyro_bias,
            s_cfg.imu_gyro_bias * 0.7,
            s_cfg.imu_gyro_bias * 1.2,
        ], dtype=float)

        self.accel_noise_std = s_cfg.imu_accel_noise
        self.gyro_noise_std = s_cfg.imu_gyro_noise
        self.random_walk_std = drift_cfg.walk_std_mps

    def measure(
        self,
        true_accel: tuple[float, float, float],
        true_angular_vel: tuple[float, float, float],
        dt: float,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Simulate noisy, biased IMU readings and update internal bias random walk."""
        # Random walk drift on accelerometer bias
        drift = self.rng.normal(0.0, self.random_walk_std * math.sqrt(dt), size=3)
        self.accel_bias += drift

        # High-frequency white noise
        accel_noise = self.rng.normal(0.0, self.accel_noise_std, size=3)
        gyro_noise = self.rng.normal(0.0, self.gyro_noise_std, size=3)

        a_meas = np.array(true_accel, dtype=float) + self.accel_bias + accel_noise
        w_meas = np.array(true_angular_vel, dtype=float) + self.gyro_bias + gyro_noise

        return (
            (float(a_meas[0]), float(a_meas[1]), float(a_meas[2])),
            (float(w_meas[0]), float(w_meas[1]), float(w_meas[2])),
        )


class DeadReckoningIntegrator:
    """Open-loop double integration of raw IMU measurements showing unconstrained drift."""

    def __init__(self, initial_pos: tuple[float, float, float]) -> None:
        self.pos = np.array(initial_pos, dtype=float)
        self.vel = np.zeros(3, dtype=float)

    def step(self, accel_meas: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        a = np.array(accel_meas, dtype=float)
        # Numerical double integration
        self.pos += self.vel * dt + 0.5 * a * (dt**2)
        self.vel += a * dt
        return float(self.pos[0]), float(self.pos[1]), float(self.pos[2])
