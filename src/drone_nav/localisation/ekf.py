"""9-State Extended Kalman Filter fusing IMU accelerometry with visual fiducial fixes."""

import numpy as np

from drone_nav.config.models import WarehouseConfig


class ExtendedKalmanFilter:
    """Fuses IMU dead-reckoning with asynchronous visual marker position fixes."""

    def __init__(
        self,
        config: WarehouseConfig,
        initial_pos: tuple[float, float, float],
        initial_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.config = config
        ekf_cfg = config.localisation.fusion_filter
        drift_cfg = config.localisation.drift_model

        # State vector x: [px, py, pz, vx, vy, vz, bax, bay, baz]^T
        self.x = np.zeros(9, dtype=float)
        self.x[0:3] = initial_pos
        self.x[3:6] = initial_vel
        self.x[6:9] = 0.0  # initial bias estimate

        # Initial covariance matrix P
        self.P = np.diag([
            0.01, 0.01, 0.01,  # pos variance (m^2)
            0.05, 0.05, 0.05,  # vel variance (m/s)^2
            0.02, 0.02, 0.02,  # bias variance (m/s^2)^2
        ]).astype(float)

        self.q_pos = ekf_cfg.process_noise_pos
        self.q_vel = ekf_cfg.process_noise_vel
        self.q_bias = drift_cfg.walk_std_mps
        self.r_marker = ekf_cfg.meas_noise_marker

    @property
    def position(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def velocity(self) -> tuple[float, float, float]:
        return float(self.x[3]), float(self.x[4]), float(self.x[5])

    @property
    def accel_bias(self) -> tuple[float, float, float]:
        return float(self.x[6]), float(self.x[7]), float(self.x[8])

    @property
    def position_uncertainty_3sigma(self) -> float:
        """3-sigma spherical position uncertainty radius in metres."""
        var_pos = float(self.P[0, 0] + self.P[1, 1] + self.P[2, 2])
        return 3.0 * float(np.sqrt(max(1e-8, var_pos)))

    def predict(self, accel_meas: tuple[float, float, float], dt: float) -> None:
        """Predict step propagating state and covariance via kinematics and IMU."""
        a_m = np.array(accel_meas, dtype=float)
        b_a = self.x[6:9]
        a_corr = a_m - b_a

        # State integration matching kinematic drone simulator
        v_next = self.x[3:6] + a_corr * dt
        self.x[0:3] = self.x[0:3] + v_next * dt
        self.x[3:6] = v_next

        # State transition Jacobian F
        I3 = np.eye(3)
        O3 = np.zeros((3, 3))
        F = np.block([
            [I3, I3 * dt, -I3 * (dt**2)],
            [O3, I3, -I3 * dt],
            [O3, O3, I3],
        ])

        # Continuous-to-discrete process noise covariance Q
        var_p = (self.q_pos * dt) ** 2
        var_v = (self.q_vel * dt) ** 2
        var_b = (self.q_bias * dt) ** 2
        Q = np.diag([
            var_p, var_p, var_p,
            var_v, var_v, var_v,
            var_b, var_b, var_b,
        ])

        # Covariance prediction
        self.P = F @ self.P @ F.T + Q

    def update_velocity(
        self, meas_velocity: tuple[float, float, float], vel_noise: float = 0.05
    ) -> None:
        """Update step fusing visual odometry / optical flow velocity measurement."""
        z_v = np.array(meas_velocity, dtype=float)
        Hv = np.zeros((3, 9), dtype=float)
        Hv[0:3, 3:6] = np.eye(3)

        Rv = np.eye(3, dtype=float) * (vel_noise**2)
        y = z_v - self.x[3:6]
        S = Hv @ self.P @ Hv.T + Rv
        K = self.P @ Hv.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I9 = np.eye(9)
        IKH = I9 - K @ Hv
        self.P = IKH @ self.P @ IKH.T + K @ Rv @ K.T

    def update_marker(self, marker_observed_pos: tuple[float, float, float]) -> None:
        """Update step fusing a detected fiducial marker 3D position fix."""
        z = np.array(marker_observed_pos, dtype=float)

        # Measurement matrix H: maps state x to observed 3D position
        H = np.zeros((3, 9), dtype=float)
        H[0:3, 0:3] = np.eye(3)

        # Measurement noise covariance R
        R = np.eye(3, dtype=float) * (self.r_marker**2)

        # Innovation
        y = z - self.x[0:3]

        # Innovation covariance S
        S = H @ self.P @ H.T + R

        # Kalman gain K
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Joseph-form covariance update: P = (I - KH) P (I - KH)^T + K R K^T
        I9 = np.eye(9)
        IKH = I9 - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
