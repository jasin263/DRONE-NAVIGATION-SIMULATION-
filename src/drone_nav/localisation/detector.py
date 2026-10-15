"""Visual fiducial marker detector simulation with FOV, occlusion, and noise models."""

import math

import numpy as np

from drone_nav.config.models import WarehouseConfig
from drone_nav.localisation.models import MarkerObservation
from drone_nav.world.models import FiducialMarker


class MarkerDetector:
    """Simulates onboard camera detection of AprilTag/ArUco fiducial markers."""

    def __init__(self, config: WarehouseConfig, seed: int = 42) -> None:
        self.config = config
        self.rng = np.random.RandomState(seed)

        # Use the configured navigation camera FOV (shared with the scanning camera)
        self.hfov_rad = math.radians(config.drone.camera.hfov_deg)
        self.vfov_rad = math.radians(config.drone.camera.vfov_deg)
        self.base_noise_m = config.drone.sensors.marker_pos_noise
        self.miss_prob = config.drone.sensors.marker_miss_prob

        # Optical detection limit scaling with marker size
        self.max_range_m = max(3.5, config.localisation.marker_size_m * 24.0)

    def detect_markers(
        self,
        drone_pos: tuple[float, float, float],
        drone_yaw: float,
        markers: list[FiducialMarker],
        timestamp_s: float,
    ) -> list[MarkerObservation]:
        """Scan candidate markers and return successful observations with measurement noise."""
        observations: list[MarkerObservation] = []
        px, py, pz = drone_pos

        for marker in markers:
            mx, my, mz = marker.position
            dx = mx - px
            dy = my - py
            dz = mz - pz
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            # 1. Range check
            if dist > self.max_range_m or dist < 0.2:
                continue

            # 2. Facing direction check: marker normal must face toward drone
            # Vector from marker to drone is (-dx, -dy, -dz)
            nx, ny, nz = marker.normal
            normal_dot = nx * (-dx) + ny * (-dy) + nz * (-dz)
            if normal_dot <= 0.0:
                continue  # Drone is behind the marker

            # 3. Horizontal Field-of-View (bearing) check
            horiz_dist = math.hypot(dx, dy)
            if horiz_dist > 1e-4:
                los_yaw = math.atan2(dy, dx)
                yaw_diff = abs(math.atan2(math.sin(los_yaw - drone_yaw), math.cos(los_yaw - drone_yaw)))
                if yaw_diff > (self.hfov_rad / 2.0):
                    continue
            else:
                yaw_diff = 0.0

            # 4. Vertical Field-of-View (elevation) check
            pitch_angle = abs(math.atan2(dz, max(1e-3, horiz_dist)))
            if pitch_angle > (self.vfov_rad / 2.0):
                continue

            # 5. Stochastic miss probability (occlusion, blur, poor lighting)
            if self.rng.uniform(0.0, 1.0) < self.miss_prob:
                continue

            # 6. Add distance-dependent Gaussian measurement noise
            # Standard deviation grows gently with distance
            noise_std = self.base_noise_m * (1.0 + 0.4 * (dist / 2.0))
            noise = self.rng.normal(0.0, noise_std, size=3)

            # Drone position observation derived from marker solvePnP
            obs_drone_pos = (
                px + float(noise[0]),
                py + float(noise[1]),
                pz + float(noise[2]),
            )

            observations.append(
                MarkerObservation(
                    timestamp_s=timestamp_s,
                    marker_id=marker.id,
                    tag_id=marker.tag_id,
                    true_marker_pos=marker.position,
                    observed_pos=obs_drone_pos,
                    range_m=round(dist, 3),
                    bearing_deg=round(math.degrees(yaw_diff), 2),
                )
            )

        return observations
