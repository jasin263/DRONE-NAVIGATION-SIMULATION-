"""Optical camera model, viewing geometry, and image quality degradation functions."""

import math

from drone_nav.config.models import WarehouseConfig
from drone_nav.world.models import Bin


class CameraScanner:
    """Simulates onboard scanning camera with FOV geometry and optical quality factors."""

    def __init__(self, config: WarehouseConfig) -> None:
        self.config = config
        cam_cfg = config.drone.camera

        self.res_w = cam_cfg.resolution_w
        self.res_h = cam_cfg.resolution_h
        self.hfov_rad = math.radians(cam_cfg.hfov_deg)
        self.vfov_rad = math.radians(cam_cfg.vfov_deg)
        self.mount_angle_rad = math.radians(cam_cfg.mount_angle_deg)
        self.target_standoff_m = cam_cfg.standoff_distance_m
        self.max_blur_speed_mps = cam_cfg.max_blur_speed_mps

    def compute_scan_quality(
        self,
        drone_pos: tuple[float, float, float],
        drone_yaw: float,
        drone_speed: float,
        target_bin: Bin,
    ) -> tuple[bool, float, dict[str, float]]:
        """Evaluate whether target bin is visible and calculate optical image quality Q in [0, 1].

        Returns:
            is_visible: True if bin is inside camera frustum and facing camera
            quality_score: Net image quality Q in [0.0, 1.0]
            factors: Dictionary of individual degradation factors
        """
        px, py, pz = drone_pos
        bx, by, bz = target_bin.center

        dx = bx - px
        dy = by - py
        dz = bz - pz
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # 1. Distance check (must be within reasonable focus range)
        min_range = max(0.3, self.target_standoff_m * 0.4)
        max_range = self.target_standoff_m * 2.8
        if dist < min_range or dist > max_range:
            return False, 0.0, {"dist": 0.0, "angle": 0.0, "motion": 0.0}

        # 2. Bin face normal check: bin face must face toward camera
        nx, ny, nz = target_bin.scan_face_normal
        # Vector from bin to drone is (-dx, -dy, -dz)
        dot_face = nx * (-dx) + ny * (-dy) + nz * (-dz)
        if dot_face <= 0.0:
            return False, 0.0, {"dist": 0.0, "angle": 0.0, "motion": 0.0}

        # 3. Horizontal and vertical bearing within camera FOV
        horiz_dist = math.hypot(dx, dy)
        if horiz_dist > 1e-4:
            los_yaw = math.atan2(dy, dx)
            yaw_diff = abs(math.atan2(math.sin(los_yaw - drone_yaw), math.cos(los_yaw - drone_yaw)))
            if yaw_diff > (self.hfov_rad / 2.0):
                return False, 0.0, {"dist": 0.0, "angle": 0.0, "motion": 0.0}
        else:
            yaw_diff = 0.0

        pitch_angle = abs(math.atan2(dz, max(1e-3, horiz_dist)) - self.mount_angle_rad)
        if pitch_angle > (self.vfov_rad / 2.0):
            return False, 0.0, {"dist": 0.0, "angle": 0.0, "motion": 0.0}

        # 4. Compute Image Quality Factors
        # A. Distance factor: peak at standoff distance, falls off parabolically
        dist_offset = abs(dist - self.target_standoff_m) / self.target_standoff_m
        q_dist = max(0.1, min(1.0, 1.0 - 0.7 * dist_offset))

        # B. Angle factor: cosine of incidence angle between bin normal and LOS
        cos_inc = dot_face / (dist * math.sqrt(nx**2 + ny**2 + nz**2) + 1e-6)
        q_angle = max(0.1, min(1.0, float(cos_inc**2)))

        # C. Motion blur factor: decreases as speed approaches max blur speed
        speed_ratio = min(1.5, drone_speed / max(0.1, self.max_blur_speed_mps))
        q_motion = max(0.1, min(1.0, 1.0 - 0.8 * (speed_ratio**2)))

        # Combined net quality score
        net_q = round(float(q_dist * q_angle * q_motion), 4)

        factors = {
            "dist_factor": round(float(q_dist), 4),
            "angle_factor": round(float(q_angle), 4),
            "motion_factor": round(float(q_motion), 4),
            "distance_m": round(dist, 3),
        }

        return True, net_q, factors
