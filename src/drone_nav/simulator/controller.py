"""Cascaded trajectory tracking controller for 6-DOF kinematic drone simulation."""

import math

import numpy as np

from drone_nav.config.models import DroneConfig


class TrajectoryTrackingController:
    """Computes commanded accelerations and attitude to track 3D waypoints within physical limits."""

    def __init__(self, config: DroneConfig) -> None:
        self.config = config
        self.max_speed = config.max_speed_mps
        self.max_accel = config.max_accel_mps2
        self.max_yaw_rate = config.max_yaw_rate_radps

        # Control gains
        self.kp_pos = 1.6  # Position error to velocity gain
        self.kv_vel = 2.4  # Velocity error to acceleration gain
        self.kp_yaw = 2.0  # Heading error to yaw rate gain
        self.gravity = 9.81

    def compute_control(
        self,
        current_pos: tuple[float, float, float],
        current_vel: tuple[float, float, float],
        current_yaw: float,
        target_pos: tuple[float, float, float],
        target_speed: float,
        target_yaw: float,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        """Compute commanded acceleration (ax, ay, az), attitude (roll, pitch, yaw), and yaw rate.

        Returns:
            cmd_accel: (ax, ay, az) in m/s^2
            cmd_rpy: (roll, pitch, yaw) in radians
            cmd_yaw_rate: d(yaw)/dt in rad/s
        """
        p_curr = np.array(current_pos, dtype=float)
        v_curr = np.array(current_vel, dtype=float)
        p_targ = np.array(target_pos, dtype=float)

        # 1. Position error
        pos_err = p_targ - p_curr
        dist_to_targ = float(np.linalg.norm(pos_err))

        # 2. Velocity command with smooth deceleration profile
        desired_speed = min(self.max_speed, target_speed)
        # Slow down smoothly when within stopping distance (v^2 = 2 * a * d)
        stopping_dist = (desired_speed**2) / (2.0 * max(0.5, self.max_accel * 0.8))
        if dist_to_targ < stopping_dist and stopping_dist > 1e-4:
            approach_speed = desired_speed * math.sqrt(max(0.01, dist_to_targ / stopping_dist))
        else:
            approach_speed = desired_speed

        if dist_to_targ > 1e-3:
            unit_dir = pos_err / dist_to_targ
            v_cmd = unit_dir * approach_speed
        else:
            v_cmd = np.zeros(3)

        # 3. Acceleration command via proportional velocity feedback
        vel_err = v_cmd - v_curr
        a_cmd = self.kv_vel * vel_err

        # Saturate acceleration magnitude to physical limits
        accel_mag = float(np.linalg.norm(a_cmd))
        if accel_mag > self.max_accel:
            a_cmd = (a_cmd / accel_mag) * self.max_accel

        # 4. Attitude computation (Roll & Pitch from horizontal accelerations)
        # In quadrotor kinematics, accelerating forward requires pitching down,
        # accelerating right requires rolling right
        ax, ay, az = float(a_cmd[0]), float(a_cmd[1]), float(a_cmd[2])
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        # Body-frame forward and right accelerations
        a_body_x = ax * cos_yaw + ay * sin_yaw
        a_body_y = -ax * sin_yaw + ay * cos_yaw

        # Clamp max tilt angle to 35 degrees (0.61 rad) for safety
        max_tilt = math.radians(35.0)
        pitch_cmd = math.atan2(a_body_x, self.gravity)
        pitch_cmd = max(-max_tilt, min(max_tilt, pitch_cmd))

        roll_cmd = math.atan2(-a_body_y, self.gravity)
        roll_cmd = max(-max_tilt, min(max_tilt, roll_cmd))

        # 5. Yaw rate command with shortest angular path
        yaw_err = math.atan2(
            math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw)
        )
        yaw_rate_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.kp_yaw * yaw_err))

        return (ax, ay, az), (roll_cmd, pitch_cmd, current_yaw), yaw_rate_cmd
