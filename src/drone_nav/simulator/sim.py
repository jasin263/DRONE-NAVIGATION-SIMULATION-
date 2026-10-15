"""Kinematic flight simulator with physics limits, battery drain, and wind disturbances."""

import math
from typing import Any

import numpy as np

from drone_nav.config.models import WarehouseConfig
from drone_nav.planner.models import FlightPlan
from drone_nav.simulator.battery import BatteryDischargeSimulator
from drone_nav.simulator.controller import TrajectoryTrackingController
from drone_nav.simulator.models import DroneState, SimulationResult
from drone_nav.simulator.wind import WindModel
from drone_nav.world.models import World


def _to_f3(arr: np.ndarray) -> tuple[float, float, float]:
    return (float(arr[0]), float(arr[1]), float(arr[2]))


class FlightSimulator:
    """Deterministic kinematic simulator integrating trajectory tracking, battery, and wind."""

    def __init__(
        self,
        world: World,
        config: WarehouseConfig,
        dt: float = 0.05,
        seed: int = 42,
        wind_speed: float = 0.2,
        supervisor: Any | None = None,
    ) -> None:
        self.world = world
        self.config = config
        self.dt = dt
        self.seed = seed
        self.wind_speed = wind_speed
        self.supervisor = supervisor

        self.controller = TrajectoryTrackingController(config.drone)
        self.wind_model = WindModel(
            base_wind=(0.1, 0.0, 0.0), gust_intensity=wind_speed, seed=seed
        )
        self.battery_sim = BatteryDischargeSimulator(config.drone.battery)

    def simulate_flight(
        self, flight_plan: FlightPlan, mission_id: str = "MISSION-SIM"
    ) -> SimulationResult:
        """Run numerical kinematic simulation over a flight plan's waypoints."""
        if not flight_plan.waypoints:
            raise ValueError("Flight plan contains no waypoints to simulate")

        states: list[DroneState] = []
        tracking_errors: list[float] = []

        # Initial conditions at first waypoint
        first_wp = flight_plan.waypoints[0]
        pos = np.array(first_wp.position, dtype=float)
        vel = np.zeros(3, dtype=float)
        acc = np.zeros(3, dtype=float)
        yaw = first_wp.yaw_rad
        attitude = (0.0, 0.0, yaw)

        t_sim = 0.0
        total_dist_flown = 0.0
        initial_battery = self.battery_sim.remaining_wh

        prev_pos = np.array(first_wp.position, dtype=float)
        wp_idx = 0
        acceptance_radius_m = 0.20  # Waypoint arrival threshold

        while wp_idx < len(flight_plan.waypoints):
            target_wp = flight_plan.waypoints[wp_idx]
            targ_pos = np.array(target_wp.position, dtype=float)

            # Compute cross-track tracking error to current flight segment
            seg = targ_pos - prev_pos
            seg_len_sq = float(np.dot(seg, seg))
            if seg_len_sq > 1e-6:
                u = float(np.clip(np.dot(pos - prev_pos, seg) / seg_len_sq, 0.0, 1.0))
                closest_pt = prev_pos + u * seg
                cross_track_err = float(np.linalg.norm(pos - closest_pt))
            else:
                cross_track_err = float(np.linalg.norm(pos - targ_pos))
            tracking_errors.append(cross_track_err)

            # Check if waypoint reached
            dist_to_targ = float(np.linalg.norm(targ_pos - pos))
            if dist_to_targ <= acceptance_radius_m:
                # If this is a scan waypoint with dwell time, simulate hover dwell
                if target_wp.action.value == "scan" and target_wp.wait_time_s > 0.0:
                    dwell_steps = int(target_wp.wait_time_s / self.dt)
                    for _ in range(dwell_steps):
                        # Wind step
                        self.wind_model.step(self.dt)
                        a_drag = self.wind_model.compute_drag_acceleration(
                            _to_f3(vel), self.config.drone.mass_kg
                        )
                        # Hover control counteracting drag
                        acc = np.array([-a_drag[0], -a_drag[1], -a_drag[2]])
                        vel = vel * 0.9  # damp out velocity during hover
                        total_dist_flown += float(np.linalg.norm(vel * self.dt))
                        pos += vel * self.dt

                        self.battery_sim.step(
                            _to_f3(vel), _to_f3(acc), self.dt, payload_power_w=18.0
                        )

                        t_sim += self.dt
                        states.append(
                            DroneState(
                                timestamp_s=t_sim,
                                position=_to_f3(pos),
                                velocity=_to_f3(vel),
                                acceleration=_to_f3(acc),
                                attitude_rpy=attitude,
                                angular_velocity=(0.0, 0.0, 0.0),
                                battery_wh=self.battery_sim.remaining_wh,
                                battery_pct=self.battery_sim.percentage,
                                waypoint_index=wp_idx,
                                action="scan_dwell",
                                target_bin_id=target_wp.target_bin_id,
                            )
                        )

                prev_pos = targ_pos.copy()
                wp_idx += 1
                if wp_idx >= len(flight_plan.waypoints):
                    break
                target_wp = flight_plan.waypoints[wp_idx]
                targ_pos = np.array(target_wp.position, dtype=float)

            # Trajectory Tracking Controller Step
            a_cmd, rpy_cmd, yaw_rate_cmd = self.controller.compute_control(
                current_pos=_to_f3(pos),
                current_vel=_to_f3(vel),
                current_yaw=yaw,
                target_pos=_to_f3(targ_pos),
                target_speed=target_wp.speed_mps,
                target_yaw=target_wp.yaw_rad,
            )

            # Wind Disturbance Drag Step
            self.wind_model.step(self.dt)
            a_wind_drag = self.wind_model.compute_drag_acceleration(
                _to_f3(vel), self.config.drone.mass_kg
            )

            # Net acceleration
            acc_net = np.array(a_cmd, dtype=float) + np.array(a_wind_drag, dtype=float)
            acc = acc_net

            # Kinematic numerical integration
            vel += acc_net * self.dt

            action_label = target_wp.action.value
            if self.supervisor is not None:
                failsafe, safe_vel, speed_override = self.supervisor.evaluate_step(
                    timestamp_s=t_sim,
                    position=_to_f3(pos),
                    velocity=_to_f3(vel),
                    battery_pct=self.battery_sim.percentage,
                )
                vel = np.array(safe_vel, dtype=float)
                max_spd = (
                    min(self.config.drone.max_speed_mps, speed_override)
                    if speed_override is not None
                    else self.config.drone.max_speed_mps
                )
                if failsafe.value != "none":
                    action_label = f"failsafe_{failsafe.value}"
            else:
                max_spd = self.config.drone.max_speed_mps

            # Enforce max speed limit
            speed = float(np.linalg.norm(vel))
            if speed > max_spd:
                vel = (vel / speed) * max_spd

            step_dist = float(np.linalg.norm(vel * self.dt))
            total_dist_flown += step_dist
            pos += vel * self.dt

            # Yaw integration
            yaw += yaw_rate_cmd * self.dt
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            attitude = (rpy_cmd[0], rpy_cmd[1], yaw)

            # Battery discharge update
            self.battery_sim.step(
                _to_f3(vel), _to_f3(acc), self.dt, payload_power_w=12.0
            )

            t_sim += self.dt

            states.append(
                DroneState(
                    timestamp_s=t_sim,
                    position=_to_f3(pos),
                    velocity=_to_f3(vel),
                    acceleration=_to_f3(acc),
                    attitude_rpy=attitude,
                    angular_velocity=(0.0, 0.0, yaw_rate_cmd),
                    battery_wh=self.battery_sim.remaining_wh,
                    battery_pct=self.battery_sim.percentage,
                    waypoint_index=wp_idx,
                    action=action_label,
                    target_bin_id=target_wp.target_bin_id,
                )
            )

        # Degenerate plans (e.g. a single waypoint already at the start position)
        # never enter the integration loop; emit at least one state so downstream
        # telemetry consumers always receive a non-empty trace.
        if not states:
            states.append(
                DroneState(
                    timestamp_s=t_sim,
                    position=_to_f3(pos),
                    velocity=_to_f3(vel),
                    acceleration=_to_f3(acc),
                    attitude_rpy=attitude,
                    angular_velocity=(0.0, 0.0, yaw),
                    battery_wh=self.battery_sim.remaining_wh,
                    battery_pct=self.battery_sim.percentage,
                    waypoint_index=0,
                    action=first_wp.action.value,
                    target_bin_id=first_wp.target_bin_id,
                )
            )

        # Statistics
        mean_err = float(np.mean(tracking_errors)) if tracking_errors else 0.0
        p95_err = float(np.percentile(tracking_errors, 95)) if tracking_errors else 0.0
        max_err = float(np.max(tracking_errors)) if tracking_errors else 0.0

        res = SimulationResult(
            mission_id=mission_id,
            flight_id=flight_plan.flight_id,
            states=states,
            total_time_s=t_sim,
            total_distance_flown_m=total_dist_flown,
            tracking_errors_m=tracking_errors,
            mean_tracking_error_m=round(mean_err, 4),
            p95_tracking_error_m=round(p95_err, 4),
            max_tracking_error_m=round(max_err, 4),
            initial_battery_wh=round(initial_battery, 4),
            final_battery_wh=round(self.battery_sim.remaining_wh, 4),
            energy_consumed_wh=round(self.battery_sim.total_consumed_wh, 4),
            final_battery_pct=round(self.battery_sim.percentage, 2),
            reproducibility_hash="",
            planned_waypoints_count=len(flight_plan.waypoints),
        )
        res.reproducibility_hash = res.compute_hash()
        return res
