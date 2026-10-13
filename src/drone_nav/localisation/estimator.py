"""Localisation coordinator, drift analysis, and marker spacing benchmarks."""

import copy
import math

import numpy as np

from drone_nav.config.models import WarehouseConfig
from drone_nav.localisation.detector import MarkerDetector
from drone_nav.localisation.ekf import ExtendedKalmanFilter
from drone_nav.localisation.imu import DeadReckoningIntegrator, IMUSimulator
from drone_nav.localisation.models import (
    LocalisationEstimate,
    LocalisationResult,
    MarkerObservation,
    SpacingSweepResult,
)
from drone_nav.planner.models import FlightPlan
from drone_nav.simulator.models import SimulationResult
from drone_nav.simulator.sim import FlightSimulator
from drone_nav.world.generator import generate_world_from_config
from drone_nav.world.models import World


class LocalisationEstimator:
    """Orchestrates sensor simulation, EKF state estimation, and benchmark telemetry."""

    def __init__(
        self,
        world: World,
        config: WarehouseConfig,
        seed: int = 42,
    ) -> None:
        self.world = world
        self.config = config
        self.seed = seed

    def run_localisation(
        self,
        sim_result: SimulationResult,
        marker_drop_interval: tuple[float, float] | None = None,
    ) -> LocalisationResult:
        """Run EKF and dead-reckoning fusion over simulated drone flight telemetry."""
        if not sim_result.states:
            raise ValueError("Simulation result contains no telemetry states")

        imu = IMUSimulator(self.config, seed=self.seed)
        detector = MarkerDetector(self.config, seed=self.seed)

        initial_state = sim_result.states[0]
        # Assemble complete list of markers including takeoff/landing pads
        from drone_nav.world.models import FiducialMarker
        candidate_markers = list(self.world.markers)
        for pad in self.world.pads:
            candidate_markers.append(
                FiducialMarker(
                    id=f"TAG-PAD-{pad.id}",
                    tag_id=900,
                    position=(pad.position[0], pad.position[1], 0.05),
                    normal=(0.0, 0.0, 1.0),
                    size_m=0.25,
                )
            )

        rng = np.random.RandomState(self.seed)

        ekf = ExtendedKalmanFilter(
            config=self.config,
            initial_pos=initial_state.position,
            initial_vel=initial_state.velocity,
        )
        dead_reckoner = DeadReckoningIntegrator(initial_pos=initial_state.position)

        # Initial estimate at departure pad
        estimates: list[LocalisationEstimate] = [
            LocalisationEstimate(
                timestamp_s=initial_state.timestamp_s,
                true_position=initial_state.position,
                estimated_position=initial_state.position,
                dead_reckoning_position=initial_state.position,
                estimated_velocity=initial_state.velocity,
                estimated_accel_bias=(0.0, 0.0, 0.0),
                pos_uncertainty_3sigma_m=ekf.position_uncertainty_3sigma,
                position_error_m=0.0,
                dead_reckoning_error_m=0.0,
                marker_observed=True,
                observed_marker_id=f"TAG-PAD-{self.world.pads[0].id}" if self.world.pads else None,
            )
        ]
        all_observations: list[MarkerObservation] = []

        ekf_errors: list[float] = [0.0]
        dr_errors: list[float] = [0.0]

        for idx in range(1, len(sim_result.states)):
            s_prev = sim_result.states[idx - 1]
            state = sim_result.states[idx]
            dt = max(1e-4, state.timestamp_s - s_prev.timestamp_s)

            # 1. Simulate noisy IMU sensor
            a_meas, _ = imu.measure(
                true_accel=state.acceleration,
                true_angular_vel=state.angular_velocity,
                dt=dt,
            )

            # 2. Dead-reckoning integration (no fusion)
            dr_pos = dead_reckoner.step(a_meas, dt)

            # 3. EKF prediction step
            ekf.predict(a_meas, dt)

            # 4. Visual odometry / optical flow linear velocity update
            v_noise = rng.normal(0.0, 0.035, size=3)
            v_meas = (
                state.velocity[0] + float(v_noise[0]),
                state.velocity[1] + float(v_noise[1]),
                state.velocity[2] + float(v_noise[2]),
            )
            ekf.update_velocity(v_meas, vel_noise=0.04)

            # 5. Marker visual detection
            # Check if this timestamp falls inside an injected marker dropout gap
            in_drop_interval = False
            if marker_drop_interval is not None:
                t_start, t_end = marker_drop_interval
                if t_start <= state.timestamp_s <= t_end:
                    in_drop_interval = True

            marker_obs_this_step: list[MarkerObservation] = []
            if not in_drop_interval:
                marker_obs_this_step = detector.detect_markers(
                    drone_pos=state.position,
                    drone_yaw=state.attitude_rpy[2],
                    markers=candidate_markers,
                    timestamp_s=state.timestamp_s,
                )

            # 6. EKF measurement update for detected markers
            marker_hit = len(marker_obs_this_step) > 0
            observed_id = None
            for obs in marker_obs_this_step:
                ekf.update_marker(obs.observed_pos)
                all_observations.append(obs)
                observed_id = obs.marker_id

            # 7. Compute estimation errors relative to ground truth
            est_pos = ekf.position
            p_true = state.position

            err_ekf = math.sqrt(
                (est_pos[0] - p_true[0]) ** 2
                + (est_pos[1] - p_true[1]) ** 2
                + (est_pos[2] - p_true[2]) ** 2
            )
            err_dr = math.sqrt(
                (dr_pos[0] - p_true[0]) ** 2
                + (dr_pos[1] - p_true[1]) ** 2
                + (dr_pos[2] - p_true[2]) ** 2
            )

            ekf_errors.append(err_ekf)
            dr_errors.append(err_dr)

            estimates.append(
                LocalisationEstimate(
                    timestamp_s=state.timestamp_s,
                    true_position=p_true,
                    estimated_position=est_pos,
                    dead_reckoning_position=dr_pos,
                    estimated_velocity=ekf.velocity,
                    estimated_accel_bias=ekf.accel_bias,
                    pos_uncertainty_3sigma_m=ekf.position_uncertainty_3sigma,
                    position_error_m=round(err_ekf, 4),
                    dead_reckoning_error_m=round(err_dr, 4),
                    marker_observed=marker_hit,
                    observed_marker_id=observed_id,
                )
            )

        mean_ekf = float(np.mean(ekf_errors)) if ekf_errors else 0.0
        p95_ekf = float(np.percentile(ekf_errors, 95)) if ekf_errors else 0.0
        max_ekf = float(np.max(ekf_errors)) if ekf_errors else 0.0

        mean_dr = float(np.mean(dr_errors)) if dr_errors else 0.0
        max_dr = float(np.max(dr_errors)) if dr_errors else 0.0

        return LocalisationResult(
            estimates=estimates,
            observations=all_observations,
            total_time_s=sim_result.total_time_s,
            total_markers_in_map=len(self.world.markers),
            total_marker_detections=len(all_observations),
            mean_ekf_error_m=round(mean_ekf, 4),
            p95_ekf_error_m=round(p95_ekf, 4),
            max_ekf_error_m=round(max_ekf, 4),
            mean_dr_error_m=round(mean_dr, 4),
            max_dr_error_m=round(max_dr, 4),
            expected_drims_error_m=round(p95_ekf, 4),
        )


def run_marker_spacing_sweep(
    world_base: World,
    config: WarehouseConfig,
    flight_plan: FlightPlan,
    spacing_candidates: list[int] | None = None,
    seed: int = 42,
) -> list[SpacingSweepResult]:
    """Sweep marker spacing interval N bays and evaluate localisation accuracy degradation."""
    if spacing_candidates is None:
        spacing_candidates = [1, 2, 4, 8]

    # Pre-simulate true kinematic trajectory once
    flight_sim = FlightSimulator(world=world_base, config=config, dt=0.05, seed=seed)
    sim_res = flight_sim.simulate_flight(flight_plan)

    results: list[SpacingSweepResult] = []

    for n_bays in spacing_candidates:
        # Create config variant with modified spacing
        cfg_variant = copy.deepcopy(config)
        cfg_variant.localisation.marker_spacing.every_n_bays = n_bays

        # Re-generate warehouse markers
        world_variant = generate_world_from_config(cfg_variant)

        estimator = LocalisationEstimator(
            world=world_variant,
            config=cfg_variant,
            seed=seed,
        )
        res = estimator.run_localisation(sim_res)

        status = "PASS" if res.p95_ekf_error_m < 0.20 else "DEGRADED"

        results.append(
            SpacingSweepResult(
                spacing_bays=n_bays,
                total_markers=len(world_variant.markers),
                marker_detections=res.total_marker_detections,
                mean_error_m=res.mean_ekf_error_m,
                p95_error_m=res.p95_ekf_error_m,
                max_error_m=res.max_ekf_error_m,
                max_drift_m=res.max_dr_error_m,
                status=status,
            )
        )

    return results
