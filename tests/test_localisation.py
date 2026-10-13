"""Unit tests for sensor fusion, EKF, IMU dead-reckoning, marker detection, and drift."""

import pytest

from drone_nav.config.loader import load_config
from drone_nav.localisation.detector import MarkerDetector
from drone_nav.localisation.ekf import ExtendedKalmanFilter
from drone_nav.localisation.estimator import (
    LocalisationEstimator,
    run_marker_spacing_sweep,
)
from drone_nav.localisation.imu import DeadReckoningIntegrator, IMUSimulator
from drone_nav.localisation.plots import (
    export_localisation_error_svg,
    export_localisation_trajectory_svg,
)
from drone_nav.planner.planner import MissionPlanner
from drone_nav.simulator.sim import FlightSimulator
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def small_setup():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    planner = MissionPlanner(world, config)
    mission = planner.plan_mission(strategy_name="serpentine")
    sim = FlightSimulator(world, config, dt=0.05, seed=42)
    sim_res = sim.simulate_flight(mission.flights[0])
    return config, world, mission.flights[0], sim_res


def test_marker_detector_visibility(small_setup):
    config, world, _, _ = small_setup
    detector = MarkerDetector(config, seed=42)

    # Marker TAG-WH-SMALL-R01-B01 is at (2.6, 2.8, 0.7) facing +X (1, 0, 0)
    # Drone placed in front of marker at (3.5, 2.8, 0.7) facing -X (pi)
    drone_pos = (3.5, 2.8, 0.7)
    drone_yaw = 3.14159

    obs = detector.detect_markers(drone_pos, drone_yaw, world.markers, timestamp_s=1.0)
    assert len(obs) >= 1
    assert any("R01-B01" in o.marker_id for o in obs)

    # Test out-of-range (>4.5m)
    far_pos = (15.0, 2.8, 0.7)
    obs_far = detector.detect_markers(far_pos, drone_yaw, world.markers, timestamp_s=1.0)
    assert not any("R01-B01" in o.marker_id for o in obs_far)

    # Test drone behind marker (e.g. x < 2.6)
    behind_pos = (2.0, 2.8, 0.7)
    obs_behind = detector.detect_markers(behind_pos, 0.0, world.markers, timestamp_s=1.0)
    assert not any("R01-B01" in o.marker_id for o in obs_behind)


def test_imu_sensor_and_dead_reckoning(small_setup):
    config, _, _, _ = small_setup
    imu = IMUSimulator(config, seed=42)
    dr = DeadReckoningIntegrator(initial_pos=(0.0, 0.0, 0.0))

    # Static drone with 0 acceleration
    for _ in range(100):
        a_meas, w_meas = imu.measure((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), dt=0.05)
        dr.step(a_meas, dt=0.05)

    # Measured accel should contain non-zero bias
    assert abs(a_meas[0]) > 0.001
    # Open loop dead-reckoning should accumulate drift even when static
    assert dr.pos[0] > 0.01


def test_ekf_marker_update_convergence(small_setup):
    config, _, _, _ = small_setup
    # Initialized with a 0.5m offset
    ekf = ExtendedKalmanFilter(config, initial_pos=(0.5, 0.5, 0.5))

    initial_uncertainty = ekf.position_uncertainty_3sigma

    # Perform repeated marker updates at true position (0, 0, 0)
    for _ in range(30):
        ekf.predict((0.0, 0.0, 0.0), dt=0.05)
        ekf.update_marker((0.0, 0.0, 0.0))

    # Position error should have shrunk dramatically
    err = (ekf.position[0] ** 2 + ekf.position[1] ** 2 + ekf.position[2] ** 2) ** 0.5
    assert err < 0.05
    # Covariance uncertainty should have contracted
    assert ekf.position_uncertainty_3sigma < initial_uncertainty


def test_full_localisation_pipeline(small_setup):
    config, world, _, sim_res = small_setup
    estimator = LocalisationEstimator(world, config, seed=42)
    loc_res = estimator.run_localisation(sim_res)

    assert loc_res.total_time_s > 10.0
    assert loc_res.total_marker_detections > 500
    assert loc_res.mean_ekf_error_m < 0.10  # Under 10cm mean error
    assert loc_res.p95_ekf_error_m < 0.15   # Under 15cm P95 error
    assert loc_res.max_ekf_error_m < 0.25   # Under 25cm max error
    # Unconstrained dead-reckoning should diverge significantly (>1,000m)
    assert loc_res.max_dr_error_m > 1000.0


def test_marker_spacing_sweep(small_setup):
    config, world, flight, _ = small_setup
    sweep_rows = run_marker_spacing_sweep(
        world_base=world,
        config=config,
        flight_plan=flight,
        spacing_candidates=[1, 2, 4],
        seed=42,
    )

    assert len(sweep_rows) == 3
    assert sweep_rows[0].spacing_bays == 1
    assert sweep_rows[1].spacing_bays == 2
    assert sweep_rows[2].spacing_bays == 4

    # Marker density decreases: N=1 has more markers than N=4
    assert sweep_rows[0].total_markers > sweep_rows[2].total_markers
    assert sweep_rows[0].marker_detections > sweep_rows[2].marker_detections


def test_marker_dropout_failure_case(small_setup):
    config, world, _, sim_res = small_setup
    estimator = LocalisationEstimator(world, config, seed=42)

    # Standard run without dropout
    normal_res = estimator.run_localisation(sim_res)

    # Run with injected 60s dropout (from t=50 to t=110)
    drop_res = estimator.run_localisation(
        sim_res, marker_drop_interval=(50.0, 110.0)
    )

    # During the 60s dropout, fewer markers should be seen
    assert drop_res.total_marker_detections < normal_res.total_marker_detections
    # Max error should be higher due to temporary drift during the blackout gap
    assert drop_res.max_ekf_error_m >= normal_res.max_ekf_error_m


def test_localisation_plots_generation(small_setup, tmp_path):
    config, world, _, sim_res = small_setup
    estimator = LocalisationEstimator(world, config, seed=42)
    loc_res = estimator.run_localisation(sim_res)

    p_traj = tmp_path / "loc_traj.svg"
    p_err = tmp_path / "loc_err.svg"

    out1 = export_localisation_trajectory_svg(world, loc_res, p_traj)
    out2 = export_localisation_error_svg(loc_res, p_err)

    assert out1.exists()
    assert out2.exists()
    assert "<svg" in out1.read_text(encoding="utf-8")
    assert "<svg" in out2.read_text(encoding="utf-8")
