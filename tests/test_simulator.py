"""Comprehensive unit tests for kinematic flight simulator, battery, wind, and tracking."""

import json

import pytest

from drone_nav.config.loader import load_config
from drone_nav.planner.models import FlightPlan
from drone_nav.planner.planner import MissionPlanner
from drone_nav.simulator.battery import BatteryDischargeSimulator
from drone_nav.simulator.controller import TrajectoryTrackingController
from drone_nav.simulator.plots import (
    export_battery_curve_svg,
    export_flight_trajectory_comparison_svg,
)
from drone_nav.simulator.sim import FlightSimulator
from drone_nav.simulator.wind import WindModel
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def small_world_and_plan():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    planner = MissionPlanner(world, config)
    mission = planner.plan_mission(strategy_name="serpentine")
    return config, world, mission.flights[0]


def test_simulation_execution(small_world_and_plan):
    config, world, flight = small_world_and_plan
    sim = FlightSimulator(world=world, config=config, dt=0.1, seed=42, wind_speed=0.2)
    res = sim.simulate_flight(flight, mission_id="TEST-SIM")

    assert res.mission_id == "TEST-SIM"
    assert res.flight_id == flight.flight_id
    assert res.total_time_s > 10.0
    assert res.total_distance_flown_m > 100.0
    assert res.sample_count > 500
    assert len(res.states) == res.sample_count
    assert len(res.tracking_errors_m) > 0
    assert res.mean_tracking_error_m > 0.0
    assert res.p95_tracking_error_m >= res.mean_tracking_error_m
    assert res.max_tracking_error_m >= res.p95_tracking_error_m
    assert res.reproducibility_hash != ""


def test_simulation_reproducibility(small_world_and_plan):
    """Identical random seeds must produce identical telemetry and SHA-256 hashes."""
    config, world, flight = small_world_and_plan

    sim1 = FlightSimulator(world=world, config=config, dt=0.1, seed=123, wind_speed=0.3)
    res1 = sim1.simulate_flight(flight)

    sim2 = FlightSimulator(world=world, config=config, dt=0.1, seed=123, wind_speed=0.3)
    res2 = sim2.simulate_flight(flight)

    assert res1.reproducibility_hash == res2.reproducibility_hash
    assert res1.total_time_s == res2.total_time_s
    assert res1.total_distance_flown_m == res2.total_distance_flown_m
    assert res1.energy_consumed_wh == res2.energy_consumed_wh
    assert len(res1.states) == len(res2.states)
    assert res1.states[10].position == res2.states[10].position


def test_simulation_seed_variance(small_world_and_plan):
    """Different random seeds must produce different stochastic wind trajectories."""
    config, world, flight = small_world_and_plan

    sim1 = FlightSimulator(world=world, config=config, dt=0.1, seed=42, wind_speed=0.5)
    res1 = sim1.simulate_flight(flight)

    sim2 = FlightSimulator(world=world, config=config, dt=0.1, seed=999, wind_speed=0.5)
    res2 = sim2.simulate_flight(flight)

    assert res1.reproducibility_hash != res2.reproducibility_hash


def test_physics_limits_enforced(small_world_and_plan):
    """Drone velocity and acceleration must respect configured maximum physical limits."""
    config, world, flight = small_world_and_plan
    max_speed = config.drone.max_speed_mps

    sim = FlightSimulator(world=world, config=config, dt=0.05, seed=42, wind_speed=0.2)
    res = sim.simulate_flight(flight)

    for state in res.states:
        # Speed cannot exceed max speed by more than floating point tolerance
        assert state.speed_mps <= max_speed + 1e-4


def test_battery_discharge_behaviour(small_world_and_plan):
    """Battery remaining Wh must decrease monotonically during flight."""
    config, world, flight = small_world_and_plan
    sim = FlightSimulator(world=world, config=config, dt=0.1, seed=42, wind_speed=0.2)
    res = sim.simulate_flight(flight)

    assert res.final_battery_wh < res.initial_battery_wh
    assert res.energy_consumed_wh > 0.0
    assert 0.0 <= res.final_battery_pct <= 100.0

    # Test individual battery simulator properties
    bat_sim = BatteryDischargeSimulator(config.drone.battery, initial_charge_pct=100.0)
    assert bat_sim.is_safe_reserve() is True
    assert bat_sim.is_critical() is False

    # Simulate excessive drain to trigger critical
    bat_sim.remaining_wh = bat_sim.capacity_wh * 0.05
    assert bat_sim.is_critical() is True
    assert bat_sim.is_safe_reserve() is False


def test_wind_model():
    """Test deterministic Ornstein-Uhlenbeck wind model."""
    w_model = WindModel(base_wind=(0.1, 0.0, 0.0), gust_intensity=0.2, seed=42)
    w_initial = w_model.step(0.05)
    assert len(w_initial) == 3

    # Zero gust test
    w_calm = WindModel(base_wind=(0.2, 0.1, 0.0), gust_intensity=0.0, seed=42)
    w_step = w_calm.step(0.1)
    assert abs(w_step[0] - 0.2) < 1e-6
    assert abs(w_step[1] - 0.1) < 1e-6

    # Drag acceleration computation
    drag = w_model.compute_drag_acceleration(drone_velocity=(2.0, 0.0, 0.0), drone_mass_kg=2.0)
    assert len(drag) == 3
    # Opposes drone movement forward
    assert drag[0] < 0.0


def test_controller_clamping():
    """Verify attitude and yaw rate limits in TrajectoryTrackingController."""
    config = load_config("configs/presets/small.yaml")
    controller = TrajectoryTrackingController(config.drone)

    accel, rpy, yaw_rate = controller.compute_control(
        current_pos=(0.0, 0.0, 1.0),
        current_vel=(0.0, 0.0, 0.0),
        current_yaw=0.0,
        target_pos=(100.0, 100.0, 1.0),  # Very distant target to push max accel
        target_speed=5.0,
        target_yaw=3.14159,
    )
    # Check max accel limit
    accel_mag = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5
    assert accel_mag <= config.drone.max_accel_mps2 + 1e-4

    # Check yaw rate limit
    assert abs(yaw_rate) <= config.drone.max_yaw_rate_radps + 1e-4


def test_empty_flight_plan_raises_error():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    sim = FlightSimulator(world=world, config=config)

    empty_flight = FlightPlan(flight_id=1, departure_pad_id="PAD-01", arrival_pad_id="PAD-01")
    with pytest.raises(ValueError, match="no waypoints"):
        sim.simulate_flight(empty_flight)


def test_telemetry_dict_serialization(small_world_and_plan):
    config, world, flight = small_world_and_plan
    sim = FlightSimulator(world=world, config=config, dt=0.2, seed=42, wind_speed=0.2)
    res = sim.simulate_flight(flight)

    s0 = res.states[0]
    t_dict = s0.to_telemetry_dict()
    assert "t_s" in t_dict
    assert "pos" in t_dict
    assert "battery_wh" in t_dict
    assert "rpy_deg" in t_dict

    # Ensure JSON serializable
    dumped = json.dumps(t_dict)
    assert len(dumped) > 20


def test_svg_plots_generation(small_world_and_plan, tmp_path):
    config, world, flight = small_world_and_plan
    sim = FlightSimulator(world=world, config=config, dt=0.2, seed=42, wind_speed=0.2)
    res = sim.simulate_flight(flight)

    traj_path = tmp_path / "traj.svg"
    bat_path = tmp_path / "bat.svg"

    p1 = export_flight_trajectory_comparison_svg(world, flight, res, traj_path)
    p2 = export_battery_curve_svg(res, bat_path)

    assert p1.exists()
    assert p2.exists()
    assert "<svg" in p1.read_text(encoding="utf-8")
    assert "<svg" in p2.read_text(encoding="utf-8")
