"""Unit tests for safety supervisor, geofence, clearance, and failure injection."""

import math

import pytest

from drone_nav.config.loader import load_config
from drone_nav.planner.graph import NavigationGraph
from drone_nav.safety.clearance import RackClearanceMonitor
from drone_nav.safety.geofence import GeofenceMonitor
from drone_nav.safety.injector import FailureInjectionFramework
from drone_nav.safety.models import FailsafeAction, SafetyViolationType
from drone_nav.safety.replanner import ObstacleReplanner
from drone_nav.safety.supervisor import SafetySupervisor
from drone_nav.world.generator import generate_world_from_config


@pytest.fixture
def small_world_and_config():
    config = load_config("configs/presets/small.yaml")
    world = generate_world_from_config(config)
    return world, config


def test_geofence_perimeter_checks(small_world_and_config):
    world, config = small_world_and_config
    geo = GeofenceMonitor(world, config)

    # Valid interior point
    safe, msg = geo.check_outer_perimeter((5.0, 5.0, 2.0))
    assert safe is True
    assert msg is None

    # Beyond West boundary
    safe_w, msg_w = geo.check_outer_perimeter((-1.0, 5.0, 2.0))
    assert safe_w is False
    assert "West" in str(msg_w)

    # Beyond Ceiling
    safe_c, msg_c = geo.check_outer_perimeter((5.0, 5.0, 100.0))
    assert safe_c is False
    assert "Ceiling" in str(msg_c)

    # Velocity clamping
    clamped_v = geo.clamp_velocity_at_boundary((0.0, 5.0, 2.0), (-2.0, 1.0, 0.0))
    assert clamped_v[0] == 0.0
    assert clamped_v[1] == 1.0


def test_rack_clearance_monitor(small_world_and_config):
    world, config = small_world_and_config
    monitor = RackClearanceMonitor(world, config)
    first_rack = world.racks[0]
    b = first_rack.bounds

    # Center of aisle (safe distance)
    safe_pos = (b.x_max + 1.2, (b.y_min + b.y_max) / 2.0, 1.5)
    is_safe, dist, rack_id, normal = monitor.check_rack_clearance(safe_pos)
    assert is_safe is True
    assert dist >= config.safety.min_rack_clearance_m

    # Close to rack shelf (breach)
    close_pos = (b.x_max + 0.15, (b.y_min + b.y_max) / 2.0, 1.5)
    is_safe_close, dist_close, rack_id_close, normal_close = monitor.check_rack_clearance(close_pos)
    assert is_safe_close is False
    assert dist_close < config.safety.min_rack_clearance_m
    assert normal_close[0] > 0.0  # Pushes outward (+X away from rack face)


def test_obstacle_replanner_static_and_dynamic(small_world_and_config):
    world, config = small_world_and_config
    nav_graph = NavigationGraph(world, config)
    replanner = ObstacleReplanner(nav_graph)

    # Static Aisle Block
    target_aisle = world.aisles[0].id
    start_node = f"JUNC-SOUTH-{target_aisle}"
    goal_node = f"JUNC-NORTH-{target_aisle}"

    p_init = replanner.replan_path(start_node, goal_node)
    assert p_init is not None

    blocked_nodes = replanner.block_aisle(target_aisle)
    assert len(blocked_nodes) > 0

    p_detour = replanner.replan_path(start_node, goal_node)
    assert p_detour is not None
    assert not any(n in p_detour for n in blocked_nodes)

    # Dynamic Obstacle Conflict
    obs_path = [[0.0, 4.0, 0.0], [10.0, 4.0, 0.0]]
    conflict, sep, pos = replanner.detect_dynamic_conflict(
        drone_pos=(4.0, 4.0, 1.0),
        dynamic_obs_path=obs_path,
        dynamic_obs_speed=2.0,
        start_time_s=0.0,
        current_time_s=2.0,
        conflict_radius_m=2.0,
    )
    assert conflict is True
    assert sep < 2.0
    assert pos is not None


def test_safety_supervisor_evaluations(small_world_and_config):
    world, config = small_world_and_config
    sup = SafetySupervisor(world, config)

    # Safe step
    action, vel, speed_limit = sup.evaluate_step(
        timestamp_s=1.0, position=(4.0, 4.0, 1.5), velocity=(1.0, 0.0, 0.0), battery_pct=90.0
    )
    assert action == FailsafeAction.NONE

    # Battery Low Return Home
    action_rtl, _, _ = sup.evaluate_step(
        timestamp_s=50.0, position=(4.0, 4.0, 1.5), velocity=(1.0, 0.0, 0.0), battery_pct=20.0
    )
    assert action_rtl == FailsafeAction.RETURN_HOME
    assert any(e.violation_type == SafetyViolationType.BATTERY_LOW_RTL for e in sup.events)

    # Battery Critical Emergency Land
    action_land, v_land, _ = sup.evaluate_step(
        timestamp_s=60.0, position=(4.0, 4.0, 1.5), velocity=(1.0, 0.0, 0.0), battery_pct=10.0
    )
    assert action_land == FailsafeAction.EMERGENCY_LAND
    assert v_land[2] < 0.0

    # Emergency Stop
    sup.trigger_emergency_stop(timestamp_s=70.0, drone_pos=(4.0, 4.0, 1.5))
    action_estop, v_estop, _ = sup.evaluate_step(
        timestamp_s=70.1, position=(4.0, 4.0, 1.5), velocity=(2.0, 0.0, 0.0), battery_pct=50.0
    )
    assert action_estop == FailsafeAction.EMERGENCY_STOP
    assert math.hypot(v_estop[0], v_estop[1]) < 2.0


def test_failure_injection_framework_all_11_scenarios(small_world_and_config):
    world, config = small_world_and_config
    framework = FailureInjectionFramework(world, config)
    results = framework.run_all_scenarios()

    assert len(results) == 11
    for r in results:
        assert r.passed is True, f"Scenario {r.scenario_id} failed: {r.details}"
        assert r.response_time_s >= 0.0
        assert len(r.events_log) > 0
