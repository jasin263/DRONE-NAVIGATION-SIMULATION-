"""Unit and edge-case tests for mission flight planning."""

from pathlib import Path

from drone_nav.config.loader import load_config
from drone_nav.planner.astar import astar_path
from drone_nav.planner.graph import NavigationGraph
from drone_nav.planner.models import WaypointAction
from drone_nav.planner.planner import MissionPlanner
from drone_nav.planner.plotter import export_mission_path_svg
from drone_nav.world.generator import generate_world_from_config

PRESETS_DIR = Path("configs/presets")


def test_500_bin_medium_plan():
    """Verify 500-bin plan achieves >= 95% coverage, computes flights, path length, and duration."""
    cfg = load_config(PRESETS_DIR / "medium.yaml")
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)

    plan = planner.plan_mission(strategy_name="serpentine")
    assert plan.bins_targeted == 512
    assert plan.bins_scanned == 512
    assert plan.coverage_pct == 100.0
    assert plan.total_flights >= 1
    assert plan.total_distance_m > 1000.0
    assert plan.total_duration_s > 500.0
    assert plan.total_energy_wh > 30.0

    # Every scan waypoint has scan pose and target bin
    scan_wps = [wp for f in plan.flights for wp in f.waypoints if wp.action == WaypointAction.SCAN]
    assert len(scan_wps) == 512
    for wp in scan_wps:
        assert wp.target_bin_id is not None
        assert wp.wait_time_s > 0.0


def test_strategy_comparison_table():
    """Verify comparator produces table rows for all strategies with naive vs optimized contrast."""
    cfg = load_config(PRESETS_DIR / "medium.yaml")
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)

    rows = planner.compare_strategies()
    assert len(rows) == 4
    strategy_names = [r.strategy for r in rows]
    assert "naive" in strategy_names
    assert "serpentine" in strategy_names
    assert "nearest" in strategy_names
    assert "minimum_time" in strategy_names

    naive_row = next(r for r in rows if r.strategy == "naive")
    serp_row = next(r for r in rows if r.strategy == "serpentine")

    # Serpentine should be significantly more compact than naive
    assert serp_row.total_distance_m < naive_row.total_distance_m
    assert serp_row.total_duration_s < naive_row.total_duration_s
    assert serp_row.total_energy_wh < naive_row.total_energy_wh


def test_edge_case_tiny_battery_flight_splitting():
    """Verify battery-aware flight splitting when battery is too small for a single sortie."""
    cfg = load_config(PRESETS_DIR / "small.yaml")
    # Small warehouse has 90 bins. Total energy is ~15-20 Wh.
    # Set capacity to tiny 6.0 Wh (safe reserve 20% -> 4.8 Wh usable)
    cfg.drone.battery.capacity_wh = 6.0
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)

    plan = planner.plan_mission(strategy_name="serpentine")
    assert plan.total_flights > 1, f"Expected multiple flights with tiny battery, got {plan.total_flights}"
    assert plan.bins_scanned == 90
    assert plan.coverage_pct == 100.0

    # Verify each flight sortie ends with RETURN and LAND at pad
    for f in plan.flights:
        assert f.waypoints[-1].action == WaypointAction.LAND
        assert f.estimated_energy_wh <= 6.0 * (1.0 - cfg.drone.battery.safe_reserve_pct / 100.0) + 1.0


def test_edge_case_blocked_aisle_rerouting():
    """Verify that placing an obstacle across an aisle causes A* to reroute."""
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)
    nav_graph = NavigationGraph(world, cfg)

    # In small warehouse, Aisle 1 (A01) has center x=3.5.
    # Check original path exists from South junction to North junction
    p_orig = astar_path(nav_graph.graph, "JUNC-SOUTH-A01", "JUNC-NORTH-A01")
    assert p_orig is not None

    # Now block A01 by removing its internal edges
    blocked_graph = nav_graph.graph.copy()
    edges_to_remove = [
        (u, v) for u, v in blocked_graph.edges if "A01" in u and "A01" in v
    ]
    for u, v in edges_to_remove:
        blocked_graph.remove_edge(u, v)

    # Path directly through A01 is blocked
    p_blocked = astar_path(blocked_graph, "NODE-A01-0", "NODE-A01-4")
    assert p_blocked is None

    # But path between South and North can route via adjacent Aisle A02
    p_reroute = astar_path(blocked_graph, "JUNC-SOUTH-A01", "JUNC-NORTH-A01")
    assert p_reroute is not None
    # Must use A02 nodes
    assert any("A02" in n for n in p_reroute)


def test_edge_case_unreachable_bin_reporting():
    """Verify that an unreachable bin (e.g. inside a blocked zone) is reported without crashing."""
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)

    # Deliberately sever all edges to one specific bin's scan node
    target_bin = next(iter(world.bins.values()))
    scan_node = planner.nav_graph.connect_scan_pose(target_bin)

    # Disconnect scan node completely from graph
    for pred in list(planner.nav_graph.graph.predecessors(scan_node)):
        planner.nav_graph.graph.remove_edge(pred, scan_node)
    for succ in list(planner.nav_graph.graph.successors(scan_node)):
        planner.nav_graph.graph.remove_edge(scan_node, succ)

    plan = planner.plan_mission(strategy_name="serpentine")
    assert target_bin.id in plan.unreachable_bins
    assert plan.bins_scanned == len(world.bins) - 1
    assert plan.coverage_pct < 100.0


def test_edge_case_one_way_aisle_enforcement():
    """Verify that backward travel in a one-way forward aisle is strictly forbidden."""
    cfg = load_config(PRESETS_DIR / "small.yaml")
    # In small.yaml, A02 is one_way: True, direction: forward (South to North)
    world = generate_world_from_config(cfg)
    nav_graph = NavigationGraph(world, cfg)

    # South to North must succeed
    p_forward = astar_path(nav_graph.graph, "JUNC-SOUTH-A02", "JUNC-NORTH-A02")
    assert p_forward is not None

    # North to South in A02 must NOT be directly through A02
    # If we isolate A02 (remove arterials), North to South must have no path
    isolated = nav_graph.graph.copy()
    # Remove cross-arterial connections
    for u, v in list(isolated.edges):
        if "JUNC" in u and "JUNC" in v:
            isolated.remove_edge(u, v)

    p_reverse = astar_path(isolated, "JUNC-NORTH-A02", "JUNC-SOUTH-A02")
    assert p_reverse is None, "Backward travel through a one-way forward aisle must be blocked!"


def test_plot_svg_generation(tmp_path: Path):
    """Verify flight path SVG plot export."""
    cfg = load_config(PRESETS_DIR / "small.yaml")
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)
    plan = planner.plan_mission(strategy_name="serpentine")

    svg_file = tmp_path / "test_path.svg"
    out = export_mission_path_svg(world, plan, svg_file)
    assert out.exists()
    content = out.read_text(encoding="utf-8")
    assert "<svg" in content
    assert "<polyline" in content
    assert "Mission Plan" in content
