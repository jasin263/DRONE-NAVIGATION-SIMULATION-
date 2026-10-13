"""Unit tests for ScenarioRunner parameter sweeps and SVG plotter."""

import tempfile
from pathlib import Path

from drone_nav.config.loader import load_config
from drone_nav.scenarios.models import SweepIterationResult, SweepReport
from drone_nav.scenarios.plotter import export_sweep_plot_svg
from drone_nav.scenarios.runner import ScenarioRunner
from drone_nav.world.generator import generate_world_from_config


def test_scenario_runner_wind_sweep():
    config = load_config(Path("configs/presets/small.yaml"))
    world = generate_world_from_config(config)
    runner = ScenarioRunner(world, config, seed=42)

    report = runner.run_wind_sweep(wind_speeds=[0.0, 1.0])
    assert report.sweep_name == "Wind Disturbance Sweep"
    assert report.parameter_swept == "wind_speed_mps"
    assert len(report.results) == 2

    # Higher wind should increase mean tracking error or flight energy
    r1 = report.results[0]
    r2 = report.results[1]
    assert r1.distance_m > 0
    assert r2.distance_m > 0
    assert r2.mean_tracking_error_cm >= r1.mean_tracking_error_cm or r2.energy_consumed_wh >= r1.energy_consumed_wh


def test_scenario_runner_speed_sweep():
    config = load_config(Path("configs/presets/small.yaml"))
    world = generate_world_from_config(config)
    runner = ScenarioRunner(world, config, seed=42)

    report = runner.run_speed_sweep(max_speeds=[1.5, 3.0])
    assert report.sweep_name == "Cruise Flight Speed Sweep"
    assert report.parameter_swept == "max_speed_mps"
    assert len(report.results) == 2
    # Higher speed should finish in less or equal flight time
    assert report.results[1].flight_time_s <= report.results[0].flight_time_s


def test_scenario_runner_strategy_sweep():
    config = load_config(Path("configs/presets/small.yaml"))
    world = generate_world_from_config(config)
    runner = ScenarioRunner(world, config, seed=42)

    report = runner.run_strategy_sweep(strategies=["serpentine", "nearest"])
    assert report.sweep_name == "Waypoints Sequencing Strategy Sweep"
    assert report.parameter_swept == "strategy"
    assert len(report.results) == 2
    assert report.results[0].bins_scanned == 90
    assert report.results[1].bins_scanned == 90


def test_export_sweep_plot_svg():
    report = SweepReport(
        sweep_name="Unit Test Sweep",
        parameter_swept="param_x",
        values_tested=[1.0, 2.0],
        results=[
            SweepIterationResult(
                iteration=1,
                params={"param_x": 1.0},
                distance_m=100.0,
                flight_time_s=60.0,
                energy_consumed_wh=5.0,
                mean_tracking_error_cm=12.5,
                p95_tracking_error_cm=25.0,
                bins_scanned=90,
                read_success_rate=0.9,
                coverage_pct=100.0,
            ),
            SweepIterationResult(
                iteration=2,
                params={"param_x": 2.0},
                distance_m=100.0,
                flight_time_s=55.0,
                energy_consumed_wh=6.5,
                mean_tracking_error_cm=15.0,
                p95_tracking_error_cm=30.0,
                bins_scanned=90,
                read_success_rate=0.9,
                coverage_pct=100.0,
            ),
        ],
    )

    with tempfile.TemporaryDirectory() as tmp_dir:
        svg_file = Path(tmp_dir) / "test_plot.svg"
        saved = export_sweep_plot_svg(report, svg_file)
        assert saved.exists()
        content = saved.read_text(encoding="utf-8")
        assert "<svg" in content
        assert "Unit Test Sweep" in content
        assert "Mean Tracking Error" in content
