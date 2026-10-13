"""Unit tests for drone-nav CLI commands."""

from pathlib import Path

from typer.testing import CliRunner

from drone_nav.cli import app

runner = CliRunner()


def test_cli_help():
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "Indoor warehouse drone navigation" in result.output


def test_cli_validate_success():
    result = runner.invoke(app, ["validate", "configs/presets/small.yaml"])
    assert result.exit_code == 0
    assert "[OK] Config valid" in result.output
    assert "WH-SMALL" in result.output


def test_cli_validate_failure(tmp_path: Path):
    bad_cfg = tmp_path / "bad.yaml"
    bad_cfg.write_text("site: { id: '' }\n", encoding="utf-8")
    result = runner.invoke(app, ["validate", str(bad_cfg)])
    assert result.exit_code == 1
    assert "[FAIL] Validation failed" in result.output


def test_cli_layout_generate():
    result = runner.invoke(app, ["layout", "generate", "configs/presets/medium.yaml", "--summary"])
    assert result.exit_code == 0
    assert "Regional Distribution" in result.output
    assert "WH-MEDIUM" in result.output
    assert "512" in result.output


def test_cli_layout_generate_json_export(tmp_path: Path):
    out_json = tmp_path / "world.json"
    result = runner.invoke(
        app, ["layout", "generate", "configs/presets/small.yaml", "-o", str(out_json)]
    )
    assert result.exit_code == 0
    assert out_json.exists()


def test_cli_schema_export(tmp_path: Path):
    out_schema = tmp_path / "schema.json"
    result = runner.invoke(app, ["schema", "-o", str(out_schema)])
    assert result.exit_code == 0
    assert out_schema.exists()


def test_cli_csv_export_and_import(tmp_path: Path):
    csv_file = tmp_path / "export.csv"
    res1 = runner.invoke(app, ["layout", "export-csv", "configs/presets/small.yaml", str(csv_file)])
    assert res1.exit_code == 0
    assert csv_file.exists()

    res2 = runner.invoke(app, ["layout", "import-csv", str(csv_file)])
    assert res2.exit_code == 0
    assert "Imported successfully" in res2.output
    assert "90" in res2.output


def test_cli_plan_command(tmp_path: Path):
    plot_path = tmp_path / "plan_plot.svg"
    json_path = tmp_path / "plan.json"
    result = runner.invoke(
        app,
        [
            "plan",
            "configs/presets/small.yaml",
            "--strategy",
            "serpentine",
            "--plot",
            str(plot_path),
            "-o",
            str(json_path),
        ],
    )
    assert result.exit_code == 0
    assert "Mission Plan" in result.output
    assert plot_path.exists()
    assert json_path.exists()


def test_cli_plan_compare():
    result = runner.invoke(app, ["plan", "configs/presets/small.yaml", "--compare"])
    assert result.exit_code == 0
    assert "Scan Strategy Benchmark" in result.output
    assert "SERPENTINE" in result.output
    assert "NAIVE" in result.output


def test_cli_simulate_command(tmp_path):
    telemetry_file = tmp_path / "telemetry.jsonl"
    traj_svg = tmp_path / "traj.svg"
    bat_svg = tmp_path / "bat.svg"

    result = runner.invoke(
        app,
        [
            "simulate",
            "configs/presets/small.yaml",
            "--flight-id",
            "1",
            "--seed",
            "42",
            "-o",
            str(telemetry_file),
            "--plot-path",
            str(traj_svg),
            "--plot-battery",
            str(bat_svg),
        ],
    )
    assert result.exit_code == 0
    assert "Kinematic Flight Simulation" in result.output
    assert "Mean Tracking Error" in result.output
    assert "Reproducibility Hash" in result.output
    assert telemetry_file.exists()
    assert traj_svg.exists()
    assert bat_svg.exists()


def test_cli_simulate_invalid_flight_id():
    result = runner.invoke(
        app,
        ["simulate", "configs/presets/small.yaml", "--flight-id", "99"],
    )
    assert result.exit_code == 1
    assert "out of bounds" in result.output


def test_cli_localise_command(tmp_path):
    p_traj = tmp_path / "loc_traj.svg"
    p_err = tmp_path / "loc_err.svg"
    p_json = tmp_path / "loc_stats.json"

    result = runner.invoke(
        app,
        [
            "localise",
            "configs/presets/small.yaml",
            "--flight-id",
            "1",
            "--plot-traj",
            str(p_traj),
            "--plot-error",
            str(p_err),
            "-o",
            str(p_json),
        ],
    )
    assert result.exit_code == 0
    assert "Localisation EKF Fusion" in result.output
    assert "Mean EKF Position Error" in result.output
    assert p_traj.exists()
    assert p_err.exists()
    assert p_json.exists()


def test_cli_localise_spacing_sweep():
    result = runner.invoke(
        app,
        ["localise", "configs/presets/small.yaml", "--spacing-sweep"],
    )
    assert result.exit_code == 0
    assert "Marker Spacing Benchmark" in result.output
    assert "Every 1 bays" in result.output
    assert "PASS" in result.output


def test_cli_localise_with_dropout():
    result = runner.invoke(
        app,
        [
            "localise",
            "configs/presets/small.yaml",
            "--drop-start",
            "60",
            "--drop-duration",
            "30",
        ],
    )
    assert result.exit_code == 0
    assert "Injected Marker Dropout" in result.output


