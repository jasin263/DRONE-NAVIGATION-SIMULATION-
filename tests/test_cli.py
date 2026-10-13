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

