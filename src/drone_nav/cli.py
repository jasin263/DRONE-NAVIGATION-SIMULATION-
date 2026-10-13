"""Command-line interface for drone-nav."""

import json
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from drone_nav.config.loader import (
    ConfigError,
    load_config,
    save_json_schema,
)
from drone_nav.world.csv_importer import CSVImportError, export_drims_csv, import_drims_csv
from drone_nav.world.generator import generate_world_from_config

app = typer.Typer(
    name="drone-nav",
    help="Indoor warehouse drone navigation and flight simulation system",
    add_completion=False,
)
layout_app = typer.Typer(name="layout", help="Warehouse layout generation and conversion tools")
app.add_typer(layout_app)

console = Console()


@app.command()
def validate(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML or JSON configuration file")],
) -> None:
    """Validate a warehouse configuration against the Pydantic v2 schema."""
    try:
        config = load_config(config_path)
        console.print(f"[bold green][OK] Config valid:[/bold green] {config_path}")
        console.print(
            f"  Site ID: [cyan]{config.site.id}[/cyan] | "
            f"Name: [white]{config.site.name}[/white] | "
            f"Aisles: [yellow]{len(config.layout.aisles)}[/yellow]"
        )
    except ConfigError as exc:
        console.print(f"[bold red][FAIL] Validation failed for {config_path}:[/bold red]")
        console.print(str(exc))
        raise typer.Exit(code=1) from exc


@app.command()
def schema(
    output: Annotated[
        Path | None, typer.Option("--output", "-o", help="File to write JSON Schema to")
    ] = None,
) -> None:
    """Export the official JSON Schema for warehouse configurations."""
    out_path = output or Path("configs/schema/warehouse_config.schema.json")
    saved = save_json_schema(out_path)
    console.print(f"[bold green][OK] JSON Schema exported to:[/bold green] {saved}")


@layout_app.command("generate")
def layout_generate(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML or JSON configuration file")],
    summary: Annotated[
        bool, typer.Option("--summary", "-s", help="Print warehouse layout summary table")
    ] = True,
    output: Annotated[
        Path | None, typer.Option("--output", "-o", help="Save serialized world to JSON")
    ] = None,
) -> None:
    """Generate 3D warehouse world from a configuration file."""
    try:
        config = load_config(config_path)
        world = generate_world_from_config(config)
        s = world.get_summary()

        if summary:
            table = Table(title=f"Warehouse Layout: {s.site_name} ({s.site_id})")
            table.add_column("Metric", style="cyan", no_wrap=True)
            table.add_column("Value", style="magenta")

            table.add_row(
                "Floor Dimensions (X x Y)",
                f"{s.floor_dimensions_m[0]:.1f}m x {s.floor_dimensions_m[1]:.1f}m",
            )
            table.add_row("Ceiling Height", f"{s.ceiling_height_m:.1f}m")
            table.add_row("Flight Aisles", str(s.total_aisles))
            table.add_row("Racks", str(s.total_racks))
            table.add_row("Total Storage Bays", str(s.total_bays))
            table.add_row("Total Storage Bins", f"[bold green]{s.total_bins:,}[/bold green]")
            table.add_row("Fiducial Markers", str(s.total_markers))
            table.add_row("Takeoff/Landing Pads", str(s.total_pads))
            table.add_row("Charging Stations", str(s.total_charging_stations))
            table.add_row("No-Fly Zones", str(s.total_no_fly_zones))
            table.add_row("People Zones", str(s.total_people_zones))

            console.print(table)

        if output:
            from drone_nav.viewer.app import serialize_world

            output.parent.mkdir(parents=True, exist_ok=True)
            data = serialize_world(world)
            with open(output, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            console.print(f"[green][OK] Serialized world saved to:[/green] {output}")

    except Exception as exc:
        console.print(f"[bold red]Error generating layout:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@layout_app.command("import-csv")
def layout_import_csv(
    csv_path: Annotated[Path, typer.Argument(help="Path to DRIMS location CSV file")],
    export_csv: Annotated[
        Path | None,
        typer.Option("--export-csv", help="Re-export to another CSV to test round-trip"),
    ] = None,
) -> None:
    """Import warehouse bins from a DRIMS location CSV file."""
    try:
        world = import_drims_csv(csv_path)
        s = world.get_summary()
        console.print(
            Panel(
                f"[bold green]Imported successfully from {csv_path}[/bold green]\n"
                f"* Site: {s.site_id}\n"
                f"* Bins imported: [bold cyan]{s.total_bins}[/bold cyan]\n"
                f"* Aisles inferred: {s.total_aisles}\n"
                f"* Racks inferred: {s.total_racks}\n"
                f"* Floor bounds: {s.floor_dimensions_m[0]:.1f}m x {s.floor_dimensions_m[1]:.1f}m",
                title="DRIMS CSV Ingestion",
            )
        )
        if export_csv:
            export_drims_csv(world, export_csv)
            console.print(f"[green][OK] Re-exported to:[/green] {export_csv}")
    except CSVImportError as exc:
        console.print(f"[bold red]CSV Import Error:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@layout_app.command("export-csv")
def layout_export_csv(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML configuration file")],
    output_csv: Annotated[Path, typer.Argument(help="Output CSV destination path")],
) -> None:
    """Export warehouse bin locations to DRIMS CSV format."""
    try:
        config = load_config(config_path)
        world = generate_world_from_config(config)
        out = export_drims_csv(world, output_csv)
        console.print(
            f"[bold green][OK] Exported {len(world.bins)} bins to DRIMS CSV:[/bold green] {out}"
        )
    except Exception as exc:
        console.print(f"[bold red]Error exporting CSV:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@app.command()
def plan(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML configuration file")],
    strategy: Annotated[
        str,
        typer.Option(
            "--strategy",
            "-s",
            help="Scan sequencing strategy (serpentine, naive, nearest, minimum_time)",
        ),
    ] = "serpentine",
    compare: Annotated[
        bool, typer.Option("--compare", help="Compare all scan strategies side-by-side")
    ] = False,
    plot: Annotated[
        Path | None, typer.Option("--plot", help="Path to save SVG flight path plot")
    ] = None,
    output_json: Annotated[
        Path | None,
        typer.Option("--output-json", "-o", help="Save mission plan to JSON file"),
    ] = None,
) -> None:
    """Plan collision-free warehouse drone scanning flight missions."""
    try:
        from drone_nav.planner.planner import MissionPlanner
        from drone_nav.planner.plotter import export_mission_path_svg

        config = load_config(config_path)
        world = generate_world_from_config(config)
        planner = MissionPlanner(world, config)

        if compare:
            rows = planner.compare_strategies()
            table = Table(title=f"Scan Strategy Benchmark: {world.site_name} ({len(world.bins)} Bins)")
            table.add_column("Strategy", style="cyan", no_wrap=True)
            table.add_column("Coverage", style="green")
            table.add_column("Flights", style="yellow")
            table.add_column("Path Length", style="magenta")
            table.add_column("Duration", style="blue")
            table.add_column("Energy (Wh)", style="red")
            table.add_column("Dist/Bin", style="white")

            for r in rows:
                dur_min = r.total_duration_s / 60.0
                table.add_row(
                    r.strategy.upper(),
                    f"{r.coverage_pct}%",
                    str(r.total_flights),
                    f"{r.total_distance_m:.1f}m",
                    f"{dur_min:.1f} min ({r.total_duration_s:.0f}s)",
                    f"{r.total_energy_wh:.1f} Wh",
                    f"{r.avg_dist_per_bin_m:.2f}m",
                )
            console.print(table)
            return

        mission = planner.plan_mission(strategy_name=strategy)

        table = Table(title=f"Mission Plan: {mission.mission_id} [{strategy.upper()}]")
        table.add_column("Parameter", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        dur_min = mission.total_duration_s / 60.0
        table.add_row("Site ID", mission.site_id)
        table.add_row("Strategy", mission.strategy.upper())
        table.add_row("Target Bins", str(mission.bins_targeted))
        table.add_row("Scanned Bins", str(mission.bins_scanned))
        table.add_row("Coverage", f"[bold green]{mission.coverage_pct}%[/bold green]")
        table.add_row("Total Flights (Sorties)", str(mission.total_flights))
        table.add_row("Total Path Distance", f"{mission.total_distance_m:.1f}m")
        table.add_row("Estimated Duration", f"{dur_min:.1f} min ({mission.total_duration_s:.0f}s)")
        table.add_row("Total Battery Consumed", f"{mission.total_energy_wh:.1f} Wh")
        if mission.unreachable_bins:
            table.add_row(
                "Unreachable Bins", f"[bold red]{len(mission.unreachable_bins)}[/bold red]"
            )

        console.print(table)

        if plot:
            svg_path = export_mission_path_svg(world, mission, plot)
            console.print(f"[bold green][OK] Flight path SVG plotted to:[/bold green] {svg_path}")

        if output_json:
            output_json.parent.mkdir(parents=True, exist_ok=True)
            output_json.write_text(json.dumps(mission.to_dict(), indent=2), encoding="utf-8")
            console.print(
                f"[bold green][OK] Mission plan JSON saved to:[/bold green] {output_json}"
            )

    except Exception as exc:
        console.print(f"[bold red]Planning Error:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@app.command()
def simulate(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML configuration file")],
    strategy: Annotated[
        str,
        typer.Option(
            "--strategy",
            "-s",
            help="Scan sequencing strategy (serpentine, naive, nearest, minimum_time)",
        ),
    ] = "serpentine",
    flight_id: Annotated[
        int,
        typer.Option("--flight-id", "-f", help="Flight sortie ID to simulate (1-indexed)"),
    ] = 1,
    seed: Annotated[
        int, typer.Option("--seed", help="Deterministic random seed for wind & noise")
    ] = 42,
    wind_speed: Annotated[
        float, typer.Option("--wind-speed", help="Turbulence gust intensity (m/s)")
    ] = 0.2,
    dt: Annotated[
        float, typer.Option("--dt", help="Simulation time step in seconds")
    ] = 0.05,
    output_telemetry: Annotated[
        Path | None,
        typer.Option("--output-telemetry", "-o", help="Path to export JSONL telemetry file"),
    ] = None,
    plot_path: Annotated[
        Path | None,
        typer.Option("--plot-path", help="Path to export planned vs flown trajectory SVG"),
    ] = None,
    plot_battery: Annotated[
        Path | None,
        typer.Option("--plot-battery", help="Path to export battery discharge curve SVG"),
    ] = None,
) -> None:
    """Execute kinematic flight simulation with tracking controller, wind, and battery drain."""
    try:
        from drone_nav.planner.planner import MissionPlanner
        from drone_nav.simulator.plots import (
            export_battery_curve_svg,
            export_flight_trajectory_comparison_svg,
        )
        from drone_nav.simulator.sim import FlightSimulator

        config = load_config(config_path)
        world = generate_world_from_config(config)
        planner = MissionPlanner(world, config)
        mission = planner.plan_mission(strategy_name=strategy)

        if not mission.flights:
            console.print("[bold red]No flights generated in mission plan.[/bold red]")
            raise typer.Exit(code=1)

        flight_idx = flight_id - 1
        if flight_idx < 0 or flight_idx >= len(mission.flights):
            console.print(
                f"[bold red]Flight ID {flight_id} out of bounds (total flights: {len(mission.flights)}).[/bold red]"
            )
            raise typer.Exit(code=1)

        flight = mission.flights[flight_idx]

        sim = FlightSimulator(
            world=world, config=config, dt=dt, seed=seed, wind_speed=wind_speed
        )
        sim_res = sim.simulate_flight(flight, mission_id=mission.mission_id)

        table = Table(
            title=f"Kinematic Flight Simulation: Flight {flight.flight_id}/{mission.total_flights}"
        )
        table.add_column("Metric", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        dur_min = sim_res.total_time_s / 60.0
        table.add_row("Mission ID", sim_res.mission_id)
        table.add_row("Flight Sortie", f"{sim_res.flight_id} of {mission.total_flights}")
        table.add_row("Planned Distance", f"{flight.distance_m:.2f} m")
        table.add_row("Flown Distance", f"{sim_res.total_distance_flown_m:.2f} m")
        table.add_row(
            "Flight Time", f"{dur_min:.2f} min ({sim_res.total_time_s:.1f} s)"
        )
        table.add_row("Samples Recorded", str(sim_res.sample_count))
        table.add_row(
            "Mean Tracking Error", f"[bold green]{sim_res.mean_tracking_error_m * 100:.2f} cm[/bold green]"
        )
        table.add_row("P95 Tracking Error", f"{sim_res.p95_tracking_error_m * 100:.2f} cm")
        table.add_row("Max Tracking Error", f"{sim_res.max_tracking_error_m * 100:.2f} cm")
        table.add_row("Initial Battery", f"{sim_res.initial_battery_wh:.2f} Wh (100.0%)")
        table.add_row(
            "Final Battery",
            f"{sim_res.final_battery_wh:.2f} Wh ({sim_res.final_battery_pct:.1f}%)",
        )
        table.add_row("Energy Consumed", f"{sim_res.energy_consumed_wh:.2f} Wh")
        table.add_row("Reproducibility Hash", f"[bold yellow]{sim_res.reproducibility_hash[:16]}...[/bold yellow]")

        console.print(table)

        if output_telemetry:
            output_telemetry.parent.mkdir(parents=True, exist_ok=True)
            lines = [json.dumps(s.to_telemetry_dict()) for s in sim_res.states]
            output_telemetry.write_text("\n".join(lines), encoding="utf-8")
            console.print(
                f"[bold green][OK] Telemetry exported ({len(lines)} records):[/bold green] {output_telemetry}"
            )

        if plot_path:
            p = export_flight_trajectory_comparison_svg(world, flight, sim_res, plot_path)
            console.print(
                f"[bold green][OK] Trajectory comparison SVG saved:[/bold green] {p}"
            )

        if plot_battery:
            b = export_battery_curve_svg(
                sim_res,
                plot_battery,
                safe_reserve_pct=config.drone.battery.safe_reserve_pct,
                critical_reserve_pct=config.drone.battery.critical_reserve_pct,
            )
            console.print(
                f"[bold green][OK] Battery discharge curve SVG saved:[/bold green] {b}"
            )

    except Exception as exc:
        console.print(f"[bold red]Simulation Error:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@app.command()
def localise(
    config_path: Annotated[Path, typer.Argument(help="Path to YAML configuration file")],
    strategy: Annotated[
        str,
        typer.Option(
            "--strategy",
            "-s",
            help="Scan sequencing strategy (serpentine, naive, nearest, minimum_time)",
        ),
    ] = "serpentine",
    flight_id: Annotated[
        int,
        typer.Option("--flight-id", "-f", help="Flight sortie ID to evaluate (1-indexed)"),
    ] = 1,
    spacing_sweep: Annotated[
        bool,
        typer.Option(
            "--spacing-sweep",
            help="Sweep fiducial marker bay spacing (N=1, 2, 4, 8) and compare accuracy",
        ),
    ] = False,
    drop_start: Annotated[
        float | None,
        typer.Option("--drop-start", help="Start time in seconds to inject marker loss gap"),
    ] = None,
    drop_duration: Annotated[
        float,
        typer.Option("--drop-duration", help="Duration in seconds of injected marker loss gap"),
    ] = 60.0,
    seed: Annotated[
        int, typer.Option("--seed", help="Deterministic random seed for sensor noise")
    ] = 42,
    plot_traj: Annotated[
        Path | None,
        typer.Option("--plot-traj", help="Path to export Ground Truth vs EKF trajectory SVG"),
    ] = None,
    plot_error: Annotated[
        Path | None,
        typer.Option("--plot-error", help="Path to export EKF error and 3-sigma envelope SVG"),
    ] = None,
    output_json: Annotated[
        Path | None,
        typer.Option("--output-json", "-o", help="Path to export evaluation metrics JSON"),
    ] = None,
) -> None:
    """Evaluate EKF sensor fusion of IMU dead-reckoning with visual fiducial marker fixes."""
    try:
        from drone_nav.localisation.estimator import (
            LocalisationEstimator,
            run_marker_spacing_sweep,
        )
        from drone_nav.localisation.plots import (
            export_localisation_error_svg,
            export_localisation_trajectory_svg,
        )
        from drone_nav.planner.planner import MissionPlanner
        from drone_nav.simulator.sim import FlightSimulator

        config = load_config(config_path)
        world = generate_world_from_config(config)
        planner = MissionPlanner(world, config)
        mission = planner.plan_mission(strategy_name=strategy)

        if not mission.flights:
            console.print("[bold red]No flights found in mission plan.[/bold red]")
            raise typer.Exit(code=1)

        flight_idx = flight_id - 1
        if flight_idx < 0 or flight_idx >= len(mission.flights):
            console.print(
                f"[bold red]Flight ID {flight_id} out of bounds (total flights: {len(mission.flights)}).[/bold red]"
            )
            raise typer.Exit(code=1)

        flight = mission.flights[flight_idx]

        if spacing_sweep:
            sweep_results = run_marker_spacing_sweep(
                world_base=world, config=config, flight_plan=flight, seed=seed
            )
            table = Table(title=f"Marker Spacing Benchmark: {world.site_name} (Sweep N=1, 2, 4, 8 Bays)")
            table.add_column("Spacing (Bays)", style="cyan", no_wrap=True)
            table.add_column("Markers in Map", style="yellow")
            table.add_column("Fixes Count", style="blue")
            table.add_column("Mean Error", style="green")
            table.add_column("P95 Error", style="magenta")
            table.add_column("Max Error", style="white")
            table.add_column("Max DR Drift", style="red")
            table.add_column("Status", style="bold")

            for r in sweep_results:
                status_fmt = (
                    f"[bold green]{r.status}[/bold green]"
                    if r.status == "PASS"
                    else f"[bold yellow]{r.status}[/bold yellow]"
                )
                table.add_row(
                    f"Every {r.spacing_bays} bays",
                    str(r.total_markers),
                    str(r.marker_detections),
                    f"{r.mean_error_m * 100:.2f} cm",
                    f"{r.p95_error_m * 100:.2f} cm",
                    f"{r.max_error_m * 100:.2f} cm",
                    f"{r.max_drift_m:.1f} m",
                    status_fmt,
                )
            console.print(table)
            return

        # Pre-simulate flight
        sim = FlightSimulator(world=world, config=config, dt=0.05, seed=seed)
        sim_res = sim.simulate_flight(flight, mission_id=mission.mission_id)

        # Injected marker dropout gap if requested
        drop_interval = (
            (drop_start, drop_start + drop_duration) if drop_start is not None else None
        )

        estimator = LocalisationEstimator(world=world, config=config, seed=seed)
        loc_res = estimator.run_localisation(sim_res, marker_drop_interval=drop_interval)

        table = Table(
            title=f"Localisation EKF Fusion: Flight {flight.flight_id}/{mission.total_flights}"
        )
        table.add_column("Metric", style="cyan", no_wrap=True)
        table.add_column("Value", style="magenta")

        dur_min = loc_res.total_time_s / 60.0
        table.add_row("Mission ID", mission.mission_id)
        table.add_row("Flight Duration", f"{dur_min:.2f} min ({loc_res.total_time_s:.1f} s)")
        table.add_row("Total Markers in Warehouse", str(loc_res.total_markers_in_map))
        table.add_row("Visual Marker Detections", str(loc_res.total_marker_detections))
        table.add_row(
            "Mean EKF Position Error",
            f"[bold green]{loc_res.mean_ekf_error_m * 100:.2f} cm[/bold green]",
        )
        table.add_row(
            "P95 EKF Position Error", f"{loc_res.p95_ekf_error_m * 100:.2f} cm"
        )
        table.add_row("Max EKF Position Error", f"{loc_res.max_ekf_error_m * 100:.2f} cm")
        table.add_row(
            "Raw Dead-Reckoning Max Drift",
            f"[bold red]{loc_res.max_dr_error_m:.2f} m[/bold red] (unconstrained)",
        )
        table.add_row(
            "Expected DRIMS Telemetry Error",
            f"[bold green]+/- {loc_res.expected_drims_error_m * 100:.2f} cm (P95)[/bold green]",
        )

        if drop_interval is not None:
            table.add_row(
                "Injected Marker Dropout",
                f"[bold red]{drop_interval[0]:.1f}s - {drop_interval[1]:.1f}s ({drop_duration:.1f}s gap)[/bold red]",
            )

        console.print(table)

        if plot_traj:
            p = export_localisation_trajectory_svg(world, loc_res, plot_traj)
            console.print(
                f"[bold green][OK] Localisation trajectory SVG saved:[/bold green] {p}"
            )

        if plot_error:
            e = export_localisation_error_svg(loc_res, plot_error)
            console.print(
                f"[bold green][OK] Position error envelope SVG saved:[/bold green] {e}"
            )

        if output_json:
            output_json.parent.mkdir(parents=True, exist_ok=True)
            summary_data = {
                "mission_id": mission.mission_id,
                "flight_id": flight.flight_id,
                "markers_in_map": loc_res.total_markers_in_map,
                "marker_detections": loc_res.total_marker_detections,
                "mean_ekf_error_m": loc_res.mean_ekf_error_m,
                "p95_ekf_error_m": loc_res.p95_ekf_error_m,
                "max_ekf_error_m": loc_res.max_ekf_error_m,
                "mean_dr_error_m": loc_res.mean_dr_error_m,
                "max_dr_error_m": loc_res.max_dr_error_m,
                "expected_drims_error_m": loc_res.expected_drims_error_m,
            }
            output_json.write_text(json.dumps(summary_data, indent=2), encoding="utf-8")
            console.print(
                f"[bold green][OK] Localisation metrics JSON saved:[/bold green] {output_json}"
            )

    except Exception as exc:
        console.print(f"[bold red]Localisation Error:[/bold red] {exc}")
        raise typer.Exit(code=1) from exc


@app.command()
def viewer(
    config: Annotated[
        str, typer.Option("--config", "-c", help="Preset name or path to config file")
    ] = "medium",
    host: Annotated[str, typer.Option("--host", "-h", help="Bind host address")] = "127.0.0.1",
    port: Annotated[int, typer.Option("--port", "-p", help="Bind port")] = 8000,
) -> None:
    """Launch the interactive web viewer server."""
    import uvicorn

    console.print(f"[bold green]Starting drone-nav viewer at http://{host}:{port}[/bold green]")
    uvicorn.run("drone_nav.viewer.app:app", host=host, port=port, reload=False)


if __name__ == "__main__":
    app()
