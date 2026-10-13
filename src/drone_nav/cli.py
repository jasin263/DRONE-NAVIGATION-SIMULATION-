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
