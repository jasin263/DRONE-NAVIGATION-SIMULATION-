"""FastAPI application for drone-nav warehouse viewer."""

from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, HTTPException, Query, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

from drone_nav.config.loader import ConfigError, get_json_schema, load_config
from drone_nav.world.csv_importer import CSVImportError, import_drims_csv
from drone_nav.world.generator import generate_world_from_config
from drone_nav.world.models import World

app = FastAPI(
    title="drone-nav Warehouse Simulator & Viewer",
    description="Interactive viewer and API for drone warehouse navigation simulation",
    version="0.1.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

CONFIGS_DIR = Path("configs")
PRESETS_DIR = CONFIGS_DIR / "presets"


def serialize_world(world: World) -> dict[str, Any]:
    """Serialize World object into JSON-friendly format for web visualization."""
    return {
        "site_id": world.site_id,
        "site_name": world.site_name,
        "floor_bounds": {
            "x_min": world.floor_bounds[0],
            "x_max": world.floor_bounds[1],
            "y_min": world.floor_bounds[2],
            "y_max": world.floor_bounds[3],
        },
        "ceiling_height": world.ceiling_height,
        "summary": {
            "total_aisles": len(world.aisles),
            "total_racks": len(world.racks),
            "total_bins": len(world.bins),
            "total_markers": len(world.markers),
            "total_pads": len(world.pads),
            "total_charging_stations": len(world.charging_stations),
            "total_no_fly_zones": len(world.no_fly_zones),
            "total_people_zones": len(world.people_zones),
        },
        "aisles": [
            {
                "id": a.id,
                "name": a.name,
                "x_start": a.x_start,
                "y_start": a.y_start,
                "x_end": a.x_end,
                "y_end": a.y_end,
                "width": a.width,
                "one_way": a.one_way,
                "direction": a.direction,
                "bounds": {
                    "x_min": a.corridor_bounds.x_min,
                    "x_max": a.corridor_bounds.x_max,
                    "y_min": a.corridor_bounds.y_min,
                    "y_max": a.corridor_bounds.y_max,
                },
            }
            for a in world.aisles
        ],
        "racks": [
            {
                "id": r.id,
                "aisle_id": r.aisle_id,
                "side": r.side,
                "bays": r.bays,
                "levels": r.levels,
                "bounds": {
                    "x_min": r.bounds.x_min,
                    "x_max": r.bounds.x_max,
                    "y_min": r.bounds.y_min,
                    "y_max": r.bounds.y_max,
                    "z_min": r.bounds.z_min,
                    "z_max": r.bounds.z_max,
                },
                "bin_count": len(r.bins),
            }
            for r in world.racks
        ],
        "bins": [
            {
                "id": b.id,
                "label_code": b.label_code,
                "aisle_id": b.aisle_id,
                "rack_id": b.rack_id,
                "bay_index": b.bay_index,
                "level_index": b.level_index,
                "bounds": {
                    "x_min": b.bounds.x_min,
                    "x_max": b.bounds.x_max,
                    "y_min": b.bounds.y_min,
                    "y_max": b.bounds.y_max,
                    "z_min": b.bounds.z_min,
                    "z_max": b.bounds.z_max,
                },
                "center": list(b.center),
                "scan_normal": list(b.scan_face_normal),
                "recommended_scan_pos": list(b.recommended_scan_pos),
            }
            for b in world.bins.values()
        ],
        "markers": [
            {
                "id": m.id,
                "tag_id": m.tag_id,
                "position": list(m.position),
                "normal": list(m.normal),
                "size_m": m.size_m,
                "rack_id": m.rack_id,
                "bay_index": m.bay_index,
            }
            for m in world.markers
        ],
        "pads": [
            {
                "id": p.id,
                "position": list(p.position),
                "radius": p.radius,
            }
            for p in world.pads
        ],
        "charging_stations": [
            {
                "id": cs.id,
                "position": list(cs.position),
                "charge_rate_kw": cs.charge_rate_kw,
            }
            for cs in world.charging_stations
        ],
        "doors": [
            {
                "id": d.id,
                "position": list(d.position),
                "width": d.width,
                "height": d.height,
            }
            for d in world.doors
        ],
        "no_fly_zones": [
            {
                "id": nfz.id,
                "name": nfz.name,
                "polygon": [list(pt) for pt in nfz.polygon],
                "z_min": nfz.z_min,
                "z_max": nfz.z_max,
            }
            for nfz in world.no_fly_zones
        ],
        "people_zones": [
            {
                "id": pz.id,
                "name": pz.name,
                "polygon": [list(pt) for pt in pz.polygon],
                "max_speed_mps": pz.max_speed_mps,
            }
            for pz in world.people_zones
        ],
        "static_obstacles": [
            {
                "id": obs.id,
                "type": obs.type,
                "bounds": {
                    "x_min": obs.bounds.x_min,
                    "x_max": obs.bounds.x_max,
                    "y_min": obs.bounds.y_min,
                    "y_max": obs.bounds.y_max,
                    "z_min": obs.bounds.z_min,
                    "z_max": obs.bounds.z_max,
                },
            }
            for obs in world.static_obstacles
        ],
        "dynamic_obstacles": [
            {
                "id": dyn.id,
                "type": dyn.type,
                "path": [list(pt) for pt in dyn.path],
                "speed_mps": dyn.speed_mps,
                "schedule_start_s": dyn.schedule_start_s,
                "schedule_end_s": dyn.schedule_end_s,
            }
            for dyn in world.dynamic_obstacles
        ],
    }


@app.get("/api/presets")
def list_presets() -> list[dict[str, Any]]:
    """List available presets."""
    presets = []
    if PRESETS_DIR.exists():
        for f in PRESETS_DIR.glob("*.yaml"):
            try:
                cfg = load_config(f)
                presets.append(
                    {
                        "id": f.stem,
                        "filename": f.name,
                        "name": cfg.site.name,
                        "site_id": cfg.site.id,
                    }
                )
            except Exception:
                presets.append(
                    {"id": f.stem, "filename": f.name, "name": f.stem, "site_id": "UNKNOWN"}
                )
    return presets


@app.get("/api/schema")
def get_schema() -> dict[str, Any]:
    """Return JSON Schema for WarehouseConfig."""
    return get_json_schema()


@app.get("/api/world")
def get_world(config: str = Query(default="medium")) -> dict[str, Any]:
    """Generate and return world geometry for a given preset name or config path."""
    # Check if preset name or direct file
    candidates = [
        Path(config),
        PRESETS_DIR / f"{config}.yaml",
        PRESETS_DIR / f"{config}.yml",
    ]
    target_path = None
    for p in candidates:
        if p.exists():
            target_path = p
            break

    if target_path is None:
        raise HTTPException(
            status_code=404,
            detail=f"Configuration not found. Checked: {[str(c) for c in candidates]}",
        )

    try:
        cfg = load_config(target_path)
        world = generate_world_from_config(cfg)
        return serialize_world(world)
    except ConfigError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Internal error generating world: {exc}")


@app.post("/api/import-csv")
async def api_import_csv(file: UploadFile = File(...)) -> dict[str, Any]:
    """Import uploaded DRIMS CSV and return world layout."""
    import tempfile

    suffix = Path(file.filename or "uploaded.csv").suffix
    with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
        content = await file.read()
        tmp.write(content)
        tmp_path = Path(tmp.name)

    try:
        world = import_drims_csv(tmp_path)
        return serialize_world(world)
    except CSVImportError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


@app.get("/", response_class=HTMLResponse)
def index_page() -> str:
    """Serve top-down interactive viewer application."""
    html_path = Path(__file__).parent / "templates" / "index.html"
    if html_path.exists():
        return html_path.read_text(encoding="utf-8")
    return "<h1>Viewer UI template not found</h1>"
