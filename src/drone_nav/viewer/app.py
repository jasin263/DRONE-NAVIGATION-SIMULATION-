"""FastAPI application for drone-nav warehouse viewer, mission execution, and simulation API."""

import io
import json
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import yaml
from fastapi import Body, FastAPI, File, HTTPException, Query, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles

from drone_nav.config.loader import ConfigError, get_json_schema, load_config
from drone_nav.config.models import WarehouseConfig
from drone_nav.export.exporter import DRIMSExporter
from drone_nav.localisation.estimator import LocalisationEstimator
from drone_nav.planner.planner import MissionPlanner
from drone_nav.safety.injector import FailureInjectionFramework
from drone_nav.scanning.service import ScanningMissionSimulator
from drone_nav.scenarios.plotter import export_sweep_plot_svg
from drone_nav.scenarios.runner import ScenarioRunner
from drone_nav.world.csv_importer import CSVImportError, import_drims_csv
from drone_nav.world.generator import generate_world_from_config
from drone_nav.world.models import World

app = FastAPI(
    title="drone-nav Warehouse Simulator & Viewer",
    description="Interactive viewer, 3D simulation replay, and REST API for indoor warehouse drones",
    version="1.0.0",
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

# Mount Amazon Bin Image Dataset statically if present
amazon_img_path = Path("Amazonbindata/img")
if amazon_img_path.exists() and amazon_img_path.is_dir():
    app.mount("/img/amazon", StaticFiles(directory=str(amazon_img_path)), name="amazon_images")

# In-memory mission runs cache
MISSION_RUNS: dict[str, dict[str, Any]] = {}


def _resolve_config_path(config_param: str) -> Path:
    candidates = [
        Path(config_param),
        PRESETS_DIR / f"{config_param}.yaml",
        PRESETS_DIR / f"{config_param}.yml",
    ]
    for p in candidates:
        if p.exists():
            return p
    raise HTTPException(
        status_code=404,
        detail=f"Configuration '{config_param}' not found. Checked: {[str(c) for c in candidates]}",
    )


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
                "shelf_slot": b.shelf_slot,
                "center": list(b.center),
                "scan_pos": list(b.recommended_scan_pos),
                "normal": list(b.scan_face_normal),
                "bounds": {
                    "x_min": b.bounds.x_min,
                    "x_max": b.bounds.x_max,
                    "y_min": b.bounds.y_min,
                    "y_max": b.bounds.y_max,
                    "z_min": b.bounds.z_min,
                    "z_max": b.bounds.z_max,
                },
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
            }
            for m in world.markers
        ],
        "pads": [
            {"id": p.id, "position": list(p.position), "radius": p.radius}
            for p in world.pads
        ],
        "charging_stations": [
            {"id": c.id, "position": list(c.position), "charge_rate_kw": c.charge_rate_kw}
            for c in world.charging_stations
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
    }


# ============================================================================
# Core API Routes
# ============================================================================


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
    target_path = _resolve_config_path(config)
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


# ============================================================================
# Phase 7: Extended API Endpoints
# ============================================================================


@app.get("/api/v1/config")
def api_get_config(config: str = Query(default="medium")) -> dict[str, Any]:
    """Retrieve full configuration payload and raw YAML for the config editor."""
    target_path = _resolve_config_path(config)
    yaml_text = target_path.read_text(encoding="utf-8")
    cfg = load_config(target_path)
    return {
        "preset_id": target_path.stem,
        "config": cfg.model_dump(),
        "yaml_text": yaml_text,
    }


@app.post("/api/v1/config/validate")
def api_validate_config(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
    """Validate configuration YAML or JSON dict against Pydantic schema."""
    yaml_text = payload.get("yaml_text")
    if yaml_text:
        try:
            data = yaml.safe_load(yaml_text)
            cfg = WarehouseConfig.model_validate(data)
            return {
                "valid": True,
                "site_id": cfg.site.id,
                "site_name": cfg.site.name,
                "aisles_count": len(cfg.layout.aisles),
            }
        except Exception as exc:
            return {"valid": False, "error": str(exc)}
    elif "config" in payload:
        try:
            cfg = WarehouseConfig.model_validate(payload["config"])
            return {"valid": True, "site_id": cfg.site.id}
        except Exception as exc:
            return {"valid": False, "error": str(exc)}
    return {"valid": False, "error": "Missing yaml_text or config payload"}


@app.post("/api/v1/config/save")
def api_save_config(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
    """Save custom configuration YAML to configs/presets/."""
    preset_name = payload.get("preset_name", "custom").strip()
    yaml_text = payload.get("yaml_text", "")
    if not yaml_text:
        raise HTTPException(status_code=400, detail="Empty yaml_text")

    try:
        data = yaml.safe_load(yaml_text)
        WarehouseConfig.model_validate(data)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Validation failed: {exc}")

    dest_file = PRESETS_DIR / f"{preset_name}.yaml"
    dest_file.write_text(yaml_text, encoding="utf-8")
    return {"saved": True, "path": str(dest_file), "preset_id": preset_name}


@app.post("/api/v1/mission/plan")
def api_mission_plan(
    config: str = Query(default="medium"),
    strategy: str = Query(default="serpentine"),
) -> dict[str, Any]:
    """Generate 4D waypoint mission plan."""
    cfg_path = _resolve_config_path(config)
    cfg = load_config(cfg_path)
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)
    mission = planner.plan_mission(strategy_name=strategy)

    return {
        "mission_id": mission.mission_id,
        "site_id": world.site_id,
        "strategy": strategy,
        "total_flights": mission.total_flights,
        "total_distance_m": round(mission.total_distance_m, 2),
        "estimated_duration_s": round(mission.total_duration_s, 2),
        "bins_scanned": mission.bins_scanned,
        "bins_total": mission.bins_targeted,
        "coverage_pct": mission.coverage_pct,
        "flights": [
            {
                "flight_id": f.flight_id,
                "departure_pad": f.departure_pad_id,
                "arrival_pad": f.arrival_pad_id,
                "waypoints_count": len(f.waypoints),
                "distance_m": round(f.total_distance_m, 2),
                "duration_s": round(f.estimated_duration_s, 2),
                "waypoints": [
                    {
                        "id": wp.id,
                        "pos": list(wp.position),
                        "yaw_rad": round(wp.yaw_rad, 3),
                        "speed_mps": round(wp.speed_mps, 2),
                        "action": wp.action.value,
                        "target_bin_id": wp.target_bin_id,
                    }
                    for wp in f.waypoints
                ],
            }
            for f in mission.flights
        ],
    }


@app.post("/api/v1/mission/run")
def api_mission_run(
    config: str = Query(default="medium"),
    strategy: str = Query(default="serpentine"),
    seed: int = Query(default=42),
    wind_speed: float = Query(default=0.2),
) -> dict[str, Any]:
    """Execute kinematic simulation, scanning, and EKF localisation."""
    cfg_path = _resolve_config_path(config)
    cfg = load_config(cfg_path)
    world = generate_world_from_config(cfg)

    # 1. Plan Mission
    planner = MissionPlanner(world, cfg)
    mission = planner.plan_mission(strategy_name=strategy)
    if not mission.flights:
        raise HTTPException(status_code=400, detail="No flights generated in mission plan")

    # 2. Scanning + Kinematic Flight Simulation
    scanner_sim = ScanningMissionSimulator(
        world=world,
        config=cfg,
        seed=seed,
        wind_speed=wind_speed,
    )
    tracker, sim_results = scanner_sim.simulate_mission_scanning(mission)

    # 3. Localisation EKF on primary flight
    loc_estimator = LocalisationEstimator(world=world, config=cfg, seed=seed)
    loc_res = loc_estimator.run_localisation(sim_results[0])

    # 4. DRIMS Package Exporter
    exporter = DRIMSExporter(config=cfg, drone_id="DRONE-ALPHA-01")
    out_dir = Path("exports") / mission.mission_id
    export_meta = exporter.export_mission(
        world=world,
        tracker=tracker,
        sim_results=sim_results,
        mission_id=mission.mission_id,
        output_dir=out_dir,
    )

    # Downsample states for fast replay transmission (5Hz)
    sampled_states = []
    primary_sim = sim_results[0]
    step = max(1, len(primary_sim.states) // 400)
    for st in primary_sim.states[::step]:
        sampled_states.append(
            {
                "t_s": round(st.timestamp_s, 2),
                "position": [round(st.position[0], 3), round(st.position[1], 3), round(st.position[2], 3)],
                "velocity": [round(st.velocity[0], 2), round(st.velocity[1], 2), round(st.velocity[2], 2)],
                "speed_mps": round(st.speed_mps, 2),
                "yaw_rad": round(st.attitude_rpy[2], 3),
                "battery_pct": round(st.battery_pct, 1),
                "action": st.action,
                "target_bin_id": st.target_bin_id,
            }
        )

    # Scans payload
    scans_data = []
    for b_id, rec in tracker.records.items():
        if rec.scanned:
            scans_data.append(
                {
                    "bin_id": b_id,
                    "label_code": rec.label_code,
                    "read_success": rec.read_success,
                    "confidence": rec.confidence,
                    "read_type": rec.read_type,
                    "symbology": rec.symbology,
                    "decoded_value": rec.decoded_value,
                    "image_ref": rec.image_ref,
                    "drone_pos": list(rec.drone_pos_at_scan),
                    "bbox": rec.bbox,
                    "flight_id": rec.flight_id,
                }
            )

    # Cache run for replay / scan inspector
    run_record = {
        "mission_id": mission.mission_id,
        "site_id": world.site_id,
        "strategy": strategy,
        "total_flights": len(sim_results),
        "bins_targeted": tracker.total_bins,
        "bins_scanned": tracker.scanned_bins,
        "successful_reads": tracker.successful_reads,
        "coverage_pct": tracker.coverage_pct,
        "read_success_rate": tracker.read_success_rate,
        "duration_s": round(primary_sim.total_time_s, 1),
        "distance_m": round(primary_sim.total_distance_flown_m, 1),
        "mean_tracking_error_cm": round(primary_sim.mean_tracking_error_m * 100, 2),
        "mean_ekf_error_cm": round(loc_res.mean_ekf_error_m * 100, 2),
        "p95_ekf_error_cm": round(loc_res.p95_ekf_error_m * 100, 2),
        "idempotency_hash": export_meta["idempotency_hash"],
        "export_dir": str(out_dir),
        "states": sampled_states,
        "waypoints": [
            {"pos": list(wp.position), "yaw_rad": wp.yaw_rad, "action": wp.action.value}
            for wp in mission.flights[0].waypoints
        ],
        "scans": scans_data,
    }
    MISSION_RUNS[mission.mission_id] = run_record

    return run_record


@app.get("/api/v1/mission/replay/{mission_id}")
def api_mission_replay(mission_id: str) -> dict[str, Any]:
    """Retrieve full 4D trajectory states and waypoint sequence for 3D/2D animation."""
    run = MISSION_RUNS.get(mission_id)
    if not run:
        # Check if stored on disk in exports/
        out_dir = Path("exports") / mission_id
        if (out_dir / "mission.json").exists():
            m_sum = json.loads((out_dir / "mission.json").read_text(encoding="utf-8"))
            scans_j = json.loads((out_dir / "scans.json").read_text(encoding="utf-8")) if (out_dir / "scans.json").exists() else []
            telem_j = json.loads((out_dir / "telemetry.json").read_text(encoding="utf-8")) if (out_dir / "telemetry.json").exists() else []
            return {
                "mission_id": mission_id,
                "site_id": m_sum.get("siteId", "WH"),
                "states": [
                    {
                        "t_s": t.get("t_s", 0),
                        "position": [t["position"]["x"], t["position"]["y"], t["position"]["z"]],
                        "speed_mps": t.get("speed_mps", 0),
                        "battery_pct": t.get("battery_pct", 100),
                        "action": t.get("action", "fly"),
                    }
                    for t in telem_j
                ],
                "scans": scans_j,
            }
        raise HTTPException(status_code=404, detail=f"Mission run '{mission_id}' not found")
    return run


@app.get("/api/v1/mission/scans/{mission_id}")
def api_mission_scans(mission_id: str) -> dict[str, Any]:
    """Retrieve scanned bin audit log and decoded label values."""
    run = MISSION_RUNS.get(mission_id)
    if not run:
        out_dir = Path("exports") / mission_id
        scans_path = out_dir / "scans.json"
        if scans_path.exists():
            scans_j = json.loads(scans_path.read_text(encoding="utf-8"))
            return {"mission_id": mission_id, "total_scans": len(scans_j), "scans": scans_j}
        raise HTTPException(status_code=404, detail=f"Mission scans for '{mission_id}' not found")
    return {"mission_id": mission_id, "total_scans": len(run["scans"]), "scans": run["scans"]}


@app.post("/api/v1/mission/compare")
def api_mission_compare(config: str = Query(default="small")) -> dict[str, Any]:
    """Execute strategy benchmark comparison matrix (Serpentine vs Nearest vs Min-Time)."""
    cfg_path = _resolve_config_path(config)
    cfg = load_config(cfg_path)
    world = generate_world_from_config(cfg)
    planner = MissionPlanner(world, cfg)
    comparison = planner.compare_strategies()

    return {
        "site_id": world.site_id,
        "total_bins": len(world.bins),
        "strategies": [
            {
                "strategy": row.strategy,
                "total_flights": row.total_flights,
                "total_distance_m": round(row.total_distance_m, 2),
                "estimated_duration_s": round(row.total_duration_s, 2),
                "estimated_duration_min": round(row.total_duration_s / 60.0, 2),
                "estimated_energy_wh": round(row.total_energy_wh, 2),
                "bins_scanned": int(round(len(world.bins) * (row.coverage_pct / 100.0))),
                "coverage_pct": round(row.coverage_pct, 1),
            }
            for row in comparison
        ],
    }


@app.get("/api/v1/export/download/{mission_id}")
def api_export_download(mission_id: str) -> Response:
    """Download zipped DRIMS package (mission.json, scans.json, telemetry.json, ground_truth.json, complete)."""
    out_dir = Path("exports") / mission_id
    if not out_dir.exists() or not (out_dir / "mission.json").exists():
        raise HTTPException(status_code=404, detail=f"Export package for mission '{mission_id}' not found")

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in out_dir.iterdir():
            if f.is_file():
                zf.writestr(f.name, f.read_bytes())
    buf.seek(0)

    headers = {"Content-Disposition": f'attachment; filename="drims_export_{mission_id}.zip"'}
    return Response(content=buf.getvalue(), media_type="application/zip", headers=headers)


@app.get("/api/v1/safety/scenarios")
def api_safety_scenarios_list() -> list[dict[str, Any]]:
    """List all 11 failure injection scenarios."""
    from drone_nav.safety.models import FailureScenarioId

    return [
        {"id": 1, "code": FailureScenarioId.SCENARIO_1_STATIC_OBSTACLE.value, "name": "Static Obstacle Avoidance & Replanning"},
        {"id": 2, "code": FailureScenarioId.SCENARIO_2_DYNAMIC_OBSTACLE.value, "name": "Dynamic Obstacle Conflict Detection"},
        {"id": 3, "code": FailureScenarioId.SCENARIO_3_PEOPLE_ZONE_SPEED.value, "name": "People Zone Speed Limit Enforcement"},
        {"id": 4, "code": FailureScenarioId.SCENARIO_4_MIN_RACK_CLEARANCE.value, "name": "Minimum Rack Clearance Buffer Enforcement"},
        {"id": 5, "code": FailureScenarioId.SCENARIO_5_GEOFENCE_BREACH.value, "name": "3D Geofence Boundary Containment"},
        {"id": 6, "code": FailureScenarioId.SCENARIO_6_BATTERY_RETURN_HOME.value, "name": "Battery Safe Reserve Return-to-Charge Trigger"},
        {"id": 7, "code": FailureScenarioId.SCENARIO_7_CRITICAL_BATTERY_LAND.value, "name": "Critical Battery Low Emergency Landing"},
        {"id": 8, "code": FailureScenarioId.SCENARIO_8_LINK_LOSS.value, "name": "Telemetry Link Loss Protocol"},
        {"id": 9, "code": FailureScenarioId.SCENARIO_9_LOCALISATION_LOSS.value, "name": "Localisation Loss Protocol"},
        {"id": 10, "code": FailureScenarioId.SCENARIO_10_EMERGENCY_STOP.value, "name": "Emergency Stop Deceleration Clamp"},
        {"id": 11, "code": FailureScenarioId.SCENARIO_11_SOFTWARE_CRASH_WATCHDOG.value, "name": "Software Crash & Hardware Watchdog Failsafe"},
    ]


@app.post("/api/v1/safety/run")
def api_safety_run(
    config: str = Query(default="small"),
    scenario: str = Query(default="all"),
) -> dict[str, Any]:
    """Execute failure injection scenario(s)."""
    cfg_path = _resolve_config_path(config)
    cfg = load_config(cfg_path)
    world = generate_world_from_config(cfg)
    framework = FailureInjectionFramework(world, cfg)

    if scenario.lower() == "all":
        results = framework.run_all_scenarios()
    else:
        try:
            num = int(scenario)
            matches = [m for m in dir(framework) if m.startswith(f"run_scenario_{num}_")]
            if not matches:
                raise HTTPException(status_code=400, detail=f"Scenario number {num} not found")
            runner_fn = getattr(framework, matches[0])
            results = [runner_fn()]
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid scenario; choose 1-11 or 'all'")

    return {
        "site_id": world.site_id,
        "all_passed": all(r.passed for r in results),
        "results": [
            {
                "scenario_id": r.scenario_id.value,
                "name": r.name,
                "description": r.description,
                "passed": r.passed,
                "trigger_time_s": r.trigger_time_s,
                "response_time_s": r.response_time_s,
                "action_taken": r.action_taken,
                "details": r.details,
                "events_log": r.events_log,
            }
            for r in results
        ],
    }


@app.post("/api/v1/scenarios/sweep")
def api_scenarios_sweep(
    config: str = Query(default="small"),
    sweep_type: str = Query(default="wind"),
) -> dict[str, Any]:
    """Execute parameter sweep (wind, speed, strategy) and return results + SVG plot."""
    cfg_path = _resolve_config_path(config)
    cfg = load_config(cfg_path)
    world = generate_world_from_config(cfg)
    runner = ScenarioRunner(world, cfg)

    if sweep_type == "wind":
        report = runner.run_wind_sweep()
    elif sweep_type == "speed":
        report = runner.run_speed_sweep()
    elif sweep_type == "strategy":
        report = runner.run_strategy_sweep()
    else:
        raise HTTPException(status_code=400, detail=f"Unknown sweep_type '{sweep_type}'")

    svg_file = Path("docs/plots") / f"sweep_{sweep_type}_{world.site_id.lower()}.svg"
    export_sweep_plot_svg(report, svg_file)
    svg_content = svg_file.read_text(encoding="utf-8")

    return {
        "sweep_name": report.sweep_name,
        "parameter_swept": report.parameter_swept,
        "results": [
            {
                "iteration": r.iteration,
                "params": r.params,
                "distance_m": r.distance_m,
                "flight_time_s": r.flight_time_s,
                "energy_consumed_wh": r.energy_consumed_wh,
                "mean_tracking_error_cm": r.mean_tracking_error_cm,
                "p95_tracking_error_cm": r.p95_tracking_error_cm,
                "bins_scanned": r.bins_scanned,
                "coverage_pct": r.coverage_pct,
            }
            for r in report.results
        ],
        "svg_chart": svg_content,
    }


# ============================================================================
# Web UI Index Page
# ============================================================================


@app.get("/", response_class=HTMLResponse)
def index_page() -> str:
    """Serve top-down interactive viewer application."""
    candidates = [
        Path(__file__).parent / "templates" / "index.html",
        Path("src/drone_nav/viewer/templates/index.html"),
        Path("/app/src/drone_nav/viewer/templates/index.html"),
    ]
    for p in candidates:
        if p.exists():
            return p.read_text(encoding="utf-8")
    return "<h1>Viewer UI template not found</h1>"
