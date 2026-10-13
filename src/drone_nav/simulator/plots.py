"""Plotting utilities for kinematic flight simulation trajectory and telemetry curves."""

from pathlib import Path

from drone_nav.planner.models import FlightPlan
from drone_nav.simulator.models import SimulationResult
from drone_nav.world.models import World


def export_flight_trajectory_comparison_svg(
    world: World,
    flight_plan: FlightPlan,
    sim_result: SimulationResult,
    target_path: str | Path,
) -> Path:
    """Generate SVG comparing planned vs flown flight trajectory in the warehouse."""
    path = Path(target_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fb = world.floor_bounds
    width_m = fb[1] - fb[0]
    height_m = fb[3] - fb[2]

    scale = 22.0 if width_m < 50.0 else 10.0
    margin = 60
    svg_w = int(width_m * scale) + margin * 2
    svg_h = int(height_m * scale) + margin * 2

    def to_svg(x: float, y: float) -> tuple[float, float]:
        sx = margin + (x - fb[0]) * scale
        sy = svg_h - (margin + (y - fb[2]) * scale)
        return sx, sy

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" '
        f'viewBox="0 0 {svg_w} {svg_h}" style="background:#090d16; font-family: Inter, sans-serif;">',
        f'<rect x="{margin}" y="{margin}" width="{width_m * scale}" height="{height_m * scale}" '
        f'fill="#0f172a" stroke="#1e293b" stroke-width="2" rx="4" />',
    ]

    # Aisles
    for a in world.aisles:
        x1, y1 = to_svg(a.x_start, a.y_start)
        x2, y2 = to_svg(a.x_end, a.y_end)
        corridor_w = a.width * scale
        svg.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#10b981" '
            f'stroke-width="{corridor_w}" stroke-linecap="round" opacity="0.10" />'
        )

    # Racks
    for r in world.racks:
        rx1, ry1 = to_svg(r.bounds.x_min, r.bounds.y_max)
        rx2, ry2 = to_svg(r.bounds.x_max, r.bounds.y_min)
        rw = abs(rx2 - rx1)
        rh = abs(ry2 - ry1)
        top_x = min(rx1, rx2)
        top_y = min(ry1, ry2)
        svg.append(
            f'<rect x="{top_x}" y="{top_y}" width="{rw}" height="{rh}" '
            f'fill="#1e293b" stroke="#334155" stroke-width="1" rx="1" />'
        )

    # No Fly Zones
    for nfz in world.no_fly_zones:
        if len(nfz.polygon) >= 3:
            pts = [to_svg(pt[0], pt[1]) for pt in nfz.polygon]
            pts_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)
            svg.append(
                f'<polygon points="{pts_str}" fill="#ef4444" fill-opacity="0.25" '
                f'stroke="#ef4444" stroke-width="1.5" />'
            )

    # Takeoff Pads
    for pad in world.pads:
        px, py = to_svg(pad.position[0], pad.position[1])
        pr = pad.radius * scale
        svg.append(
            f'<circle cx="{px}" cy="{py}" r="{pr}" fill="#06b6d4" fill-opacity="0.25" '
            f'stroke="#06b6d4" stroke-width="2" />'
        )
        svg.append(
            f'<text x="{px}" y="{py + 4}" fill="#06b6d4" font-size="12px" '
            f'font-weight="bold" text-anchor="middle">H</text>'
        )

    # 1. Planned Trajectory (Dashed Cyan)
    if len(flight_plan.waypoints) >= 2:
        planned_pts = [to_svg(wp.x, wp.y) for wp in flight_plan.waypoints]
        planned_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in planned_pts)
        svg.append(
            f'<polyline points="{planned_str}" fill="none" stroke="#38bdf8" '
            f'stroke-width="2.5" stroke-dasharray="6,4" stroke-linecap="round" opacity="0.75" />'
        )
        for wp in flight_plan.waypoints:
            wx, wy = to_svg(wp.x, wp.y)
            if wp.action.value == "scan":
                svg.append(f'<circle cx="{wx:.1f}" cy="{wy:.1f}" r="2.5" fill="#38bdf8" opacity="0.8" />')

    # 2. Flown Trajectory (Solid Lime/Amber)
    if len(sim_result.states) >= 2:
        flown_pts = [to_svg(s.position[0], s.position[1]) for s in sim_result.states]
        flown_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in flown_pts)
        svg.append(
            f'<polyline points="{flown_str}" fill="none" stroke="#a3e635" '
            f'stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" opacity="0.9" />'
        )

    # Header & Tracking Stats
    title = (
        f"Flight {flight_plan.flight_id} Trajectory: Planned vs Flown | "
        f"Mean Error: {sim_result.mean_tracking_error_m * 100:.1f}cm | "
        f"P95: {sim_result.p95_tracking_error_m * 100:.1f}cm | "
        f"Max: {sim_result.max_tracking_error_m * 100:.1f}cm"
    )
    svg.append(
        f'<text x="{margin}" y="28" fill="#f8fafc" font-size="14px" '
        f'font-weight="bold">{title}</text>'
    )

    # Legend at bottom
    leg_y = svg_h - 18
    svg.append(
        f'<line x1="{margin}" y1="{leg_y}" x2="{margin + 30}" y2="{leg_y}" stroke="#38bdf8" '
        f'stroke-width="2.5" stroke-dasharray="6,4" />'
    )
    svg.append(
        f'<text x="{margin + 36}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">'
        f'Planned Path ({flight_plan.distance_m:.1f}m)</text>'
    )

    svg.append(
        f'<line x1="{margin + 180}" y1="{leg_y}" x2="{margin + 210}" y2="{leg_y}" stroke="#a3e635" '
        f'stroke-width="2" />'
    )
    svg.append(
        f'<text x="{margin + 216}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">'
        f'Flown Path ({sim_result.total_distance_flown_m:.1f}m, {sim_result.total_time_s:.1f}s)</text>'
    )

    svg.append(
        f'<text x="{svg_w - margin}" y="{leg_y + 4}" fill="#64748b" font-size="11px" text-anchor="end">'
        f'Hash: {sim_result.reproducibility_hash[:12]}</text>'
    )

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def export_battery_curve_svg(
    sim_result: SimulationResult,
    target_path: str | Path,
    safe_reserve_pct: float = 20.0,
    critical_reserve_pct: float = 10.0,
) -> Path:
    """Generate SVG showing battery discharge percentage over simulation flight time."""
    path = Path(target_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    svg_w = 720
    svg_h = 360
    pad_left = 60
    pad_right = 30
    pad_top = 50
    pad_bottom = 50

    plot_w = svg_w - pad_left - pad_right
    plot_h = svg_h - pad_top - pad_bottom

    max_t = max(1.0, sim_result.total_time_s)

    def to_plot(t: float, pct: float) -> tuple[float, float]:
        px = pad_left + (t / max_t) * plot_w
        py = pad_top + (1.0 - (pct / 100.0)) * plot_h
        return px, py

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" '
        f'viewBox="0 0 {svg_w} {svg_h}" style="background:#090d16; font-family: Inter, sans-serif;">',
        # Background canvas
        f'<rect x="{pad_left}" y="{pad_top}" width="{plot_w}" height="{plot_h}" '
        f'fill="#0f172a" stroke="#1e293b" stroke-width="1.5" rx="3" />',
    ]

    # Grid lines (every 20% battery)
    for pct_mark in [20, 40, 60, 80, 100]:
        _, gy = to_plot(0, pct_mark)
        svg.append(
            f'<line x1="{pad_left}" y1="{gy}" x2="{pad_left + plot_w}" y2="{gy}" '
            f'stroke="#1e293b" stroke-dasharray="3,3" stroke-width="1" />'
        )
        svg.append(
            f'<text x="{pad_left - 8}" y="{gy + 4}" fill="#64748b" font-size="10px" '
            f'text-anchor="end">{pct_mark}%</text>'
        )

    # Safe and critical reserve threshold reference lines
    _, safe_y = to_plot(0, safe_reserve_pct)
    svg.append(
        f'<line x1="{pad_left}" y1="{safe_y}" x2="{pad_left + plot_w}" y2="{safe_y}" '
        f'stroke="#f59e0b" stroke-dasharray="5,3" stroke-width="1.2" opacity="0.6" />'
    )
    svg.append(
        f'<text x="{pad_left + plot_w - 6}" y="{safe_y - 4}" fill="#f59e0b" font-size="10px" '
        f'text-anchor="end" opacity="0.9">Safe Reserve ({safe_reserve_pct:.0f}%)</text>'
    )

    _, crit_y = to_plot(0, critical_reserve_pct)
    svg.append(
        f'<line x1="{pad_left}" y1="{crit_y}" x2="{pad_left + plot_w}" y2="{crit_y}" '
        f'stroke="#ef4444" stroke-dasharray="5,3" stroke-width="1.2" opacity="0.6" />'
    )
    svg.append(
        f'<text x="{pad_left + plot_w - 6}" y="{crit_y - 4}" fill="#ef4444" font-size="10px" '
        f'text-anchor="end" opacity="0.9">Critical ({critical_reserve_pct:.0f}%)</text>'
    )

    # Battery discharge curve
    if sim_result.states:
        # Sample points to avoid overly large SVG
        step = max(1, len(sim_result.states) // 400)
        sampled_states = sim_result.states[::step]
        if sim_result.states[-1] not in sampled_states:
            sampled_states.append(sim_result.states[-1])

        pts = [to_plot(s.timestamp_s, s.battery_pct) for s in sampled_states]
        pts_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)

        # Polygon under curve
        bottom_right = to_plot(sim_result.total_time_s, 0)
        bottom_left = to_plot(0, 0)
        poly_str = f"{pad_left},{bottom_left[1]} {pts_str} {bottom_right[0]},{bottom_right[1]}"

        svg.append(
            f'<polygon points="{poly_str}" fill="#38bdf8" fill-opacity="0.08" />'
        )
        svg.append(
            f'<polyline points="{pts_str}" fill="none" stroke="#38bdf8" '
            f'stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" />'
        )

    # X-Axis Time Labels
    for t_frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        t_val = t_frac * max_t
        tx, _ = to_plot(t_val, 0)
        svg.append(
            f'<text x="{tx}" y="{pad_top + plot_h + 18}" fill="#64748b" font-size="10px" '
            f'text-anchor="middle">{t_val:.0f}s</text>'
        )

    # Axis Titles & Headers
    title = (
        f"Battery Discharge: Flight {sim_result.flight_id} | "
        f"Initial: {sim_result.initial_battery_wh:.1f}Wh | "
        f"Final: {sim_result.final_battery_wh:.1f}Wh ({sim_result.final_battery_pct:.1f}%) | "
        f"Consumed: {sim_result.energy_consumed_wh:.2f}Wh"
    )
    svg.append(
        f'<text x="{pad_left}" y="28" fill="#f8fafc" font-size="13px" '
        f'font-weight="bold">{title}</text>'
    )
    svg.append(
        f'<text x="{pad_left + plot_w / 2}" y="{svg_h - 12}" fill="#94a3b8" font-size="11px" '
        f'text-anchor="middle">Flight Time (seconds)</text>'
    )

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")
    return path
