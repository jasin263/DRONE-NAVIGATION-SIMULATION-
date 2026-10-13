"""Mission flight path visual plotter and SVG vector exporter."""

from pathlib import Path

from drone_nav.planner.models import MissionPlan
from drone_nav.world.models import World


def export_mission_path_svg(world: World, mission_plan: MissionPlan, target_path: str | Path) -> Path:
    """Generate high-resolution standalone SVG plot of the planned flight path."""
    path = Path(target_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fb = world.floor_bounds
    width_m = fb[1] - fb[0]
    height_m = fb[3] - fb[2]

    scale = 22.0 if width_m < 50.0 else 10.0
    margin = 50
    svg_w = int(width_m * scale) + margin * 2
    svg_h = int(height_m * scale) + margin * 2

    def to_svg(x: float, y: float) -> tuple[float, float]:
        sx = margin + (x - fb[0]) * scale
        sy = svg_h - (margin + (y - fb[2]) * scale)
        return sx, sy

    flight_colors = ["#38bdf8", "#f43f5e", "#a855f7", "#eab308", "#10b981", "#fb923c"]

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
            pts_str = " ".join(f"{px},{py}" for px, py in pts)
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

    # Waypoint trajectories for each flight sortie
    for f_idx, flight in enumerate(mission_plan.flights):
        color = flight_colors[f_idx % len(flight_colors)]
        if len(flight.waypoints) < 2:
            continue

        pts = [to_svg(wp.x, wp.y) for wp in flight.waypoints]
        pts_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)

        # Path trace
        svg.append(
            f'<polyline points="{pts_str}" fill="none" stroke="{color}" '
            f'stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" opacity="0.85" />'
        )

        # Waypoint nodes
        for wp in flight.waypoints:
            wx, wy = to_svg(wp.x, wp.y)
            if wp.action.value == "scan":
                svg.append(f'<circle cx="{wx:.1f}" cy="{wy:.1f}" r="2.5" fill="{color}" />')

    # Legend & Header
    title = (
        f"Mission Plan: {mission_plan.mission_id} ({mission_plan.strategy.upper()}) | "
        f"Flights: {mission_plan.total_flights} | Dist: {mission_plan.total_distance_m:.1f}m | "
        f"Coverage: {mission_plan.coverage_pct}% ({mission_plan.bins_scanned}/{mission_plan.bins_targeted})"
    )
    svg.append(
        f'<text x="{margin}" y="30" fill="#f8fafc" font-size="14px" '
        f'font-weight="bold">{title}</text>'
    )
    svg.append("</svg>")

    path.write_text("\n".join(svg), encoding="utf-8")
    return path
