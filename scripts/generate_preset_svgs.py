"""Generate standalone SVG top-down visual warehouse maps for all presets."""

import os

from drone_nav.config.loader import load_config
from drone_nav.world.generator import generate_world_from_config


def generate_svgs() -> None:
    os.makedirs("docs/maps", exist_ok=True)

    for name in ["small", "medium", "large"]:
        cfg = load_config(f"configs/presets/{name}.yaml")
        w = generate_world_from_config(cfg)
        fb = w.floor_bounds
        width_m = fb[1] - fb[0]
        height_m = fb[3] - fb[2]

        scale = 35.0 if name == "small" else (22.0 if name == "medium" else 9.0)
        margin = 40
        svg_w = int(width_m * scale) + margin * 2
        svg_h = int(height_m * scale) + margin * 2

        def to_svg(x: float, y: float) -> tuple[float, float]:
            sx = margin + (x - fb[0]) * scale
            sy = svg_h - (margin + (y - fb[2]) * scale)
            return sx, sy

        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" '
            f'viewBox="0 0 {svg_w} {svg_h}" style="background:#0a0e17; font-family: Inter, sans-serif;">',
            f'<rect x="{margin}" y="{margin}" width="{width_m * scale}" height="{height_m * scale}" '
            f'fill="#0f172a" stroke="#334155" stroke-width="2" rx="4" />',
        ]

        # Aisles (flight corridors & centerlines)
        for a in w.aisles:
            x1, y1 = to_svg(a.x_start, a.y_start)
            x2, y2 = to_svg(a.x_end, a.y_end)
            corridor_w = a.width * scale
            svg.append(
                f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#10b981" '
                f'stroke-width="{corridor_w}" stroke-linecap="round" opacity="0.12" />'
            )
            svg.append(
                f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#10b981" '
                f'stroke-width="1.5" stroke-dasharray="6,6" />'
            )

        # No Fly Zones
        for nfz in w.no_fly_zones:
            if len(nfz.polygon) >= 3:
                pts = [to_svg(pt[0], pt[1]) for pt in nfz.polygon]
                pts_str = " ".join(f"{px},{py}" for px, py in pts)
                svg.append(
                    f'<polygon points="{pts_str}" fill="#ef4444" fill-opacity="0.25" '
                    f'stroke="#ef4444" stroke-width="1.5" />'
                )

        # People Zones
        for pz in w.people_zones:
            if len(pz.polygon) >= 3:
                pts = [to_svg(pt[0], pt[1]) for pt in pz.polygon]
                pts_str = " ".join(f"{px},{py}" for px, py in pts)
                svg.append(
                    f'<polygon points="{pts_str}" fill="#f59e0b" fill-opacity="0.18" '
                    f'stroke="#f59e0b" stroke-width="1.5" stroke-dasharray="4,4" />'
                )

        # Static Obstacles
        for obs in w.static_obstacles:
            ox1, oy1 = to_svg(obs.bounds.x_min, obs.bounds.y_max)
            ox2, oy2 = to_svg(obs.bounds.x_max, obs.bounds.y_min)
            ow = abs(ox2 - ox1)
            oh = abs(oy2 - oy1)
            top_x = min(ox1, ox2)
            top_y = min(oy1, oy2)
            svg.append(
                f'<rect x="{top_x}" y="{top_y}" width="{ow}" height="{oh}" '
                f'fill="#64748b" stroke="#94a3b8" stroke-width="1" />'
            )

        # Racks
        for r in w.racks:
            rx1, ry1 = to_svg(r.bounds.x_min, r.bounds.y_max)
            rx2, ry2 = to_svg(r.bounds.x_max, r.bounds.y_min)
            rw = abs(rx2 - rx1)
            rh = abs(ry2 - ry1)
            top_x = min(rx1, rx2)
            top_y = min(ry1, ry2)
            svg.append(
                f'<rect x="{top_x}" y="{top_y}" width="{rw}" height="{rh}" '
                f'fill="#1e293b" stroke="#3b82f6" stroke-width="1" rx="2" />'
            )

        # Pads & Charging Stations
        for p in w.pads:
            px, py = to_svg(p.position[0], p.position[1])
            pr = p.radius * scale
            svg.append(
                f'<circle cx="{px}" cy="{py}" r="{pr}" fill="#06b6d4" fill-opacity="0.25" '
                f'stroke="#06b6d4" stroke-width="2" />'
            )
            svg.append(
                f'<text x="{px}" y="{py + 4}" fill="#06b6d4" font-size="12px" '
                f'font-weight="bold" text-anchor="middle">H</text>'
            )

        # Fiducial Markers
        for m in w.markers:
            mx, my = to_svg(m.position[0], m.position[1])
            svg.append(
                f'<rect x="{mx - 2.5}" y="{my - 2.5}" width="5" height="5" '
                f'fill="#eab308" stroke="#000" stroke-width="0.5" />'
            )

        # Title and Stats Header
        title_text = f"{w.site_name} | {len(w.bins):,} Bins | {len(w.racks)} Racks | {len(w.aisles)} Aisles | {len(w.markers)} Fiducials"
        svg.append(
            f'<text x="{margin}" y="26" fill="#f1f5f9" font-size="14px" '
            f'font-weight="bold">{title_text}</text>'
        )
        svg.append("</svg>")

        target_file = f"docs/maps/{name}_warehouse.svg"
        with open(target_file, "w", encoding="utf-8") as f:
            f.write("\n".join(svg))
        print(f"Generated {target_file}")


if __name__ == "__main__":
    generate_svgs()
