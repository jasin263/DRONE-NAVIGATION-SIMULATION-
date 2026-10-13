"""Vector SVG plotters for sensor fusion trajectories, error envelopes, and marker fixes."""

from pathlib import Path

from drone_nav.localisation.models import LocalisationResult
from drone_nav.world.models import World


def export_localisation_trajectory_svg(
    world: World,
    loc_result: LocalisationResult,
    target_path: str | Path,
) -> Path:
    """Generate high-resolution SVG comparing Ground Truth, EKF, and Dead-Reckoning paths."""
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

    # Fiducial Markers on racks
    for m in world.markers:
        mx, my = to_svg(m.position[0], m.position[1])
        svg.append(
            f'<circle cx="{mx:.1f}" cy="{my:.1f}" r="3.0" fill="#c084fc" opacity="0.75" />'
        )

    # 1. Dead Reckoning Path (Drifting Orange/Red)
    if loc_result.estimates:
        dr_step = max(1, len(loc_result.estimates) // 300)
        dr_pts = [
            to_svg(e.dead_reckoning_position[0], e.dead_reckoning_position[1])
            for e in loc_result.estimates[::dr_step]
        ]
        dr_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in dr_pts)
        svg.append(
            f'<polyline points="{dr_str}" fill="none" stroke="#f97316" '
            f'stroke-width="1.6" stroke-dasharray="4,4" opacity="0.65" />'
        )

    # 2. Ground Truth Path (Solid Cyan)
    if loc_result.estimates:
        gt_step = max(1, len(loc_result.estimates) // 500)
        gt_pts = [
            to_svg(e.true_position[0], e.true_position[1])
            for e in loc_result.estimates[::gt_step]
        ]
        gt_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in gt_pts)
        svg.append(
            f'<polyline points="{gt_str}" fill="none" stroke="#38bdf8" '
            f'stroke-width="2.2" stroke-linecap="round" opacity="0.8" />'
        )

    # 3. EKF Estimated Path (Lime Green)
    if loc_result.estimates:
        ekf_step = max(1, len(loc_result.estimates) // 500)
        ekf_pts = [
            to_svg(e.estimated_position[0], e.estimated_position[1])
            for e in loc_result.estimates[::ekf_step]
        ]
        ekf_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in ekf_pts)
        svg.append(
            f'<polyline points="{ekf_str}" fill="none" stroke="#a3e635" '
            f'stroke-width="1.8" stroke-linecap="round" opacity="0.9" />'
        )

    # Marker Detections fixes
    for obs in loc_result.observations[::max(1, len(loc_result.observations) // 80)]:
        ox, oy = to_svg(obs.observed_pos[0], obs.observed_pos[1])
        svg.append(
            f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="3.5" fill="#4ade80" stroke="#052e16" stroke-width="1" />'
        )

    # Header & Legend
    title = (
        f"Localisation Fusion: EKF vs Dead-Reckoning | "
        f"Mean Error: {loc_result.mean_ekf_error_m * 100:.1f}cm | "
        f"P95: {loc_result.p95_ekf_error_m * 100:.1f}cm | "
        f"Max DR Drift: {loc_result.max_dr_error_m:.1f}m"
    )
    svg.append(
        f'<text x="{margin}" y="28" fill="#f8fafc" font-size="14px" '
        f'font-weight="bold">{title}</text>'
    )

    leg_y = svg_h - 18
    svg.append(
        f'<line x1="{margin}" y1="{leg_y}" x2="{margin + 25}" y2="{leg_y}" stroke="#38bdf8" stroke-width="2.2" />'
    )
    svg.append(
        f'<text x="{margin + 30}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">Ground Truth</text>'
    )

    svg.append(
        f'<line x1="{margin + 120}" y1="{leg_y}" x2="{margin + 145}" y2="{leg_y}" stroke="#a3e635" stroke-width="2" />'
    )
    svg.append(
        f'<text x="{margin + 150}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">EKF Fused</text>'
    )

    svg.append(
        f'<line x1="{margin + 230}" y1="{leg_y}" x2="{margin + 255}" y2="{leg_y}" stroke="#f97316" stroke-width="1.8" stroke-dasharray="4,4" />'
    )
    svg.append(
        f'<text x="{margin + 260}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">Raw IMU Drift</text>'
    )

    svg.append(
        f'<circle cx="{margin + 365}" cy="{leg_y}" r="3.5" fill="#4ade80" />'
    )
    svg.append(
        f'<text x="{margin + 375}" y="{leg_y + 4}" fill="#94a3b8" font-size="11px">Marker Fix ({loc_result.total_marker_detections})</text>'
    )

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def export_localisation_error_svg(
    loc_result: LocalisationResult,
    target_path: str | Path,
) -> Path:
    """Generate SVG showing position error over time with 3-sigma uncertainty envelope."""
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

    max_t = max(1.0, loc_result.total_time_s)
    # Clip max error scale to 0.50m (50cm) to clearly see centimeters
    max_err_axis_m = 0.50

    def to_plot(t: float, err: float) -> tuple[float, float]:
        px = pad_left + (t / max_t) * plot_w
        norm_y = min(1.0, max(0.0, err / max_err_axis_m))
        py = pad_top + (1.0 - norm_y) * plot_h
        return px, py

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" '
        f'viewBox="0 0 {svg_w} {svg_h}" style="background:#090d16; font-family: Inter, sans-serif;">',
        f'<rect x="{pad_left}" y="{pad_top}" width="{plot_w}" height="{plot_h}" '
        f'fill="#0f172a" stroke="#1e293b" stroke-width="1.5" rx="3" />',
    ]

    # Gridlines (every 10cm)
    for err_m in [0.10, 0.20, 0.30, 0.40, 0.50]:
        _, gy = to_plot(0, err_m)
        svg.append(
            f'<line x1="{pad_left}" y1="{gy}" x2="{pad_left + plot_w}" y2="{gy}" '
            f'stroke="#1e293b" stroke-dasharray="3,3" stroke-width="1" />'
        )
        svg.append(
            f'<text x="{pad_left - 8}" y="{gy + 4}" fill="#64748b" font-size="10px" '
            f'text-anchor="end">{int(err_m * 100)}cm</text>'
        )

    # 3-sigma envelope and error curve
    if loc_result.estimates:
        step = max(1, len(loc_result.estimates) // 300)
        samples = loc_result.estimates[::step]
        if loc_result.estimates[-1] not in samples:
            samples.append(loc_result.estimates[-1])

        # 3-sigma uncertainty boundary curve
        upper_pts = [to_plot(s.timestamp_s, s.pos_uncertainty_3sigma_m) for s in samples]
        upper_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in upper_pts)
        svg.append(
            f'<polyline points="{upper_str}" fill="none" stroke="#818cf8" '
            f'stroke-width="1.4" stroke-dasharray="3,3" opacity="0.75" />'
        )

        # Actual EKF position error
        err_pts = [to_plot(s.timestamp_s, s.position_error_m) for s in samples]
        err_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in err_pts)
        svg.append(
            f'<polyline points="{err_str}" fill="none" stroke="#38bdf8" '
            f'stroke-width="2.0" stroke-linecap="round" />'
        )

    # Marker Observation ticks on X axis
    for obs in loc_result.observations[::max(1, len(loc_result.observations) // 60)]:
        tx, _ = to_plot(obs.timestamp_s, 0.0)
        svg.append(
            f'<line x1="{tx:.1f}" y1="{pad_top + plot_h - 6}" x2="{tx:.1f}" y2="{pad_top + plot_h}" '
            f'stroke="#4ade80" stroke-width="1.8" opacity="0.8" />'
        )

    # X axis time labels
    for t_frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        t_val = t_frac * max_t
        tx, _ = to_plot(t_val, 0)
        svg.append(
            f'<text x="{tx}" y="{pad_top + plot_h + 18}" fill="#64748b" font-size="10px" '
            f'text-anchor="middle">{t_val:.0f}s</text>'
        )

    # Headers & legend
    title = (
        f"Localisation Error: EKF Fused Position Error & 3-Sigma Envelope | "
        f"Mean: {loc_result.mean_ekf_error_m * 100:.2f}cm | P95: {loc_result.p95_ekf_error_m * 100:.2f}cm"
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
