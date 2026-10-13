"""SVG visualization plotter for batch parameter sweeps."""

from pathlib import Path

from drone_nav.scenarios.models import SweepReport


def export_sweep_plot_svg(report: SweepReport, output_path: Path | str) -> Path:
    """Generate professional SVG benchmark chart for a parameter sweep."""
    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    w = 800
    h = 450
    margin_left = 80
    margin_right = 60
    margin_top = 60
    margin_bottom = 60

    plot_w = w - margin_left - margin_right
    plot_h = h - margin_top - margin_bottom

    res = report.results
    n = len(res)
    if n == 0:
        out_file.write_text("<svg></svg>", encoding="utf-8")
        return out_file

    # Extract metrics
    x_labels = [str(r.params.get(report.parameter_swept, r.iteration)) for r in res]
    y1_vals = [r.mean_tracking_error_cm for r in res]
    y2_vals = [r.energy_consumed_wh for r in res]

    max_y1 = max(1.0, max(y1_vals) * 1.25)
    max_y2 = max(1.0, max(y2_vals) * 1.25)

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="100%" height="100%">',
        '<defs>',
        '  <linearGradient id="bgGrad" x1="0%" y1="0%" x2="100%" y2="100%">',
        '    <stop offset="0%" stop-color="#0f172a"/>',
        '    <stop offset="100%" stop-color="#1e293b"/>',
        '  </linearGradient>',
        '</defs>',
        f'<rect width="{w}" height="{h}" fill="url(#bgGrad)" rx="8"/>',
        f'<text x="{w/2}" y="32" fill="#f8fafc" font-size="16" font-weight="bold" text-anchor="middle">{report.sweep_name} Benchmark</text>',
    ]

    # Grid lines
    for i in range(5):
        y_pos = margin_top + (plot_h / 4) * i
        val1 = max_y1 * (1.0 - i / 4)
        svg_parts.append(
            f'<line x1="{margin_left}" y1="{y_pos:.1f}" x2="{w - margin_right}" y2="{y_pos:.1f}" stroke="#334155" stroke-dasharray="4,4"/>'
        )
        svg_parts.append(
            f'<text x="{margin_left - 10}" y="{y_pos + 4:.1f}" fill="#38bdf8" font-size="10" text-anchor="end">{val1:.1f} cm</text>'
        )

    # Plot lines
    pts_y1 = []
    pts_y2 = []
    for idx, (y1, y2) in enumerate(zip(y1_vals, y2_vals)):
        x_pos = margin_left + (plot_w / max(1, n - 1)) * idx if n > 1 else margin_left + plot_w / 2
        norm_y1 = margin_top + plot_h * (1.0 - y1 / max_y1)
        norm_y2 = margin_top + plot_h * (1.0 - y2 / max_y2)

        pts_y1.append(f"{x_pos:.1f},{norm_y1:.1f}")
        pts_y2.append(f"{x_pos:.1f},{norm_y2:.1f}")

        # X axis tick labels
        svg_parts.append(
            f'<text x="{x_pos:.1f}" y="{h - margin_bottom + 22}" fill="#94a3b8" font-size="11" text-anchor="middle">{x_labels[idx]}</text>'
        )

    # Polylines
    svg_parts.append(
        f'<polyline points="{" ".join(pts_y1)}" fill="none" stroke="#38bdf8" stroke-width="3" stroke-linecap="round"/>'
    )
    svg_parts.append(
        f'<polyline points="{" ".join(pts_y2)}" fill="none" stroke="#ec4899" stroke-width="3" stroke-linecap="round" stroke-dasharray="6,3"/>'
    )

    # Data points
    for pt in pts_y1:
        px, py = pt.split(",")
        svg_parts.append(f'<circle cx="{px}" cy="{py}" r="4.5" fill="#38bdf8" stroke="#ffffff" stroke-width="1.5"/>')

    for pt in pts_y2:
        px, py = pt.split(",")
        svg_parts.append(f'<circle cx="{px}" cy="{py}" r="4.5" fill="#ec4899" stroke="#ffffff" stroke-width="1.5"/>')

    # Legend
    svg_parts.append(f'<g transform="translate({w - 280}, 20)">')
    svg_parts.append('<circle cx="0" cy="0" r="4" fill="#38bdf8"/>')
    svg_parts.append('<text x="10" y="4" fill="#38bdf8" font-size="11">Mean Tracking Error (cm)</text>')
    svg_parts.append('<circle cx="160" cy="0" r="4" fill="#ec4899"/>')
    svg_parts.append('<text x="170" y="4" fill="#ec4899" font-size="11">Energy (Wh)</text>')
    svg_parts.append('</g>')

    # Axis label
    svg_parts.append(
        f'<text x="{w/2}" y="{h - 12}" fill="#cbd5e1" font-size="12" text-anchor="middle">{report.parameter_swept}</text>'
    )
    svg_parts.append('</svg>')

    out_file.write_text("\n".join(svg_parts), encoding="utf-8")
    return out_file
