"""DRIMS Location CSV importer and exporter.

Parses CSV files containing warehouse locations in the DRIMS ingestion format:
site_id, zone, aisle, rack, level, bin, label_code, x_min, x_max, y_min, y_max, z_min, z_max
"""

import csv
from pathlib import Path

from drone_nav.world.models import Aisle, Bin, BoundingBox3D, Rack, World

REQUIRED_CSV_COLUMNS = [
    "site_id",
    "zone",
    "aisle",
    "rack",
    "level",
    "bin",
    "label_code",
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "z_min",
    "z_max",
]


class CSVImportError(Exception):
    """Raised when CSV structure or records fail validation."""

    pass


def import_drims_csv(file_path: str | Path, default_ceiling_height: float = 6.0) -> World:
    """Import a DRIMS location CSV into a World model."""
    path = Path(file_path)
    if not path.exists():
        raise CSVImportError(f"DRIMS CSV file not found: {path.resolve()}")

    bins: list[Bin] = []
    site_id = "UNKNOWN"

    min_x, max_x = float("inf"), float("-inf")
    min_y, max_y = float("inf"), float("-inf")
    max_z = float("-inf")

    aisle_bins: dict[str, list[Bin]] = {}
    rack_bins: dict[str, list[Bin]] = {}

    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise CSVImportError(f"CSV file '{path}' is empty or has no header")

        # Check required columns
        missing = [col for col in REQUIRED_CSV_COLUMNS if col not in reader.fieldnames]
        if missing:
            raise CSVImportError(
                f"CSV file '{path}' is missing required columns: {missing}. "
                f"Found: {reader.fieldnames}"
            )

        row_num = 1
        for row in reader:
            row_num += 1
            try:
                s_id = row["site_id"].strip()
                site_id = s_id
                zone_id = row["zone"].strip()
                aisle_id = row["aisle"].strip()
                rack_id = row["rack"].strip()
                level_idx = int(row["level"].strip())
                bay_idx = int(row["bin"].strip())
                label = row["label_code"].strip()

                x0 = float(row["x_min"].strip())
                x1 = float(row["x_max"].strip())
                y0 = float(row["y_min"].strip())
                y1 = float(row["y_max"].strip())
                z0 = float(row["z_min"].strip())
                z1 = float(row["z_max"].strip())
            except (ValueError, KeyError) as exc:
                raise CSVImportError(f"Row {row_num} invalid data in '{path}': {exc}") from exc

            if x1 <= x0 or y1 <= y0 or z1 <= z0:
                raise CSVImportError(
                    f"Row {row_num} in '{path}' has non-positive volume: "
                    f"x:[{x0},{x1}], y:[{y0},{y1}], z:[{z0},{z1}]"
                )

            bbox = BoundingBox3D(x_min=x0, x_max=x1, y_min=y0, y_max=y1, z_min=z0, z_max=z1)
            min_x = min(min_x, x0)
            max_x = max(max_x, x1)
            min_y = min(min_y, y0)
            max_y = max(max_y, y1)
            max_z = max(max_z, z1)

            bin_id = f"{rack_id}-B{bay_idx:02d}-L{level_idx}"
            # Standoff in Y or X
            cx, cy, cz = bbox.center
            # Default normal pointing toward warehouse center or +X
            norm = (1.0, 0.0, 0.0)
            scan_pos = (cx + 1.0, cy, cz)

            b_obj = Bin(
                id=bin_id,
                label_code=label,
                site_id=s_id,
                zone_id=zone_id,
                aisle_id=aisle_id,
                rack_id=rack_id,
                bay_index=bay_idx - 1,
                level_index=level_idx - 1,
                shelf_slot=0,
                bounds=bbox,
                scan_face_normal=norm,
                recommended_scan_pos=scan_pos,
            )
            bins.append(b_obj)
            aisle_bins.setdefault(aisle_id, []).append(b_obj)
            rack_bins.setdefault(rack_id, []).append(b_obj)

    if not bins:
        raise CSVImportError(f"No bin records found in '{path}'")

    # Add margin to bounds
    floor_bounds = (max(0.0, min_x - 2.0), max_x + 2.0, max(0.0, min_y - 2.0), max_y + 2.0)
    ceiling_h = max(default_ceiling_height, max_z + 1.5)

    world = World(
        site_id=site_id,
        site_name=f"Site {site_id} (CSV Import)",
        floor_bounds=floor_bounds,
        ceiling_height=ceiling_h,
    )

    for b in bins:
        world.bins[b.id] = b

    # Create Rack objects
    for r_id, r_bins in rack_bins.items():
        rx0 = min(b.bounds.x_min for b in r_bins)
        rx1 = max(b.bounds.x_max for b in r_bins)
        ry0 = min(b.bounds.y_min for b in r_bins)
        ry1 = max(b.bounds.y_max for b in r_bins)
        rz0 = min(b.bounds.z_min for b in r_bins)
        rz1 = max(b.bounds.z_max for b in r_bins)
        bays_cnt = len({b.bay_index for b in r_bins})
        levels_cnt = len({b.level_index for b in r_bins})

        r_obj = Rack(
            id=r_id,
            aisle_id=r_bins[0].aisle_id,
            side="left",
            bays=bays_cnt,
            levels=levels_cnt,
            bounds=BoundingBox3D(x_min=rx0, x_max=rx1, y_min=ry0, y_max=ry1, z_min=rz0, z_max=rz1),
            bins=r_bins,
        )
        world.racks.append(r_obj)

    # Create Aisle objects
    for a_id, a_bins in aisle_bins.items():
        ax0 = min(b.bounds.x_min for b in a_bins)
        ax1 = max(b.bounds.x_max for b in a_bins)
        ay0 = min(b.bounds.y_min for b in a_bins)
        ay1 = max(b.bounds.y_max for b in a_bins)

        aisle_obj = Aisle(
            id=a_id,
            name=f"Aisle {a_id}",
            x_start=ax0,
            y_start=ay0,
            x_end=ax0,
            y_end=ay1,
            width=2.0,
            one_way=False,
            direction="bidirectional",
            corridor_bounds=BoundingBox3D(
                x_min=ax0 - 1.0,
                x_max=ax1 + 1.0,
                y_min=ay0,
                y_max=ay1,
                z_min=0.0,
                z_max=ceiling_h,
            ),
        )
        world.aisles.append(aisle_obj)

    return world


def export_drims_csv(world: World, target_path: str | Path) -> Path:
    """Export a World model to DRIMS CSV format."""
    path = Path(target_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=REQUIRED_CSV_COLUMNS)
        writer.writeheader()
        for b in sorted(world.bins.values(), key=lambda x: x.id):
            writer.writerow(
                {
                    "site_id": b.site_id,
                    "zone": b.zone_id,
                    "aisle": b.aisle_id,
                    "rack": b.rack_id,
                    "level": b.level_index + 1,
                    "bin": b.bay_index + 1,
                    "label_code": b.label_code,
                    "x_min": f"{b.bounds.x_min:.3f}",
                    "x_max": f"{b.bounds.x_max:.3f}",
                    "y_min": f"{b.bounds.y_min:.3f}",
                    "y_max": f"{b.bounds.y_max:.3f}",
                    "z_min": f"{b.bounds.z_min:.3f}",
                    "z_max": f"{b.bounds.z_max:.3f}",
                }
            )
    return path
