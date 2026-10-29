"""World layout generator from WarehouseConfig.

Converts declarative configuration parameters into fully realized 3D geometry
for aisles, racks, bins, fiducial markers, obstacles, and flight zones.
"""

import math
from typing import Literal

from drone_nav.config.models import RackConfig, WarehouseConfig
from drone_nav.scanning.amazon_catalog import AmazonBinCatalog
from drone_nav.world.models import (
    Aisle,
    Bin,
    BoundingBox3D,
    ChargingStation,
    Door,
    DynamicObstacle,
    FiducialMarker,
    NoFlyZone,
    PeopleZone,
    Rack,
    StaticObstacle,
    TakeoffLandingPad,
    World,
)
from drone_nav.scanning.coil_catalog import SteelCoilCatalog


def generate_world_from_config(
    config: WarehouseConfig,
    catalog: AmazonBinCatalog | SteelCoilCatalog | None = None,
) -> World:
    """Generate a complete 3D World model from a validated WarehouseConfig.

    If *catalog* is a SteelCoilCatalog or the site is an RSDC / steel coil facility,
    real steel coil metadata and client image references are assigned to each unit.
    If *catalog* is an AmazonBinCatalog and available, real ASIN data is assigned.
    Pass ``catalog=None`` (default) for purely synthetic label codes.
    """
    site = config.site
    layout = config.layout
    fb = site.floor_bounds
    standoff = config.drone.camera.standoff_distance_m
    marker_size = config.localisation.marker_size_m
    spacing_rule = config.localisation.marker_spacing

    world = World(
        site_id=site.id,
        site_name=site.name,
        floor_bounds=(fb.x_min, fb.x_max, fb.y_min, fb.y_max),
        ceiling_height=site.ceiling_height,
    )

    is_steel_coil = (
        "coil" in site.id.lower()
        or "coil" in site.name.lower()
        or "rsdc" in site.id.lower()
        or "rsdc" in site.name.lower()
        or isinstance(catalog, SteelCoilCatalog)
    )
    coil_cat = catalog if isinstance(catalog, SteelCoilCatalog) else (SteelCoilCatalog() if is_steel_coil else None)

    marker_counter = 1

    for aisle_cfg in layout.aisles:
        # Determine aisle orientation
        dx = aisle_cfg.x_end - aisle_cfg.x_start
        dy = aisle_cfg.y_end - aisle_cfg.y_start
        length = math.hypot(dx, dy)
        if length < 1e-4:
            raise ValueError(f"Aisle {aisle_cfg.id} length is too small (<0.1mm)")

        ux = dx / length
        uy = dy / length
        half_w = aisle_cfg.width / 2.0

        # Flight corridor bounds
        corridor_x_min = min(aisle_cfg.x_start, aisle_cfg.x_end) - half_w * abs(uy)
        corridor_x_max = max(aisle_cfg.x_start, aisle_cfg.x_end) + half_w * abs(uy)
        corridor_y_min = min(aisle_cfg.y_start, aisle_cfg.y_end) - half_w * abs(ux)
        corridor_y_max = max(aisle_cfg.y_start, aisle_cfg.y_end) + half_w * abs(ux)

        corridor_bbox = BoundingBox3D(
            x_min=corridor_x_min,
            x_max=corridor_x_max,
            y_min=corridor_y_min,
            y_max=corridor_y_max,
            z_min=0.0,
            z_max=site.ceiling_height,
        )

        left_rack_obj: Rack | None = None
        right_rack_obj: Rack | None = None

        rack_pairs: list[tuple[RackConfig | None, Literal["left", "right"]]] = [
            (aisle_cfg.left_rack, "left"),
            (aisle_cfg.right_rack, "right"),
        ]
        for rack_cfg, side in rack_pairs:
            if rack_cfg is None:
                continue

            rack_total_length = rack_cfg.bays * rack_cfg.bay_width
            rack_max_z = rack_cfg.level_heights[-1] + rack_cfg.bin_height

            # Approximate axis-aligned bounding box for rack
            # For aisles predominantly along Y (ux ~ 0, uy ~ 1)
            is_y_aligned = abs(uy) >= abs(ux)

            if is_y_aligned:
                # Aisle runs along Y
                y_min = min(aisle_cfg.y_start, aisle_cfg.y_end)
                y_max = y_min + rack_total_length
                if side == "left":
                    x_max = aisle_cfg.x_start - half_w
                    x_min = x_max - rack_cfg.depth
                    aisle_face_x = x_max
                else:
                    x_min = aisle_cfg.x_start + half_w
                    x_max = x_min + rack_cfg.depth
                    aisle_face_x = x_min

                rack_bbox = BoundingBox3D(
                    x_min=min(x_min, x_max),
                    x_max=max(x_min, x_max),
                    y_min=y_min,
                    y_max=y_max,
                    z_min=0.0,
                    z_max=rack_max_z,
                )
            else:
                # Aisle runs along X
                x_min = min(aisle_cfg.x_start, aisle_cfg.x_end)
                x_max = x_min + rack_total_length
                if side == "left":
                    y_min = aisle_cfg.y_start + half_w
                    y_max = y_min + rack_cfg.depth
                    aisle_face_y = y_min
                else:
                    y_max = aisle_cfg.y_start - half_w
                    y_min = y_max - rack_cfg.depth
                    aisle_face_y = y_max

                rack_bbox = BoundingBox3D(
                    x_min=x_min,
                    x_max=x_max,
                    y_min=min(y_min, y_max),
                    y_max=max(y_min, y_max),
                    z_min=0.0,
                    z_max=rack_max_z,
                )

            rack = Rack(
                id=rack_cfg.id,
                aisle_id=aisle_cfg.id,
                side=side,
                bays=rack_cfg.bays,
                levels=rack_cfg.levels,
                bounds=rack_bbox,
            )

            # Generate Bins
            slot_w = rack_cfg.bay_width / rack_cfg.bins_per_bay_level
            for b in range(rack_cfg.bays):
                for lv in range(rack_cfg.levels):
                    z_base = rack_cfg.level_heights[lv]
                    z_top = z_base + rack_cfg.bin_height

                    for s in range(rack_cfg.bins_per_bay_level):
                        if is_y_aligned:
                            bin_y_min = rack_bbox.y_min + b * rack_cfg.bay_width + s * slot_w
                            bin_y_max = bin_y_min + slot_w
                            bin_bbox = BoundingBox3D(
                                x_min=rack_bbox.x_min,
                                x_max=rack_bbox.x_max,
                                y_min=bin_y_min,
                                y_max=bin_y_max,
                                z_min=z_base,
                                z_max=z_top,
                            )
                            # Scan position in aisle facing bin
                            scan_x = aisle_face_x + standoff * (1.0 if side == "left" else -1.0)
                            scan_y = (bin_y_min + bin_y_max) / 2.0
                            scan_z = (z_base + z_top) / 2.0
                            norm = (1.0, 0.0, 0.0) if side == "left" else (-1.0, 0.0, 0.0)
                        else:
                            bin_x_min = rack_bbox.x_min + b * rack_cfg.bay_width + s * slot_w
                            bin_x_max = bin_x_min + slot_w
                            bin_bbox = BoundingBox3D(
                                x_min=bin_x_min,
                                x_max=bin_x_max,
                                y_min=rack_bbox.y_min,
                                y_max=rack_bbox.y_max,
                                z_min=z_base,
                                z_max=z_top,
                            )
                            scan_x = (bin_x_min + bin_x_max) / 2.0
                            scan_y = aisle_face_y + standoff * (-1.0 if side == "left" else 1.0)
                            scan_z = (z_base + z_top) / 2.0
                            norm = (0.0, -1.0, 0.0) if side == "left" else (0.0, 1.0, 0.0)

                        if rack_cfg.bins_per_bay_level > 1:
                            bin_id = f"{rack.id}-B{b + 1:02d}-L{lv + 1}-S{s + 1}"
                        else:
                            bin_id = f"{rack.id}-B{b + 1:02d}-L{lv + 1}"
                        # Generate label_code: use real Amazon ASIN when catalog available
                        bin_counter = (
                            len(world.bins)  # global monotonic index across all bins
                        )
                        amazon_img_id: str | None = None
                        amazon_items: list = []
                        amazon_expected_qty: int = 0
                        coil_img_id: str | None = None
                        coil_supplier: str | None = None
                        coil_grade: str | None = None
                        coil_weight_kg: float | None = None
                        coil_form_factor: str | None = None
                        coil_landmark: str | None = None

                        if coil_cat is not None:
                            coil_rec = coil_cat.get_by_index(bin_counter)
                            label_code = coil_rec.coil_id
                            coil_img_id = coil_rec.img_id
                            coil_supplier = coil_rec.mill_supplier
                            coil_grade = coil_rec.finish_grade
                            coil_weight_kg = coil_rec.approx_weight_kg
                            coil_form_factor = coil_rec.form_factor
                            coil_landmark = coil_rec.column_landmark
                        elif catalog is not None and getattr(catalog, "is_available", False) and hasattr(catalog, "by_index"):
                            amazon_rec = catalog.by_index(bin_counter)
                            if amazon_rec is not None:
                                label_code = amazon_rec.primary_asin
                                amazon_img_id = amazon_rec.img_id
                                amazon_items = list(amazon_rec.items)
                                amazon_expected_qty = amazon_rec.expected_qty
                            else:
                                label_code = layout.bin_numbering_template.format(
                                    site_id=site.id,
                                    zone_id=layout.zones[0].id if layout.zones else "Z01",
                                    aisle_id=aisle_cfg.id,
                                    rack_id=rack.id,
                                    level=lv + 1,
                                    bay=b + 1,
                                    slot=s + 1,
                                )
                        else:
                            label_code = layout.bin_numbering_template.format(
                                site_id=site.id,
                                zone_id=layout.zones[0].id if layout.zones else "Z01",
                                aisle_id=aisle_cfg.id,
                                rack_id=rack.id,
                                level=lv + 1,
                                bay=b + 1,
                                slot=s + 1,
                            )

                        bin_obj = Bin(
                            id=bin_id,
                            label_code=label_code,
                            site_id=site.id,
                            zone_id=layout.zones[0].id if layout.zones else "Z01",
                            aisle_id=aisle_cfg.id,
                            rack_id=rack.id,
                            bay_index=b,
                            level_index=lv,
                            shelf_slot=s,
                            bounds=bin_bbox,
                            scan_face_normal=norm,
                            recommended_scan_pos=(scan_x, scan_y, scan_z),
                            amazon_img_id=amazon_img_id,
                            amazon_items=amazon_items,
                            amazon_expected_qty=amazon_expected_qty,
                            coil_img_id=coil_img_id,
                            coil_supplier=coil_supplier,
                            coil_grade=coil_grade,
                            coil_weight_kg=coil_weight_kg,
                            coil_form_factor=coil_form_factor,
                            coil_landmark=coil_landmark,
                        )
                        rack.bins.append(bin_obj)
                        world.bins[bin_id] = bin_obj

            # Generate Fiducial Markers along rack front
            bays_to_mark = set()
            if spacing_rule.rack_ends:
                bays_to_mark.add(0)
                bays_to_mark.add(rack_cfg.bays - 1)
            for b in range(0, rack_cfg.bays, spacing_rule.every_n_bays):
                bays_to_mark.add(b)

            marker_z = rack_cfg.level_heights[0] + 0.1  # slightly above bottom beam
            for b in sorted(bays_to_mark):
                if is_y_aligned:
                    m_x = aisle_face_x
                    m_y = rack_bbox.y_min + (b + 0.5) * rack_cfg.bay_width
                    m_norm = (1.0, 0.0, 0.0) if side == "left" else (-1.0, 0.0, 0.0)
                else:
                    m_x = rack_bbox.x_min + (b + 0.5) * rack_cfg.bay_width
                    m_y = aisle_face_y
                    m_norm = (0.0, -1.0, 0.0) if side == "left" else (0.0, 1.0, 0.0)

                marker = FiducialMarker(
                    id=f"TAG-{site.id}-{rack.id}-B{b + 1:02d}",
                    tag_id=marker_counter,
                    position=(m_x, m_y, marker_z),
                    normal=m_norm,
                    size_m=marker_size,
                    rack_id=rack.id,
                    bay_index=b,
                )
                world.markers.append(marker)
                marker_counter += 1

            world.racks.append(rack)
            if side == "left":
                left_rack_obj = rack
            else:
                right_rack_obj = rack

        aisle_obj = Aisle(
            id=aisle_cfg.id,
            name=aisle_cfg.name,
            x_start=aisle_cfg.x_start,
            y_start=aisle_cfg.y_start,
            x_end=aisle_cfg.x_end,
            y_end=aisle_cfg.y_end,
            width=aisle_cfg.width,
            one_way=aisle_cfg.one_way,
            direction=aisle_cfg.direction.value,
            corridor_bounds=corridor_bbox,
            left_rack=left_rack_obj,
            right_rack=right_rack_obj,
        )
        world.aisles.append(aisle_obj)

    # Convert Pads, Charging Stations, Doors
    for pad in layout.takeoff_landing_pads:
        world.pads.append(
            TakeoffLandingPad(
                id=pad.id,
                position=(pad.position[0], pad.position[1], pad.position[2]),
                radius=pad.radius,
            )
        )
    for cs in layout.charging_stations:
        world.charging_stations.append(
            ChargingStation(
                id=cs.id,
                position=(cs.position[0], cs.position[1], cs.position[2]),
                charge_rate_kw=cs.charge_rate_kw,
            )
        )
    for door in layout.doors:
        world.doors.append(
            Door(
                id=door.id,
                position=(door.position[0], door.position[1], door.position[2]),
                width=door.width,
                height=door.height,
            )
        )
    for nfz in layout.no_fly_zones:
        world.no_fly_zones.append(
            NoFlyZone(
                id=nfz.id,
                name=nfz.name,
                polygon=[(pt[0], pt[1]) for pt in nfz.polygon],
                z_min=nfz.z_min,
                z_max=nfz.z_max,
            )
        )
    for pz in layout.people_zones:
        world.people_zones.append(
            PeopleZone(
                id=pz.id,
                name=pz.name,
                polygon=[(pt[0], pt[1]) for pt in pz.polygon],
                max_speed_mps=pz.max_speed_mps,
            )
        )
    for obs in config.obstacles.static:
        world.static_obstacles.append(
            StaticObstacle(
                id=obs.id,
                type=obs.type,
                bounds=BoundingBox3D(
                    x_min=obs.position[0],
                    x_max=obs.position[0] + obs.dimensions[0],
                    y_min=obs.position[1],
                    y_max=obs.position[1] + obs.dimensions[1],
                    z_min=obs.position[2],
                    z_max=obs.position[2] + obs.dimensions[2],
                ),
            )
        )
    for dyn in config.obstacles.dynamic:
        world.dynamic_obstacles.append(
            DynamicObstacle(
                id=dyn.id,
                type=dyn.type,
                path=[(pt[0], pt[1], pt[2]) for pt in dyn.path],
                speed_mps=dyn.speed_mps,
                schedule_start_s=dyn.schedule_start_s,
                schedule_end_s=dyn.schedule_end_s,
            )
        )

    return world
