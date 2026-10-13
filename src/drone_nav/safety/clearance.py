"""Rack structure proximity detection and minimum clearance enforcement."""

import math

from drone_nav.config.models import WarehouseConfig
from drone_nav.world.models import World


class RackClearanceMonitor:
    """Monitors clearance distance between drone and warehouse rack storage structures."""

    def __init__(self, world: World, config: WarehouseConfig) -> None:
        self.world = world
        self.config = config
        self.min_clearance_m = config.safety.min_rack_clearance_m

    def compute_distance_to_box(
        self,
        point: tuple[float, float, float],
        box_min: tuple[float, float, float],
        box_max: tuple[float, float, float],
    ) -> tuple[float, tuple[float, float, float]]:
        """Calculate Euclidean distance from a 3D point to an axis-aligned bounding box.

        Returns:
            distance: Shortest distance to box surface (0.0 if inside)
            normal: Unit direction vector from box surface toward the point
        """
        px, py, pz = point
        bx1, by1, bz1 = box_min
        bx2, by2, bz2 = box_max

        # Closest point on or in box
        cx = max(bx1, min(px, bx2))
        cy = max(by1, min(py, by2))
        cz = max(bz1, min(pz, bz2))

        dx = px - cx
        dy = py - cy
        dz = pz - cz
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist > 1e-6:
            normal = (dx / dist, dy / dist, dz / dist)
        else:
            # Point is inside or on the boundary; push along normal to closest face
            faces = [
                (abs(px - bx1), (-1.0, 0.0, 0.0)),
                (abs(px - bx2), (1.0, 0.0, 0.0)),
                (abs(py - by1), (0.0, -1.0, 0.0)),
                (abs(py - by2), (0.0, 1.0, 0.0)),
                (abs(pz - bz1), (0.0, 0.0, -1.0)),
                (abs(pz - bz2), (0.0, 0.0, 1.0)),
            ]
            faces.sort(key=lambda f: f[0])
            normal = faces[0][1]

        return dist, normal

    def check_rack_clearance(
        self, position: tuple[float, float, float]
    ) -> tuple[bool, float, str | None, tuple[float, float, float]]:
        """Evaluate clearance to nearest storage rack.

        Returns:
            is_safe: True if distance >= min_rack_clearance_m
            min_dist: Distance in metres to closest rack structure
            closest_rack_id: Identifier of closest rack
            repelling_normal: Unit vector pointing away from rack
        """
        min_dist = float("inf")
        closest_rack_id: str | None = None
        best_normal = (0.0, 0.0, 0.0)

        for rack in self.world.racks:
            b = rack.bounds
            dist, normal = self.compute_distance_to_box(
                position,
                (b.x_min, b.y_min, b.z_min),
                (b.x_max, b.y_max, b.z_max),
            )
            if dist < min_dist:
                min_dist = dist
                closest_rack_id = rack.id
                best_normal = normal

        is_safe = min_dist >= self.min_clearance_m
        return is_safe, min_dist, closest_rack_id, best_normal
