"""3D geofence, no-fly zone containment, and people zone speed limit monitors."""

from shapely.geometry import Point, Polygon

from drone_nav.config.models import WarehouseConfig
from drone_nav.world.models import World


class GeofenceMonitor:
    """Monitors 3D warehouse outer perimeter, no-fly zones, and people zones."""

    def __init__(self, world: World, config: WarehouseConfig) -> None:
        self.world = world
        self.config = config
        self.margin_m = config.safety.geofence_margin_m

        fb = config.site.floor_bounds
        self.x_min = fb.x_min + self.margin_m
        self.x_max = fb.x_max - self.margin_m
        self.y_min = fb.y_min + self.margin_m
        self.y_max = fb.y_max - self.margin_m
        self.z_min = 0.2
        self.z_max = max(1.0, config.site.ceiling_height - self.margin_m)

        # Precompute Shapely polygons for fast spatial intersection
        self.nfz_polygons = [
            (nfz.id, nfz.name, Polygon(nfz.polygon), nfz.z_min, nfz.z_max)
            for nfz in world.no_fly_zones
        ]

        self.people_polygons = [
            (pz.id, pz.name, Polygon(pz.polygon), pz.max_speed_mps)
            for pz in world.people_zones
        ]

    def check_outer_perimeter(
        self, position: tuple[float, float, float]
    ) -> tuple[bool, str | None]:
        """Check if 3D coordinate violates the warehouse outer boundary buffer."""
        x, y, z = position
        if x < self.x_min:
            return False, f"Breached West boundary ({x:.2f}m < {self.x_min:.2f}m)"
        if x > self.x_max:
            return False, f"Breached East boundary ({x:.2f}m > {self.x_max:.2f}m)"
        if y < self.y_min:
            return False, f"Breached South boundary ({y:.2f}m < {self.y_min:.2f}m)"
        if y > self.y_max:
            return False, f"Breached North boundary ({y:.2f}m > {self.y_max:.2f}m)"
        if z > self.z_max:
            return False, f"Breached Ceiling buffer ({z:.2f}m > {self.z_max:.2f}m)"
        if z < self.z_min:
            return False, f"Breached Floor clearance ({z:.2f}m < {self.z_min:.2f}m)"
        return True, None

    def check_no_fly_zones(
        self, position: tuple[float, float, float]
    ) -> tuple[bool, str | None]:
        """Check if position is inside any restricted 3D no-fly zone."""
        x, y, z = position
        pt = Point(x, y)
        for nfz_id, name, poly, z_bot, z_top in self.nfz_polygons:
            if z_bot <= z <= z_top and poly.contains(pt):
                return False, f"Inside No-Fly Zone '{name}' ({nfz_id}) at z={z:.2f}m"
        return True, None

    def check_people_zones(
        self, position: tuple[float, float, float]
    ) -> tuple[bool, float | None, str | None]:
        """Check if position is inside a people zone with speed restriction.

        Returns:
            in_zone: True if within people zone
            speed_limit: Maximum allowed speed in m/s, or None
            zone_name: Identifier of the zone
        """
        x, y, _ = position
        pt = Point(x, y)
        for pz_id, name, poly, limit in self.people_polygons:
            if poly.contains(pt):
                return True, limit, f"{name} ({pz_id})"
        return False, None, None

    def clamp_velocity_at_boundary(
        self,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Cancel outward velocity components if drone is at or beyond boundary."""
        x, y, z = position
        vx, vy, vz = velocity

        # Push inwards if outside or cancel velocity pointing outward
        if x <= self.x_min and vx < 0:
            vx = 0.0
        elif x >= self.x_max and vx > 0:
            vx = 0.0

        if y <= self.y_min and vy < 0:
            vy = 0.0
        elif y >= self.y_max and vy > 0:
            vy = 0.0

        if z >= self.z_max and vz > 0:
            vz = 0.0
        elif z <= self.z_min and vz < 0:
            vz = 0.0

        return (vx, vy, vz)
