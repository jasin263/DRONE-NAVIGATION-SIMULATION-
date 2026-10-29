"""Warehouse 3D navigation graph built using NetworkX.

Constructs collision-free directed flight pathways connecting aisles, cross-aisle
transit arterials, takeoff pads, and bin scanning positions while strictly respecting
one-way rules, minimum clearances, and no-fly zones.
Supports both longitudinal North-South and longitudinal East-West warehouse layouts.
"""

import math

import networkx as nx
from shapely.geometry import LineString, Point, Polygon

from drone_nav.config.models import WarehouseConfig
from drone_nav.world.models import Bin, World


class NavigationGraph:
    """Directed navigation graph for collision-free warehouse drone routing."""

    def __init__(self, world: World, config: WarehouseConfig) -> None:
        self.world = world
        self.config = config
        self.graph = nx.DiGraph()
        self.transit_altitude_m = min(config.site.ceiling_height - 0.8, 3.5)
        self.clearance_m = config.safety.min_rack_clearance_m
        self._nfz_polygons = [Polygon(nfz.polygon) for nfz in world.no_fly_zones]
        self._build_graph()

    def _is_in_no_fly_zone(self, x: float, y: float, z: float) -> bool:
        pt = Point(x, y)
        for idx, poly in enumerate(self._nfz_polygons):
            if poly.contains(pt):
                nfz = self.world.no_fly_zones[idx]
                if nfz.z_min <= z <= nfz.z_max:
                    return True
        return False

    def _is_edge_in_no_fly_zone(
        self, p1: tuple[float, float, float], p2: tuple[float, float, float]
    ) -> bool:
        line = LineString([(p1[0], p1[1]), (p2[0], p2[1])])
        z_min = min(p1[2], p2[2])
        z_max = max(p1[2], p2[2])
        for idx, poly in enumerate(self._nfz_polygons):
            if poly.intersects(line):
                nfz = self.world.no_fly_zones[idx]
                if not (z_max < nfz.z_min or z_min > nfz.z_max):
                    return True
        return False

    def _is_edge_colliding_obstacle(
        self, p1: tuple[float, float, float], p2: tuple[float, float, float]
    ) -> bool:
        for obs in self.world.static_obstacles:
            b = obs.bounds
            # Expanded by drone clearance
            x0 = b.x_min - self.clearance_m
            x1 = b.x_max + self.clearance_m
            y0 = b.y_min - self.clearance_m
            y1 = b.y_max + self.clearance_m
            z0 = b.z_min
            z1 = b.z_max + self.clearance_m

            # Line segment vs AABB intersection in 2D + Z range check
            line = LineString([(p1[0], p1[1]), (p2[0], p2[1])])
            box_poly = Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
            if box_poly.intersects(line):
                seg_z_min = min(p1[2], p2[2])
                seg_z_max = max(p1[2], p2[2])
                if not (seg_z_max < z0 or seg_z_min > z1):
                    return True
        return False

    def _add_safe_edge(
        self, u: str, v: str, p1: tuple[float, float, float], p2: tuple[float, float, float]
    ) -> bool:
        if self._is_edge_in_no_fly_zone(p1, p2):
            return False
        if self._is_edge_colliding_obstacle(p1, p2):
            return False

        dist = math.dist(p1, p2)
        self.graph.add_edge(u, v, weight=dist)
        return True

    def _build_graph(self) -> None:
        z_transit = self.transit_altitude_m

        # Detect predominant orientation across all aisles
        total_dx = sum(abs(a.x_end - a.x_start) for a in self.world.aisles)
        total_dy = sum(abs(a.y_end - a.y_start) for a in self.world.aisles)
        is_predominantly_x = total_dx >= total_dy

        if not is_predominantly_x:
            # -----------------------------------------------------------------
            # STANDARD NORTH-SOUTH (Y-ALIGNED) WAREHOUSE LAYOUT
            # -----------------------------------------------------------------
            all_aisle_y = []
            for a in self.world.aisles:
                all_aisle_y.extend([a.y_start, a.y_end])

            y_arterial_south = min(all_aisle_y) - 0.45
            y_arterial_north = max(all_aisle_y) + 0.45

            arterial_south_nodes: list[tuple[str, tuple[float, float, float]]] = []
            arterial_north_nodes: list[tuple[str, tuple[float, float, float]]] = []

            for a in self.world.aisles:
                cx = (a.x_start + a.x_end) / 2.0
                y_min = min(a.y_start, a.y_end)
                y_max = max(a.y_start, a.y_end)

                south_junc_id = f"JUNC-SOUTH-{a.id}"
                south_junc_pos = (cx, y_arterial_south, z_transit)
                north_junc_id = f"JUNC-NORTH-{a.id}"
                north_junc_pos = (cx, y_arterial_north, z_transit)

                if not self._is_in_no_fly_zone(*south_junc_pos):
                    self.graph.add_node(south_junc_id, pos=south_junc_pos, type="arterial")
                    arterial_south_nodes.append((south_junc_id, south_junc_pos))

                if not self._is_in_no_fly_zone(*north_junc_pos):
                    self.graph.add_node(north_junc_id, pos=north_junc_pos, type="arterial")
                    arterial_north_nodes.append((north_junc_id, north_junc_pos))

                step_y = 2.0
                num_steps = max(2, int((y_max - y_min) / step_y))
                aisle_nodes = []

                for i in range(num_steps + 1):
                    y = y_min + i * (y_max - y_min) / num_steps
                    node_id = f"NODE-{a.id}-{i}"
                    node_pos = (cx, y, z_transit)
                    if not self._is_in_no_fly_zone(*node_pos):
                        self.graph.add_node(node_id, pos=node_pos, type="aisle", aisle_id=a.id)
                        aisle_nodes.append((node_id, node_pos))

                if aisle_nodes and south_junc_id in self.graph:
                    if not a.one_way or a.direction != "backward":
                        self._add_safe_edge(south_junc_id, aisle_nodes[0][0], south_junc_pos, aisle_nodes[0][1])
                    if not a.one_way or a.direction != "forward":
                        self._add_safe_edge(aisle_nodes[0][0], south_junc_id, aisle_nodes[0][1], south_junc_pos)

                if aisle_nodes and north_junc_id in self.graph:
                    if not a.one_way or a.direction != "forward":
                        self._add_safe_edge(north_junc_id, aisle_nodes[-1][0], north_junc_pos, aisle_nodes[-1][1])
                    if not a.one_way or a.direction != "backward":
                        self._add_safe_edge(aisle_nodes[-1][0], north_junc_id, aisle_nodes[-1][1], north_junc_pos)

                for i in range(len(aisle_nodes) - 1):
                    u_id, u_pos = aisle_nodes[i]
                    v_id, v_pos = aisle_nodes[i + 1]
                    if a.one_way:
                        if a.direction == "forward":
                            self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                        elif a.direction == "backward":
                            self._add_safe_edge(v_id, u_id, v_pos, u_pos)
                    else:
                        self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                        self._add_safe_edge(v_id, u_id, v_pos, u_pos)

            arterial_south_nodes.sort(key=lambda item: item[1][0])
            for i in range(len(arterial_south_nodes) - 1):
                u_id, u_pos = arterial_south_nodes[i]
                v_id, v_pos = arterial_south_nodes[i + 1]
                self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                self._add_safe_edge(v_id, u_id, v_pos, u_pos)

            arterial_north_nodes.sort(key=lambda item: item[1][0])
            for i in range(len(arterial_north_nodes) - 1):
                u_id, u_pos = arterial_north_nodes[i]
                v_id, v_pos = arterial_north_nodes[i + 1]
                self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                self._add_safe_edge(v_id, u_id, v_pos, u_pos)

        else:
            # -----------------------------------------------------------------
            # LONGITUDINAL EAST-WEST (X-ALIGNED) WAREHOUSE LAYOUT (e.g. RSDC Michigan)
            # -----------------------------------------------------------------
            all_aisle_x = []
            for a in self.world.aisles:
                all_aisle_x.extend([a.x_start, a.x_end])

            x_arterial_west = min(all_aisle_x) - 3.0
            x_arterial_east = max(all_aisle_x) + 3.0

            arterial_west_nodes: list[tuple[str, tuple[float, float, float]]] = []
            arterial_east_nodes: list[tuple[str, tuple[float, float, float]]] = []

            step_x = 3.0  # 3m resolution along the longitudinal bays

            for a in self.world.aisles:
                cy = (a.y_start + a.y_end) / 2.0
                x_min = min(a.x_start, a.x_end)
                x_max = max(a.x_start, a.x_end)

                west_junc_id = f"JUNC-WEST-{a.id}"
                west_junc_pos = (x_arterial_west, cy, z_transit)
                east_junc_id = f"JUNC-EAST-{a.id}"
                east_junc_pos = (x_arterial_east, cy, z_transit)

                # Also register standard aliases for general testing
                south_alias_id = f"JUNC-SOUTH-{a.id}"
                north_alias_id = f"JUNC-NORTH-{a.id}"

                if not self._is_in_no_fly_zone(*west_junc_pos):
                    self.graph.add_node(west_junc_id, pos=west_junc_pos, type="arterial")
                    self.graph.add_node(south_alias_id, pos=west_junc_pos, type="arterial")
                    self.graph.add_edge(south_alias_id, west_junc_id, weight=0.0)
                    self.graph.add_edge(west_junc_id, south_alias_id, weight=0.0)
                    arterial_west_nodes.append((west_junc_id, west_junc_pos))

                if not self._is_in_no_fly_zone(*east_junc_pos):
                    self.graph.add_node(east_junc_id, pos=east_junc_pos, type="arterial")
                    self.graph.add_node(north_alias_id, pos=east_junc_pos, type="arterial")
                    self.graph.add_edge(north_alias_id, east_junc_id, weight=0.0)
                    self.graph.add_edge(east_junc_id, north_alias_id, weight=0.0)
                    arterial_east_nodes.append((east_junc_id, east_junc_pos))

                num_steps = max(2, int((x_max - x_min) / step_x))
                aisle_nodes = []

                for i in range(num_steps + 1):
                    x = x_min + i * (x_max - x_min) / num_steps
                    node_id = f"NODE-{a.id}-{i}"
                    node_pos = (x, cy, z_transit)
                    if not self._is_in_no_fly_zone(*node_pos):
                        self.graph.add_node(node_id, pos=node_pos, type="aisle", aisle_id=a.id)
                        aisle_nodes.append((node_id, node_pos))

                # Connect West junction to first aisle node
                if aisle_nodes and west_junc_id in self.graph:
                    first_n = aisle_nodes[0] if a.x_start <= a.x_end else aisle_nodes[-1]
                    if not a.one_way or a.direction != "backward":
                        self._add_safe_edge(west_junc_id, first_n[0], west_junc_pos, first_n[1])
                        self._add_safe_edge(south_alias_id, first_n[0], west_junc_pos, first_n[1])
                    if not a.one_way or a.direction != "forward":
                        self._add_safe_edge(first_n[0], west_junc_id, first_n[1], west_junc_pos)
                        self._add_safe_edge(first_n[0], south_alias_id, first_n[1], west_junc_pos)

                # Connect East junction to last aisle node
                if aisle_nodes and east_junc_id in self.graph:
                    last_n = aisle_nodes[-1] if a.x_start <= a.x_end else aisle_nodes[0]
                    if not a.one_way or a.direction != "forward":
                        self._add_safe_edge(east_junc_id, last_n[0], east_junc_pos, last_n[1])
                        self._add_safe_edge(north_alias_id, last_n[0], east_junc_pos, last_n[1])
                    if not a.one_way or a.direction != "backward":
                        self._add_safe_edge(last_n[0], east_junc_id, last_n[1], east_junc_pos)
                        self._add_safe_edge(last_n[0], north_alias_id, last_n[1], east_junc_pos)

                # Connect internal longitudinal aisle nodes
                for i in range(len(aisle_nodes) - 1):
                    u_id, u_pos = aisle_nodes[i]
                    v_id, v_pos = aisle_nodes[i + 1]
                    if a.one_way:
                        if a.direction == "forward":
                            self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                        elif a.direction == "backward":
                            self._add_safe_edge(v_id, u_id, v_pos, u_pos)
                    else:
                        self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                        self._add_safe_edge(v_id, u_id, v_pos, u_pos)

            # Connect West Crossway nodes vertically along Y
            arterial_west_nodes.sort(key=lambda item: item[1][1])
            for i in range(len(arterial_west_nodes) - 1):
                u_id, u_pos = arterial_west_nodes[i]
                v_id, v_pos = arterial_west_nodes[i + 1]
                self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                self._add_safe_edge(v_id, u_id, v_pos, u_pos)

            # Connect East Crossway nodes vertically along Y
            arterial_east_nodes.sort(key=lambda item: item[1][1])
            for i in range(len(arterial_east_nodes) - 1):
                u_id, u_pos = arterial_east_nodes[i]
                v_id, v_pos = arterial_east_nodes[i + 1]
                self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                self._add_safe_edge(v_id, u_id, v_pos, u_pos)

        # Connect Takeoff & Landing Pads to nearest arterial or aisle nodes
        for pad in self.world.pads:
            pad_node_id = pad.id if pad.id.startswith("PAD-") else f"PAD-{pad.id}"
            pad_pos = pad.position
            self.graph.add_node(pad_node_id, pos=pad_pos, type="pad", pad_id=pad.id)

            candidates = [
                n for n in self.graph.nodes
                if n != pad_node_id and self.graph.nodes[n].get("type") in ["arterial", "aisle"]
            ]
            candidates.sort(key=lambda n: math.dist(pad_pos, self.graph.nodes[n]["pos"]))

            connected = 0
            for cand in candidates[:6]:
                cand_pos = self.graph.nodes[cand]["pos"]
                if self._add_safe_edge(pad_node_id, cand, pad_pos, cand_pos):
                    self._add_safe_edge(cand, pad_node_id, cand_pos, pad_pos)
                    connected += 1
                if connected >= 2:
                    break

    def connect_scan_pose(self, bin_obj: Bin) -> str:
        """Dynamically attach a bin scan pose to the graph and return its node ID."""
        node_id = f"SCAN-{bin_obj.id}"
        if node_id in self.graph:
            return node_id

        pos = bin_obj.recommended_scan_pos
        self.graph.add_node(node_id, pos=pos, type="scan_pose", bin_id=bin_obj.id, aisle_id=bin_obj.aisle_id)

        # Find closest nodes along the same aisle
        aisle_nodes = [
            n for n in self.graph.nodes
            if self.graph.nodes[n].get("aisle_id") == bin_obj.aisle_id and self.graph.nodes[n].get("type") == "aisle"
        ]
        if not aisle_nodes:
            # Fallback to any node in graph
            aisle_nodes = [n for n in self.graph.nodes if n != node_id and self.graph.nodes[n].get("type") in ["aisle", "arterial"]]

        aisle_nodes.sort(key=lambda n: math.dist(pos, self.graph.nodes[n]["pos"]))

        connected = 0
        for cand in aisle_nodes[:4]:
            cand_pos = self.graph.nodes[cand]["pos"]
            if self._add_safe_edge(cand, node_id, cand_pos, pos):
                self._add_safe_edge(node_id, cand, pos, cand_pos)
                connected += 1
            if connected >= 2:
                break

        # Also connect to nearby scan poses in the same aisle for direct smooth traversal
        nearby_scans = [
            n for n in self.graph.nodes
            if n != node_id and self.graph.nodes[n].get("type") == "scan_pose" and self.graph.nodes[n].get("aisle_id") == bin_obj.aisle_id
        ]
        nearby_scans.sort(key=lambda n: math.dist(pos, self.graph.nodes[n]["pos"]))
        for other_scan in nearby_scans[:3]:
            other_pos = self.graph.nodes[other_scan]["pos"]
            if math.dist(pos, other_pos) <= 6.0:
                self._add_safe_edge(node_id, other_scan, pos, other_pos)
                self._add_safe_edge(other_scan, node_id, other_pos, pos)

        return node_id

    def get_pad_node(self, pad_id: str) -> str | None:
        if pad_id in self.graph:
            return pad_id
        name = f"PAD-{pad_id}"
        if name in self.graph:
            return name
        return None
