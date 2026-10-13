"""Warehouse 3D navigation graph built using NetworkX.

Constructs collision-free directed flight pathways connecting aisles, cross-aisle
transit arterials, takeoff pads, and bin scanning positions while strictly respecting
one-way rules, minimum clearances, and no-fly zones.
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

        # 1. Establish Arterial Cross-Aisles (South & North ends of warehouse)
        # Find minimum and maximum Y across all aisles
        all_aisle_y = []
        for a in self.world.aisles:
            all_aisle_y.extend([a.y_start, a.y_end])

        # Safe arterials right in front of aisle mouths
        y_arterial_south = min(all_aisle_y) - 0.45
        y_arterial_north = max(all_aisle_y) + 0.45

        arterial_south_nodes: list[tuple[str, tuple[float, float, float]]] = []
        arterial_north_nodes: list[tuple[str, tuple[float, float, float]]] = []

        # 2. Add nodes along each aisle
        for a in self.world.aisles:
            cx = (a.x_start + a.x_end) / 2.0
            y_min = min(a.y_start, a.y_end)
            y_max = max(a.y_start, a.y_end)

            # Arterial junction nodes for this aisle
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

            # Nodes along aisle corridor
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

            # Connect south junction to first aisle node
            if aisle_nodes and south_junc_id in self.graph:
                if not a.one_way or a.direction != "backward":
                    self._add_safe_edge(south_junc_id, aisle_nodes[0][0], south_junc_pos, aisle_nodes[0][1])
                if not a.one_way or a.direction != "forward":
                    self._add_safe_edge(aisle_nodes[0][0], south_junc_id, aisle_nodes[0][1], south_junc_pos)

            # Connect north junction to last aisle node
            if aisle_nodes and north_junc_id in self.graph:
                if not a.one_way or a.direction != "forward":
                    self._add_safe_edge(north_junc_id, aisle_nodes[-1][0], north_junc_pos, aisle_nodes[-1][1])
                if not a.one_way or a.direction != "backward":
                    self._add_safe_edge(aisle_nodes[-1][0], north_junc_id, aisle_nodes[-1][1], north_junc_pos)

            # Connect internal aisle nodes respecting one-way rules
            for i in range(len(aisle_nodes) - 1):
                u_id, u_pos = aisle_nodes[i]
                v_id, v_pos = aisle_nodes[i + 1]

                if a.one_way:
                    if a.direction == "forward":
                        # South -> North only
                        self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                    elif a.direction == "backward":
                        # North -> South only
                        self._add_safe_edge(v_id, u_id, v_pos, u_pos)
                else:
                    # Bidirectional
                    self._add_safe_edge(u_id, v_id, u_pos, v_pos)
                    self._add_safe_edge(v_id, u_id, v_pos, u_pos)

        # 3. Connect Arterial Junctions Horizontally along South and North borders
        # This allows cross-aisle transit
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

        # 4. Connect Takeoff & Landing Pads to nearest arterial or aisle nodes
        for pad in self.world.pads:
            pad_node_id = pad.id if pad.id.startswith("PAD-") else f"PAD-{pad.id}"
            pad_pos = pad.position
            self.graph.add_node(pad_node_id, pos=pad_pos, type="pad", pad_id=pad.id)

            # Connect to closest graph nodes
            candidates = [
                n for n in self.graph.nodes
                if n != pad_node_id and self.graph.nodes[n].get("type") in ["arterial", "aisle"]
            ]
            candidates.sort(key=lambda n: math.dist(pad_pos, self.graph.nodes[n]["pos"]))

            connected = 0
            for cand in candidates[:4]:
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
        self.graph.add_node(node_id, pos=pos, type="scan_pose", bin_id=bin_obj.id)

        # Find closest nodes along the same aisle
        aisle_nodes = [
            n for n in self.graph.nodes
            if self.graph.nodes[n].get("aisle_id") == bin_obj.aisle_id
        ]
        if not aisle_nodes:
            # Fallback to any node in graph
            aisle_nodes = [n for n in self.graph.nodes if n != node_id]

        aisle_nodes.sort(key=lambda n: math.dist(pos, self.graph.nodes[n]["pos"]))

        connected = 0
        for cand in aisle_nodes[:3]:
            cand_pos = self.graph.nodes[cand]["pos"]
            if self._add_safe_edge(cand, node_id, cand_pos, pos):
                self._add_safe_edge(node_id, cand, pos, cand_pos)
                connected += 1
            if connected >= 1:
                break

        return node_id

    def get_pad_node(self, pad_id: str) -> str | None:
        if pad_id in self.graph:
            return pad_id
        name = f"PAD-{pad_id}"
        if name in self.graph:
            return name
        return None
