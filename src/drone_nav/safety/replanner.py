"""Dynamic and static obstacle collision detection and online path replanner."""

import math

from drone_nav.planner.astar import astar_path
from drone_nav.planner.graph import NavigationGraph


class ObstacleReplanner:
    """Detects flight path conflicts and replans collision-free paths around obstacles."""

    def __init__(self, nav_graph: NavigationGraph) -> None:
        self.nav_graph = nav_graph

    def block_aisle(self, aisle_id: str) -> list[str]:
        """Remove all navigation nodes belonging to an aisle from routing graph."""
        blocked_nodes: list[str] = []
        for node in list(self.nav_graph.graph.nodes):
            if self.nav_graph.graph.nodes[node].get("aisle_id") == aisle_id:
                blocked_nodes.append(node)
                self.nav_graph.graph.remove_node(node)
        return blocked_nodes

    def block_box_region(
        self,
        box_min: tuple[float, float, float],
        box_max: tuple[float, float, float],
    ) -> list[str]:
        """Disable navigation graph nodes that fall inside a 3D obstacle volume."""
        x1, y1, z1 = box_min
        x2, y2, z2 = box_max
        blocked: list[str] = []

        for node in list(self.nav_graph.graph.nodes):
            pos = self.nav_graph.graph.nodes[node].get("pos")
            if pos:
                px, py, pz = pos
                if x1 <= px <= x2 and y1 <= py <= y2 and z1 <= pz <= z2:
                    blocked.append(node)
                    self.nav_graph.graph.remove_node(node)
        return blocked

    def replan_path(self, start_node: str, goal_node: str) -> list[str] | None:
        """Find new collision-free path on the modified graph using A*."""
        return astar_path(self.nav_graph.graph, start_node, goal_node)

    @staticmethod
    def interpolate_dynamic_obstacle_position(
        path: list[list[float]],
        speed_mps: float,
        schedule_start_s: float,
        current_time_s: float,
    ) -> tuple[float, float, float] | None:
        """Calculate position of dynamic obstacle at given time along its route."""
        if current_time_s < schedule_start_s:
            return None

        elapsed = current_time_s - schedule_start_s
        total_dist_traveled = elapsed * speed_mps

        accum_dist = 0.0
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            seg_dist = math.dist(p1, p2)
            if seg_dist == 0.0:
                continue

            if accum_dist + seg_dist >= total_dist_traveled:
                u = (total_dist_traveled - accum_dist) / seg_dist
                x = p1[0] + u * (p2[0] - p1[0])
                y = p1[1] + u * (p2[1] - p1[1])
                z = p1[2] + u * (p2[2] - p1[2])
                return (x, y, z)
            accum_dist += seg_dist

        # Passed end of path; stay at final waypoint
        last = path[-1]
        return (last[0], last[1], last[2])

    def detect_dynamic_conflict(
        self,
        drone_pos: tuple[float, float, float],
        dynamic_obs_path: list[list[float]],
        dynamic_obs_speed: float,
        start_time_s: float,
        current_time_s: float,
        conflict_radius_m: float = 2.0,
    ) -> tuple[bool, float, tuple[float, float, float] | None]:
        """Detect if drone is in conflict with a moving hazard.

        Returns:
            is_conflict: True if within conflict_radius_m
            distance: Current separation distance
            obs_pos: 3D position of obstacle at current time
        """
        obs_pos = self.interpolate_dynamic_obstacle_position(
            dynamic_obs_path, dynamic_obs_speed, start_time_s, current_time_s
        )
        if obs_pos is None:
            return False, float("inf"), None

        dist = math.dist(drone_pos, obs_pos)
        is_conflict = dist < conflict_radius_m
        return is_conflict, dist, obs_pos
