"""Mission Planner engine combining graph routing, scan strategies, and battery splitting."""

import math
import uuid

import networkx as nx

from drone_nav.config.models import WarehouseConfig
from drone_nav.planner.astar import astar_path, calculate_path_length
from drone_nav.planner.graph import NavigationGraph
from drone_nav.planner.models import (
    FlightPlan,
    MissionPlan,
    StrategyComparisonRow,
    Waypoint,
    WaypointAction,
)
from drone_nav.planner.strategies import (
    MinimumTimeStrategy,
    NaiveStrategy,
    NearestNeighborStrategy,
    ScanOrderStrategy,
    SerpentineStrategy,
)
from drone_nav.world.models import Bin, World


class MissionPlanner:
    """Plans multi-flight collision-free warehouse drone missions with battery management."""

    def __init__(self, world: World, config: WarehouseConfig) -> None:
        self.world = world
        self.config = config
        self.nav_graph = NavigationGraph(world, config)
        self.strategies: dict[str, ScanOrderStrategy] = {
            "naive": NaiveStrategy(),
            "serpentine": SerpentineStrategy(),
            "nearest": NearestNeighborStrategy(),
            "minimum_time": MinimumTimeStrategy(),
        }

    def _estimate_energy_wh(self, distance_m: float, duration_s: float) -> float:
        """Estimate energy consumed based on battery model."""
        bat = self.config.drone.battery
        # Power = hover_power + forward component
        speed = self.config.drone.max_speed_mps * 0.7  # cruise speed
        p_transit = bat.hover_power_w + bat.forward_power_w_per_mps * speed
        return (p_transit * duration_s) / 3600.0

    def _get_target_bins(self) -> list[Bin]:
        """Filter bins based on mission config targets."""
        target_zones = set(self.config.mission.target_zones)
        target_ids = set(self.config.mission.target_bins)

        bins: list[Bin] = []
        for b in self.world.bins.values():
            if target_ids and b.id not in target_ids and b.label_code not in target_ids:
                continue
            if target_zones and b.zone_id not in target_zones:
                continue
            bins.append(b)

        return bins

    def plan_mission(self, strategy_name: str = "serpentine") -> MissionPlan:
        """Plan a complete warehouse scanning mission using the specified strategy."""
        strat = self.strategies.get(strategy_name, self.strategies["serpentine"])
        target_bins = self._get_target_bins()
        if not target_bins:
            if not self.config.mission.target_zones and not self.config.mission.target_bins:
                # No targeting filters configured: fall back to all bins in world
                target_bins = list(self.world.bins.values())
            else:
                raise ValueError(
                    "No bins match the configured mission target_bins/target_zones"
                )

        home_pad = self.world.pads[0] if self.world.pads else None
        if not home_pad:
            raise ValueError("No takeoff/landing pads defined in world layout")

        home_pad_node = self.nav_graph.get_pad_node(home_pad.id)
        if not home_pad_node:
            raise ValueError(f"Home pad {home_pad.id} could not be connected to navigation graph")

        # Connect candidate bins and check reachability
        # Fast reachability: compute forward and reverse reachable sets from home pad once
        fwd_reachable = nx.descendants(self.nav_graph.graph, home_pad_node) | {home_pad_node}
        rev_reachable = nx.ancestors(self.nav_graph.graph, home_pad_node) | {home_pad_node}

        reachable_bins: list[Bin] = []
        unreachable_bins: list[str] = []

        for b in target_bins:
            scan_node = self.nav_graph.connect_scan_pose(b)
            can_reach = any(n in fwd_reachable for n in self.nav_graph.graph.predecessors(scan_node))
            can_return = any(n in rev_reachable for n in self.nav_graph.graph.successors(scan_node))
            if can_reach and can_return:
                reachable_bins.append(b)
            else:
                unreachable_bins.append(b.id)

        # Order reachable bins using strategy
        ordered_bins = strat.order_bins(self.world, reachable_bins)

        # Battery limits
        bat_cfg = self.config.drone.battery
        usable_ratio = 1.0 - (bat_cfg.safe_reserve_pct / 100.0)
        max_energy_wh = bat_cfg.capacity_wh * usable_ratio
        cruise_speed = min(self.config.drone.max_speed_mps, 2.5)
        scan_dwell_time_s = 0.6  # dwell time at scan pose

        # Flights container
        flights: list[FlightPlan] = []
        current_flight_id = 1
        current_flight = FlightPlan(
            flight_id=current_flight_id,
            departure_pad_id=home_pad.id,
            arrival_pad_id=home_pad.id,
        )

        current_node = home_pad_node

        # Takeoff waypoint
        takeoff_wp = Waypoint(
            id=f"F{current_flight_id}-WP-0",
            position=home_pad.position,
            yaw_rad=0.0,
            speed_mps=1.0,
            action=WaypointAction.TAKEOFF,
        )
        current_flight.waypoints.append(takeoff_wp)

        flight_energy_wh = 0.0
        flight_distance_m = 0.0
        flight_duration_s = 5.0  # Takeoff time

        for bin_obj in ordered_bins:
            scan_node = f"SCAN-{bin_obj.id}"

            # Path to scan node
            transit_path = astar_path(self.nav_graph.graph, current_node, scan_node)
            if not transit_path:
                unreachable_bins.append(bin_obj.id)
                continue

            # Return path from scan node to home pad
            return_path = astar_path(self.nav_graph.graph, scan_node, home_pad_node)
            if not return_path:
                unreachable_bins.append(bin_obj.id)
                continue

            transit_dist = calculate_path_length(self.nav_graph.graph, transit_path)
            return_dist = calculate_path_length(self.nav_graph.graph, return_path)

            transit_time = transit_dist / cruise_speed
            return_time = return_dist / cruise_speed

            transit_energy = self._estimate_energy_wh(transit_dist, transit_time)
            scan_energy = (bat_cfg.hover_power_w * scan_dwell_time_s) / 3600.0
            return_energy = self._estimate_energy_wh(return_dist, return_time)

            projected_total_energy = flight_energy_wh + transit_energy + scan_energy + return_energy

            # Check if this bin causes energy to breach safe reserve
            if projected_total_energy > max_energy_wh and len(current_flight.scanned_bin_ids) > 0:
                # Return current flight to pad
                rt_path = astar_path(self.nav_graph.graph, current_node, home_pad_node)
                if rt_path:
                    rt_dist = calculate_path_length(self.nav_graph.graph, rt_path)
                    rt_time = rt_dist / cruise_speed
                    flight_distance_m += rt_dist
                    flight_duration_s += rt_time
                    flight_energy_wh += self._estimate_energy_wh(rt_dist, rt_time)

                    for n_id in rt_path[1:]:
                        n_pos = self.nav_graph.graph.nodes[n_id]["pos"]
                        current_flight.waypoints.append(
                            Waypoint(
                                id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                                position=n_pos,
                                yaw_rad=0.0,
                                speed_mps=cruise_speed,
                                action=WaypointAction.RETURN,
                            )
                        )

                # Landing waypoint
                current_flight.waypoints.append(
                    Waypoint(
                        id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                        position=home_pad.position,
                        yaw_rad=0.0,
                        speed_mps=1.0,
                        action=WaypointAction.LAND,
                    )
                )

                current_flight.total_distance_m = flight_distance_m
                current_flight.estimated_duration_s = flight_duration_s
                current_flight.estimated_energy_wh = flight_energy_wh
                flights.append(current_flight)

                # Start new sortie
                current_flight_id += 1
                current_flight = FlightPlan(
                    flight_id=current_flight_id,
                    departure_pad_id=home_pad.id,
                    arrival_pad_id=home_pad.id,
                )
                current_node = home_pad_node
                flight_energy_wh = 0.0
                flight_distance_m = 0.0
                flight_duration_s = 5.0

                current_flight.waypoints.append(
                    Waypoint(
                        id=f"F{current_flight_id}-WP-0",
                        position=home_pad.position,
                        yaw_rad=0.0,
                        speed_mps=1.0,
                        action=WaypointAction.TAKEOFF,
                    )
                )
                # Re-compute path from pad
                transit_path = astar_path(self.nav_graph.graph, current_node, scan_node)
                if not transit_path:
                    continue
                transit_dist = calculate_path_length(self.nav_graph.graph, transit_path)
                transit_time = transit_dist / cruise_speed
                transit_energy = self._estimate_energy_wh(transit_dist, transit_time)

            # Append transit path waypoints
            for n_id in transit_path[1:-1]:
                n_pos = self.nav_graph.graph.nodes[n_id]["pos"]
                current_flight.waypoints.append(
                    Waypoint(
                        id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                        position=n_pos,
                        yaw_rad=0.0,
                        speed_mps=cruise_speed,
                        action=WaypointAction.FLY,
                    )
                )

            # Compute scan yaw facing bin face
            norm = bin_obj.scan_face_normal
            # Drone camera points toward bin face: normal vector points into aisle, so camera aims -normal
            target_yaw = math.atan2(-norm[1], -norm[0])

            # Scan pose waypoint
            scan_pos = bin_obj.recommended_scan_pos
            current_flight.waypoints.append(
                Waypoint(
                    id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                    position=scan_pos,
                    yaw_rad=target_yaw,
                    speed_mps=cruise_speed,
                    action=WaypointAction.SCAN,
                    target_bin_id=bin_obj.id,
                    wait_time_s=scan_dwell_time_s,
                )
            )

            current_flight.scanned_bin_ids.append(bin_obj.id)
            current_node = scan_node
            flight_distance_m += transit_dist
            flight_duration_s += transit_time + scan_dwell_time_s
            flight_energy_wh += transit_energy + scan_energy

        # Return final flight to home pad
        rt_path = astar_path(self.nav_graph.graph, current_node, home_pad_node)
        if rt_path:
            rt_dist = calculate_path_length(self.nav_graph.graph, rt_path)
            rt_time = rt_dist / cruise_speed
            flight_distance_m += rt_dist
            flight_duration_s += rt_time
            flight_energy_wh += self._estimate_energy_wh(rt_dist, rt_time)

            for n_id in rt_path[1:]:
                n_pos = self.nav_graph.graph.nodes[n_id]["pos"]
                current_flight.waypoints.append(
                    Waypoint(
                        id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                        position=n_pos,
                        yaw_rad=0.0,
                        speed_mps=cruise_speed,
                        action=WaypointAction.RETURN,
                    )
                )

        current_flight.waypoints.append(
            Waypoint(
                id=f"F{current_flight_id}-WP-{len(current_flight.waypoints)}",
                position=home_pad.position,
                yaw_rad=0.0,
                speed_mps=1.0,
                action=WaypointAction.LAND,
            )
        )

        current_flight.total_distance_m = flight_distance_m
        current_flight.estimated_duration_s = flight_duration_s
        current_flight.estimated_energy_wh = flight_energy_wh
        flights.append(current_flight)

        total_scanned = sum(len(f.scanned_bin_ids) for f in flights)

        return MissionPlan(
            mission_id=f"MIS-{uuid.uuid4().hex[:8].upper()}",
            site_id=self.world.site_id,
            strategy=strategy_name,
            flights=flights,
            bins_targeted=len(target_bins),
            bins_scanned=total_scanned,
            unreachable_bins=unreachable_bins,
        )

    def compare_strategies(self) -> list[StrategyComparisonRow]:
        """Compare all available scan sequencing strategies on the current layout."""
        results = []
        for name in ["naive", "serpentine", "nearest", "minimum_time"]:
            plan = self.plan_mission(strategy_name=name)
            avg_dist = plan.total_distance_m / max(1, plan.bins_scanned)
            results.append(
                StrategyComparisonRow(
                    strategy=name,
                    coverage_pct=plan.coverage_pct,
                    total_flights=plan.total_flights,
                    total_distance_m=round(plan.total_distance_m, 2),
                    total_duration_s=round(plan.total_duration_s, 1),
                    total_energy_wh=round(plan.total_energy_wh, 2),
                    avg_dist_per_bin_m=round(avg_dist, 2),
                )
            )
        return results
