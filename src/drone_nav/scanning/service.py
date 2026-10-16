"""Mission scanning simulation service combining kinematics, optics, and decoding."""

from pathlib import Path

from drone_nav.config.models import WarehouseConfig
from drone_nav.planner.models import MissionPlan, WaypointAction
from drone_nav.scanning.camera import CameraScanner
from drone_nav.scanning.reader import LabelReader
from drone_nav.scanning.tracker import CoverageTracker
from drone_nav.simulator.models import DroneState, SimulationResult
from drone_nav.simulator.sim import FlightSimulator
from drone_nav.world.models import World


class ScanningMissionSimulator:
    """Orchestrates flight simulation, optical camera evaluation, and label decoding."""

    def __init__(
        self,
        world: World,
        config: WarehouseConfig,
        seed: int = 42,
        wind_speed: float = 0.2,
        image_repo_path: Path | str = "Amazonbindata/img",
    ) -> None:
        self.world = world
        self.config = config
        self.seed = seed
        self.wind_speed = wind_speed
        self.scanner = CameraScanner(config)
        self.reader = LabelReader(seed=seed, image_repo_path=image_repo_path)

    def simulate_mission_scanning(
        self,
        mission_plan: MissionPlan,
        flight_ids: list[int] | None = None,
        start_time_iso: str | None = None,
    ) -> tuple[CoverageTracker, list[SimulationResult]]:
        """Simulate drone flight and optical scanning across specified flights."""
        tracker = CoverageTracker(self.world)
        sim_results: list[SimulationResult] = []

        target_flights = (
            [f for f in mission_plan.flights if f.flight_id in flight_ids]
            if flight_ids is not None
            else mission_plan.flights
        )

        bin_counter = 0

        for flight in target_flights:
            # Kinematic flight simulation with deterministic seed
            flight_sim = FlightSimulator(
                world=self.world,
                config=self.config,
                dt=0.05,
                seed=self.seed + flight.flight_id,
                wind_speed=self.wind_speed,
            )
            sim_res = flight_sim.simulate_flight(flight, mission_id=mission_plan.mission_id)
            sim_results.append(sim_res)

            # Map states by target_bin_id for fast lookup
            dwell_states_by_bin: dict[str, DroneState] = {}
            for state in sim_res.states:
                if state.action == "scan_dwell" and state.target_bin_id:
                    dwell_states_by_bin[state.target_bin_id] = state

            # Evaluate scans at each scan waypoint
            for wp_idx, wp in enumerate(flight.waypoints):
                if wp.action == WaypointAction.SCAN and wp.target_bin_id:
                    target_bin = self.world.bins.get(wp.target_bin_id)
                    if not target_bin:
                        continue

                    cand_state = dwell_states_by_bin.get(wp.target_bin_id)
                    if cand_state is not None:
                        state = cand_state
                    else:
                        matching_states = [s for s in sim_res.states if s.waypoint_index == wp_idx]
                        if matching_states:
                            state = matching_states[0]
                        elif sim_res.states:
                            state = sim_res.states[0]
                        else:
                            continue

                    drone_pos = state.position
                    drone_yaw = state.attitude_rpy[2]
                    drone_speed = state.speed_mps

                    is_visible, q_score, _factors = self.scanner.compute_scan_quality(
                        drone_pos=drone_pos,
                        drone_yaw=drone_yaw,
                        drone_speed=drone_speed,
                        target_bin=target_bin,
                    )

                    # AI Defect & Inventory Discrepancy Detection Simulation
                    expected_qty = target_bin.amazon_expected_qty if target_bin.amazon_expected_qty > 0 else 1
                    discrepancy_type = "none"
                    discrepancy_severity = "none"
                    detected_qty = expected_qty
                    defect_label = "Verified Stock Match"
                    defect_conf = 0.96

                    # Realistic anomaly injection on subset of bins
                    if target_bin.coil_img_id:
                        if bin_counter % 9 == 4:
                            discrepancy_type = "surface_defect"
                            discrepancy_severity = "medium"
                            defect_label = "Surface Oxidation: Minor Coil Edge Corrosion"
                            defect_conf = 0.91
                    else:
                        if bin_counter % 14 == 3:
                            discrepancy_type = "qty_mismatch"
                            discrepancy_severity = "medium"
                            detected_qty = max(0, expected_qty - 1)
                            defect_label = f"Quantity Shortage: Exp {expected_qty}, CV Count {detected_qty}"
                            defect_conf = 0.93
                        elif bin_counter % 17 == 5:
                            discrepancy_type = "package_damage"
                            discrepancy_severity = "high"
                            defect_label = "Carton Defect: Crushed Side / Torn Barcode"
                            defect_conf = 0.89
                        elif bin_counter % 23 == 7:
                            discrepancy_type = "empty_bin"
                            discrepancy_severity = "high"
                            detected_qty = 0
                            defect_label = "Empty Slot Alert: WMS expects active inventory"
                            defect_conf = 0.97

                    if is_visible:
                        read_result = self.reader.decode_label(
                            target_bin=target_bin,
                            quality_score=q_score,
                            bin_index=bin_counter,
                        )
                        bin_counter += 1

                        tracker.record_scan_event(
                            bin_id=target_bin.id,
                            read_success=read_result.success,
                            confidence=read_result.confidence,
                            read_type=read_result.read_type,
                            symbology=read_result.symbology,
                            decoded_value=read_result.value,
                            image_ref=read_result.image_ref,
                            bbox=read_result.bbox,
                            flight_id=flight.flight_id,
                            drone_pos=drone_pos,
                            timestamp_s=state.timestamp_s,
                            start_time_iso=start_time_iso,
                            discrepancy_type=discrepancy_type,
                            discrepancy_severity=discrepancy_severity,
                            expected_qty=expected_qty,
                            detected_qty=detected_qty,
                            defect_label=defect_label,
                            defect_confidence=defect_conf,
                        )
                    else:
                        bin_counter += 1
                        tracker.record_scan_event(
                            bin_id=target_bin.id,
                            read_success=False,
                            confidence=0.0,
                            read_type="BARCODE",
                            symbology="CODE128",
                            decoded_value=None,
                            image_ref=None,
                            bbox={"x": 0, "y": 0, "w": 0, "h": 0},
                            flight_id=flight.flight_id,
                            drone_pos=drone_pos,
                            timestamp_s=state.timestamp_s,
                            start_time_iso=start_time_iso,
                            discrepancy_type=discrepancy_type,
                            discrepancy_severity=discrepancy_severity,
                            expected_qty=expected_qty,
                            detected_qty=detected_qty,
                            defect_label=defect_label,
                            defect_confidence=defect_conf,
                        )

        return tracker, sim_results
