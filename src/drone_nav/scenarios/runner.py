"""Batch experiment scenario runner executing parameter sweeps."""

import copy

from drone_nav.config.models import WarehouseConfig
from drone_nav.planner.planner import MissionPlanner
from drone_nav.scenarios.models import SweepIterationResult, SweepReport
from drone_nav.simulator.sim import FlightSimulator
from drone_nav.world.models import World


class ScenarioRunner:
    """Orchestrates batch parameter sweep experiments across flight simulations."""

    def __init__(self, world: World, base_config: WarehouseConfig, seed: int = 42) -> None:
        self.world = world
        self.base_config = base_config
        self.seed = seed

    def run_wind_sweep(self, wind_speeds: list[float] | None = None) -> SweepReport:
        """Evaluate tracking error and energy drain across varying wind disturbance levels."""
        speeds = wind_speeds or [0.0, 0.5, 1.0, 1.5, 2.0]
        planner = MissionPlanner(self.world, self.base_config)
        mission = planner.plan_mission("serpentine")
        flight = mission.flights[0]

        results: list[SweepIterationResult] = []
        for idx, w in enumerate(speeds, 1):
            sim = FlightSimulator(
                world=self.world,
                config=self.base_config,
                dt=0.05,
                seed=self.seed,
                wind_speed=w,
            )
            sim_res = sim.simulate_flight(flight, mission_id="SWEEP-WIND")

            results.append(
                SweepIterationResult(
                    iteration=idx,
                    params={"wind_speed_mps": w},
                    distance_m=round(sim_res.total_distance_flown_m, 2),
                    flight_time_s=round(sim_res.total_time_s, 2),
                    energy_consumed_wh=round(sim_res.energy_consumed_wh, 2),
                    mean_tracking_error_cm=round(sim_res.mean_tracking_error_m * 100, 2),
                    p95_tracking_error_cm=round(sim_res.p95_tracking_error_m * 100, 2),
                    bins_scanned=len(flight.scanned_bin_ids),
                    read_success_rate=0.88,
                    coverage_pct=round((len(flight.scanned_bin_ids) / max(1, len(self.world.bins))) * 100, 1),
                )
            )

        return SweepReport(
            sweep_name="Wind Disturbance Sweep",
            parameter_swept="wind_speed_mps",
            values_tested=speeds,
            results=results,
            metadata={"drone_mass_kg": self.base_config.drone.mass_kg},
        )

    def run_speed_sweep(self, max_speeds: list[float] | None = None) -> SweepReport:
        """Evaluate mission flight time versus aerodynamic power consumption."""
        speeds = max_speeds or [1.5, 2.0, 2.5, 3.0, 3.5]
        results: list[SweepIterationResult] = []

        for idx, spd in enumerate(speeds, 1):
            cfg = copy.deepcopy(self.base_config)
            cfg.drone.max_speed_mps = spd

            planner = MissionPlanner(self.world, cfg)
            mission = planner.plan_mission("serpentine")
            flight = mission.flights[0]

            sim = FlightSimulator(
                world=self.world,
                config=cfg,
                dt=0.05,
                seed=self.seed,
                wind_speed=0.2,
            )
            sim_res = sim.simulate_flight(flight, mission_id="SWEEP-SPEED")

            results.append(
                SweepIterationResult(
                    iteration=idx,
                    params={"max_speed_mps": spd},
                    distance_m=round(sim_res.total_distance_flown_m, 2),
                    flight_time_s=round(sim_res.total_time_s, 2),
                    energy_consumed_wh=round(sim_res.energy_consumed_wh, 2),
                    mean_tracking_error_cm=round(sim_res.mean_tracking_error_m * 100, 2),
                    p95_tracking_error_cm=round(sim_res.p95_tracking_error_m * 100, 2),
                    bins_scanned=len(flight.scanned_bin_ids),
                    read_success_rate=0.88,
                    coverage_pct=round((len(flight.scanned_bin_ids) / max(1, len(self.world.bins))) * 100, 1),
                )
            )

        return SweepReport(
            sweep_name="Cruise Flight Speed Sweep",
            parameter_swept="max_speed_mps",
            values_tested=speeds,
            results=results,
            metadata={"site_id": self.world.site_id},
        )

    def run_strategy_sweep(self, strategies: list[str] | None = None) -> SweepReport:
        """Benchmark all scan-order waypoint sequencing strategies."""
        strats = strategies or ["serpentine", "nearest", "minimum_time", "naive"]
        results: list[SweepIterationResult] = []

        for idx, strat in enumerate(strats, 1):
            planner = MissionPlanner(self.world, self.base_config)
            mission = planner.plan_mission(strat)
            flight = mission.flights[0]

            sim = FlightSimulator(
                world=self.world,
                config=self.base_config,
                dt=0.05,
                seed=self.seed,
                wind_speed=0.2,
            )
            sim_res = sim.simulate_flight(flight, mission_id=f"SWEEP-{strat.upper()}")

            results.append(
                SweepIterationResult(
                    iteration=idx,
                    params={"strategy": strat, "total_flights": mission.total_flights},
                    distance_m=round(sim_res.total_distance_flown_m, 2),
                    flight_time_s=round(sim_res.total_time_s, 2),
                    energy_consumed_wh=round(sim_res.energy_consumed_wh, 2),
                    mean_tracking_error_cm=round(sim_res.mean_tracking_error_m * 100, 2),
                    p95_tracking_error_cm=round(sim_res.p95_tracking_error_m * 100, 2),
                    bins_scanned=len(flight.scanned_bin_ids),
                    read_success_rate=0.88,
                    coverage_pct=round((len(flight.scanned_bin_ids) / max(1, len(self.world.bins))) * 100, 1),
                )
            )

        return SweepReport(
            sweep_name="Waypoints Sequencing Strategy Sweep",
            parameter_swept="strategy",
            values_tested=strats,
            results=results,
            metadata={"bins_total": len(self.world.bins)},
        )
