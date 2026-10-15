"""Central safety supervisor monitoring geofence, clearance, battery, and fail-safes."""

import math
from typing import Any

from drone_nav.config.models import (
    LinkLossBehaviour,
    LocalisationLossBehaviour,
    WarehouseConfig,
)
from drone_nav.safety.clearance import RackClearanceMonitor
from drone_nav.safety.geofence import GeofenceMonitor
from drone_nav.safety.models import (
    FailsafeAction,
    SafetyAlertLevel,
    SafetyEvent,
    SafetyViolationType,
)
from drone_nav.world.models import World


class SafetySupervisor:
    """Real-time supervisor evaluating drone state against safety rules and triggers."""

    def __init__(self, world: World, config: WarehouseConfig) -> None:
        self.world = world
        self.config = config
        self.safety_cfg = config.safety
        self.bat_cfg = config.drone.battery

        self.geofence = GeofenceMonitor(world, config)
        self.clearance = RackClearanceMonitor(world, config)

        self.events: list[SafetyEvent] = []
        self.emergency_stop_active: bool = False
        self.link_loss_active: bool = False
        self.localisation_loss_active: bool = False
        self.watchdog_active: bool = False
        self.watchdog_last_heartbeat_s: float = 0.0

        # State tracking
        self.active_failsafe: FailsafeAction = FailsafeAction.NONE

    def record_event(
        self,
        timestamp_s: float,
        alert_level: SafetyAlertLevel,
        violation_type: SafetyViolationType,
        message: str,
        action: FailsafeAction,
        drone_pos: tuple[float, float, float],
        extra: dict[str, Any] | None = None,
    ) -> SafetyEvent:
        evt = SafetyEvent(
            timestamp_s=round(timestamp_s, 3),
            alert_level=alert_level,
            violation_type=violation_type,
            message=message,
            action_taken=action,
            drone_position=(round(drone_pos[0], 3), round(drone_pos[1], 3), round(drone_pos[2], 3)),
            extra_data=extra or {},
        )
        self.events.append(evt)
        return evt

    def trigger_emergency_stop(self, timestamp_s: float, drone_pos: tuple[float, float, float]) -> None:
        self.emergency_stop_active = True
        self.active_failsafe = FailsafeAction.EMERGENCY_STOP
        self.record_event(
            timestamp_s=timestamp_s,
            alert_level=SafetyAlertLevel.EMERGENCY,
            violation_type=SafetyViolationType.EMERGENCY_STOP,
            message="Manual Emergency Stop command received",
            action=FailsafeAction.EMERGENCY_STOP,
            drone_pos=drone_pos,
        )

    def trigger_link_loss(
        self, timestamp_s: float, drone_pos: tuple[float, float, float], is_lost: bool = True
    ) -> None:
        self.link_loss_active = is_lost
        if is_lost:
            beh = self.safety_cfg.link_loss_behaviour
            if beh == LinkLossBehaviour.RETURN_HOME:
                action = FailsafeAction.RETURN_HOME
            elif beh == LinkLossBehaviour.EMERGENCY_STOP:
                action = FailsafeAction.EMERGENCY_STOP
            else:
                action = FailsafeAction.HOLD_HOVER
            self.active_failsafe = action
            self.record_event(
                timestamp_s=timestamp_s,
                alert_level=SafetyAlertLevel.CRITICAL,
                violation_type=SafetyViolationType.TELEMETRY_LINK_LOSS,
                message=f"Ground telemetry link lost. Executing {beh.value}",
                action=action,
                drone_pos=drone_pos,
            )
        elif self.active_failsafe in (FailsafeAction.HOLD_HOVER, FailsafeAction.RETURN_HOME):
            self.active_failsafe = FailsafeAction.NONE

    def trigger_localisation_loss(
        self, timestamp_s: float, drone_pos: tuple[float, float, float], is_lost: bool = True
    ) -> None:
        self.localisation_loss_active = is_lost
        if is_lost:
            beh = self.safety_cfg.localisation_loss_behaviour
            action = (
                FailsafeAction.EMERGENCY_LAND
                if beh == LocalisationLossBehaviour.HOVER_THEN_LAND
                else FailsafeAction.EMERGENCY_STOP
            )
            self.active_failsafe = action
            self.record_event(
                timestamp_s=timestamp_s,
                alert_level=SafetyAlertLevel.CRITICAL,
                violation_type=SafetyViolationType.LOCALISATION_LOSS,
                message=f"Localisation covariance exceeded limit. Executing {beh.value}",
                action=action,
                drone_pos=drone_pos,
            )

    def trigger_watchdog_timeout(
        self, timestamp_s: float, drone_pos: tuple[float, float, float]
    ) -> None:
        self.watchdog_active = True
        self.active_failsafe = FailsafeAction.WATCHDOG_FAILSAFE
        self.record_event(
            timestamp_s=timestamp_s,
            alert_level=SafetyAlertLevel.EMERGENCY,
            violation_type=SafetyViolationType.WATCHDOG_TIMEOUT,
            message="Control loop thread failure. Hardware watchdog backup activated",
            action=FailsafeAction.WATCHDOG_FAILSAFE,
            drone_pos=drone_pos,
        )

    def evaluate_step(
        self,
        timestamp_s: float,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        battery_pct: float,
    ) -> tuple[FailsafeAction, tuple[float, float, float], float | None]:
        """Evaluate safety constraints at a simulation timestep.

        Returns:
            failsafe_action: Active safety response
            safe_velocity: Velocity vector after clamping / repelling
            speed_limit: Max allowed speed override (e.g. in people zones)
        """
        vx, vy, vz = velocity
        speed_limit_override: float | None = None

        # 1. Emergency Stop Check
        if self.emergency_stop_active:
            decel = self.safety_cfg.emergency_stop_decel_mps2
            cur_speed = math.hypot(vx, vy, vz)
            if cur_speed > 0.05:
                factor = max(0.0, 1.0 - (decel * 0.05) / cur_speed)
                return FailsafeAction.EMERGENCY_STOP, (vx * factor, vy * factor, max(-0.5, vz * factor)), None
            return FailsafeAction.EMERGENCY_STOP, (0.0, 0.0, -0.3), None

        # 2. Watchdog Failsafe Check
        if self.watchdog_active:
            # Rotor cut / gentle descent
            return FailsafeAction.WATCHDOG_FAILSAFE, (0.0, 0.0, -0.6), None

        # 3. Critical Battery Emergency Land
        if battery_pct <= self.bat_cfg.critical_reserve_pct:
            if self.active_failsafe != FailsafeAction.EMERGENCY_LAND:
                self.active_failsafe = FailsafeAction.EMERGENCY_LAND
                self.record_event(
                    timestamp_s=timestamp_s,
                    alert_level=SafetyAlertLevel.EMERGENCY,
                    violation_type=SafetyViolationType.BATTERY_CRITICAL_LAND,
                    message=f"Critical battery reserve breached ({battery_pct:.1f}% <= {self.bat_cfg.critical_reserve_pct:.1f}%)",
                    action=FailsafeAction.EMERGENCY_LAND,
                    drone_pos=position,
                )
            return FailsafeAction.EMERGENCY_LAND, (0.0, 0.0, -0.5), 0.5

        # 4. Safe Battery Reserve Return to Launch
        if battery_pct <= self.safety_cfg.battery_reserve_pct:
            if self.active_failsafe != FailsafeAction.RETURN_HOME and self.active_failsafe != FailsafeAction.EMERGENCY_LAND:
                self.active_failsafe = FailsafeAction.RETURN_HOME
                self.record_event(
                    timestamp_s=timestamp_s,
                    alert_level=SafetyAlertLevel.WARNING,
                    violation_type=SafetyViolationType.BATTERY_LOW_RTL,
                    message=f"Safe battery reserve reached ({battery_pct:.1f}% <= {self.safety_cfg.battery_reserve_pct:.1f}%). Triggering Return-To-Home",
                    action=FailsafeAction.RETURN_HOME,
                    drone_pos=position,
                )
            return FailsafeAction.RETURN_HOME, (vx, vy, vz), None

        # 5. Geofence & Boundary Enforcement
        in_bounds, breach_msg = self.geofence.check_outer_perimeter(position)
        if not in_bounds:
            vx, vy, vz = self.geofence.clamp_velocity_at_boundary(position, (vx, vy, vz))
            self.record_event(
                timestamp_s=timestamp_s,
                alert_level=SafetyAlertLevel.CRITICAL,
                violation_type=SafetyViolationType.GEOFENCE_BREACH,
                message=str(breach_msg),
                action=FailsafeAction.CLAMP_SPEED,
                drone_pos=position,
            )

        # 6. No-Fly Zone Containment
        in_nfz, nfz_msg = self.geofence.check_no_fly_zones(position)
        if not in_nfz:
            self.record_event(
                timestamp_s=timestamp_s,
                alert_level=SafetyAlertLevel.CRITICAL,
                violation_type=SafetyViolationType.NO_FLY_ZONE_BREACH,
                message=str(nfz_msg),
                action=FailsafeAction.HOLD_HOVER,
                drone_pos=position,
            )
            return FailsafeAction.HOLD_HOVER, (0.0, 0.0, 0.0), 0.0

        # 7. People Zone Speed Limit Enforcement
        in_pz, pz_limit, pz_name = self.geofence.check_people_zones(position)
        if in_pz and pz_limit is not None:
            speed_limit_override = pz_limit
            cur_speed = math.hypot(vx, vy)
            if cur_speed > pz_limit:
                factor = pz_limit / cur_speed
                vx *= factor
                vy *= factor
                self.record_event(
                    timestamp_s=timestamp_s,
                    alert_level=SafetyAlertLevel.WARNING,
                    violation_type=SafetyViolationType.PEOPLE_ZONE_OVERSPEED,
                    message=f"Exceeded speed in people zone '{pz_name}': clamped to {pz_limit:.1f}m/s",
                    action=FailsafeAction.CLAMP_SPEED,
                    drone_pos=position,
                )

        # 8. Minimum Rack Clearance Monitoring
        clear, dist, rack_id, repelling_normal = self.clearance.check_rack_clearance(position)
        if not clear:
            # Apply repulsive nudge away from rack face
            push_speed = 0.5 * (self.safety_cfg.min_rack_clearance_m - dist)
            vx += repelling_normal[0] * push_speed
            vy += repelling_normal[1] * push_speed
            self.record_event(
                timestamp_s=timestamp_s,
                alert_level=SafetyAlertLevel.WARNING,
                violation_type=SafetyViolationType.RACK_CLEARANCE_BREACH,
                message=f"Rack proximity warning ({dist:.2f}m < {self.safety_cfg.min_rack_clearance_m:.2f}m from {rack_id})",
                action=FailsafeAction.CLAMP_SPEED,
                drone_pos=position,
            )

        return self.active_failsafe, (vx, vy, vz), speed_limit_override
