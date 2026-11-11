"""Experimental onboard flip with guarded staging and automatic recovery."""

from __future__ import annotations

import select
import sys
import time
from argparse import ArgumentParser, Namespace
from collections import deque
from dataclasses import dataclass
from math import hypot, isfinite
from statistics import fmean
from typing import Any

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from rich.live import Live

from bumblebee.errors import SafetyError
from bumblebee.flights.base import FlightContext, stop_and_disarm
from bumblebee.flights.hover import _bounded_float
from bumblebee.models import FlightRequirements
from bumblebee.supervisor import request_arming
from bumblebee.telemetry import TimedTelemetryLogger, assert_supervisor_safe
from bumblebee.ui.flip import build_flip_dashboard
from bumblebee.watchdog import EmergencyStopWatchdog

FLIP_REQUEST_MAGIC = 0xB33F
MINIMUM_FLIP_VOLTAGE = 3.50
STAGING_VOLTAGE_WINDOW_SAMPLES = 5
LOW_VOLTAGE_CONFIRM_SECONDS = 0.5
MAXIMUM_STAGING_DISTANCE = 0.50
MAXIMUM_STAGING_SPEED = 0.12
STABLE_WINDOW_SECONDS = 1.0
STAGING_TIMEOUT_SECONDS = 8.0
MINIMUM_RECOVERY_ALTITUDE = 0.55
MINIMUM_RECOVERY_VERTICAL_SPEED = -0.50

FLIP_STATE_NAMES = {
    0: "Staging",
    1: "Boosting",
    2: "Spinning up",
    3: "Coasting",
    4: "Braking rotation",
    5: "Recovering",
    6: "Verifying recovery",
    7: "Complete",
    8: "Aborted",
}

FLIP_RESULT_MESSAGES = {
    0: "No result was reported.",
    1: "The rotation completed and stable-flight recovery was verified.",
    2: "The firmware did not consider the Crazyflie airborne.",
    3: "The under-load battery voltage was too low to safely begin the flip.",
    4: "The Crazyflie was not level enough to begin.",
    5: "The staging altitude was outside 0.75–1.65 m.",
    6: "The firmware could not learn a stable hover-thrust value.",
    7: "The requested rotation did not complete before its timeout.",
    8: "The supervisor stopped the maneuver.",
    9: "The supervisor denied the time-limited Acro permit.",
    10: "Net rotation was reached, but the Crazyflie was not upright.",
    11: "The rotation completed, but stable flight was not recovered.",
    12: "The installed firmware has disabled this maneuver.",
    13: "The requested phase profile was outside its guarded limits.",
    14: "The rotation could not be slowed near the upright attitude.",
    15: "The upright low-altitude motor cutoff stopped direct control.",
    16: "The time-limited Acro permit expired during direct control.",
    17: "The onboard impact detector stopped direct control.",
}

MOTOR_PWM_MAX = 65_535
CONTROLLED_LANDING_ABORT_RESULTS = frozenset({3, 5, 6, 9, 12, 13})


class PreRotationAbort(SafetyError):
    """A rejected flip request that is still safe to land normally."""


def is_pre_rotation_abort(*, result: int, raw_rotation: float) -> bool:
    return (
        result in CONTROLLED_LANDING_ABORT_RESULTS
        and isfinite(raw_rotation)
        and raw_rotation < 5.0
    )


@dataclass(frozen=True)
class FlipProfile:
    boost_height_mm: int = 100
    boost_duration_ms: int = 150
    brake_start_degrees: int = 140
    spin_thrust_percent: int = 40
    coast_thrust_percent: int = 20
    brake_thrust_percent: int = 70
    spin_torque_percent: int = 85
    brake_torque_percent: int = 95
    recovery_duration_ms: int = 500


PHASED_PROFILE = FlipProfile()


@dataclass(frozen=True)
class FlipSettings:
    height: float
    settle_seconds: float
    direction: str
    profile: FlipProfile = PHASED_PROFILE


@dataclass(frozen=True)
class StagingSnapshot:
    altitude: float
    voltage: float
    horizontal_distance: float
    horizontal_speed: float


@dataclass(frozen=True)
class RecoverySnapshot:
    raw_rotation: float
    recovery_altitude: float
    recovery_vertical_speed: float
    hover_thrust: float
    minimum_altitude: float
    minimum_voltage: float
    boost_gain: float
    brake_start_rotation: float
    brake_end_rate: float
    recovery_start_altitude: float
    rotation_end_roll: float
    rotation_end_pitch: float


@dataclass
class SustainedLowVoltageGuard:
    threshold: float = MINIMUM_FLIP_VOLTAGE
    confirmation_seconds: float = LOW_VOLTAGE_CONFIRM_SECONDS
    below_since: float | None = None

    def update(self, *, voltage: float, now: float) -> bool:
        if voltage >= self.threshold:
            self.below_since = None
            return False
        if self.below_since is None:
            self.below_since = now
            return False
        return now - self.below_since >= self.confirmation_seconds


def staging_violation(
    *,
    horizontal_distance: float,
    horizontal_speed: float,
    elapsed: float,
    settle_seconds: float,
) -> str | None:
    if not all(
        isfinite(value)
        for value in (horizontal_distance, horizontal_speed)
    ):
        return "Horizontal position telemetry became invalid."
    if horizontal_distance > MAXIMUM_STAGING_DISTANCE:
        return (
            f"Bumblebee drifted {horizontal_distance:.2f} m from takeoff; "
            f"the pre-flip limit is {MAXIMUM_STAGING_DISTANCE:.2f} m."
        )
    if elapsed > settle_seconds + STAGING_TIMEOUT_SECONDS:
        return (
            "Bumblebee could not hold a sufficiently slow staging hover "
            f"below {MAXIMUM_STAGING_SPEED:.2f} m/s."
        )
    return None


def recovery_violation(
    *,
    altitude: float,
    vertical_speed: float,
) -> str | None:
    if not all(isfinite(value) for value in (altitude, vertical_speed)):
        return "Recovery telemetry became invalid."
    if altitude < MINIMUM_RECOVERY_ALTITUDE:
        return (
            f"Recovery altitude was {altitude:.2f} m; at least "
            f"{MINIMUM_RECOVERY_ALTITUDE:.2f} m is required."
        )
    if vertical_speed < MINIMUM_RECOVERY_VERTICAL_SPEED:
        return (
            f"Recovery vertical speed was {vertical_speed:.2f} m/s; it must be "
            f"at least {MINIMUM_RECOVERY_VERTICAL_SPEED:.2f} m/s."
        )
    return None


class FlipFlight:
    key = "flip"
    title = "Experimental flip"
    description = "Take off, perform one onboard flip, recover, and land."
    requirements_label = "Flow V2 · Acro firmware"
    requirements = FlightRequirements(
        flow_deck=True,
        minimum_acro_firmware_version=17,
    )
    requires_acro = True
    confirmation_word = "FLIP"

    def add_arguments(self, parser: ArgumentParser) -> None:
        parser.add_argument(
            "--flip-height",
            type=_bounded_float(0.80, 1.50),
            default=1.50,
            help="flip staging height in metres, 0.80-1.50 (default: 1.50)",
        )
        parser.add_argument(
            "--flip-settle",
            type=_bounded_float(2.0, 6.0),
            default=3.0,
            help="seconds to learn hover thrust before flipping (default: 3)",
        )
        parser.add_argument(
            "--flip-direction",
            choices=("backward", "forward"),
            default="backward",
            help="rotation direction (default: backward)",
        )
    def settings_from(self, args: Namespace) -> FlipSettings:
        return FlipSettings(
            height=args.flip_height,
            settle_seconds=args.flip_settle,
            direction=args.flip_direction,
            profile=PHASED_PROFILE,
        )

    def plan(self, settings: FlipSettings) -> str:
        return (
            f"stage at {settings.height:.2f} m, then one direct-torque "
            f"{settings.direction} flip"
        )

    def run(
        self,
        connection: Any,
        settings: FlipSettings,
        context: FlightContext,
    ) -> None:
        commander = MotionCommander(connection, default_height=settings.height)
        dashboard = build_flip_dashboard(
            uri=context.uri,
            phase="Taking off",
            direction=settings.direction,
            target_height=settings.height,
            altitude=None,
            voltage=context.preflight_voltage,
            battery_live=False,
            minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
            rotation=0.0,
            raw_rotation=None,
            peak_pitch_rate=None,
            rotation_end_roll=None,
            rotation_end_pitch=None,
            recovery_altitude=None,
            recovery_vertical_speed=None,
            hover_thrust_ready=False,
            hover_thrust=None,
            watchdog_active=True,
            horizontal_distance=None,
            horizontal_speed=None,
            message=(
                "Ascending to the staging height. Live altitude, drift, and "
                "under-load battery readings begin when takeoff completes…"
            ),
        )

        request_arming(connection, True)
        time.sleep(0.5)
        watchdog_started = False
        controlled_landing_allowed = True
        try:
            with EmergencyStopWatchdog(connection) as watchdog:
                watchdog_started = True
                try:
                    with Live(
                        dashboard,
                        console=context.console,
                        refresh_per_second=10,
                    ) as live:
                        commander.take_off(
                            velocity=context.safety_limits.max_vertical_speed
                        )
                        staging = self._wait_for_stable_stage(
                            connection,
                            settings,
                            context,
                            watchdog,
                            live,
                        )
                        controlled_landing_allowed = False
                        self._trigger(
                            connection,
                            settings.direction,
                            settings.profile,
                        )
                        try:
                            self._monitor_flip(
                                connection,
                                settings,
                                context,
                                watchdog,
                                live,
                                staging,
                            )
                        except PreRotationAbort:
                            controlled_landing_allowed = True
                            raise
                        controlled_landing_allowed = True
                finally:
                    self._finish_flight(
                        commander,
                        connection,
                        context,
                        controlled_landing_allowed=controlled_landing_allowed,
                    )
        finally:
            if not watchdog_started:
                stop_and_disarm(connection, context)
            context.console.print(
                "[yellow]Power-cycle the Crazyflie before another flight.[/yellow] "
                "The Acro radio-loss watchdog intentionally locks after its "
                "heartbeat ends."
            )

    @staticmethod
    def _finish_flight(
        commander: MotionCommander,
        connection: Any,
        context: FlightContext,
        *,
        controlled_landing_allowed: bool,
    ) -> None:
        if not controlled_landing_allowed:
            context.console.print(
                "[bold red]Acro recovery was not confirmed.[/bold red] "
                "Stopping and disarming without another motor command."
            )
            stop_and_disarm(connection, context)
            return

        try:
            commander.land(
                velocity=context.safety_limits.max_vertical_speed
            )
        except Exception as exc:  # noqa: BLE001 - generic cflib errors.
            context.console.print(
                "[bold red]Controlled landing command failed:[/bold red] "
                f"{exc}"
            )
        stop_and_disarm(connection, context)

    def _wait_for_stable_stage(
        self,
        connection: Any,
        settings: FlipSettings,
        context: FlightContext,
        watchdog: EmergencyStopWatchdog,
        live: Live,
    ) -> StagingSnapshot:
        config = LogConfig(name="Bumblebee staging", period_in_ms=50)
        config.add_variable("pm.vbat", "float")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("stateEstimate.x", "float")
        config.add_variable("stateEstimate.y", "float")
        config.add_variable("stateEstimate.z", "float")
        config.add_variable("stateEstimate.vx", "float")
        config.add_variable("stateEstimate.vy", "float")

        started = time.monotonic()
        stable_since: float | None = None
        voltage_history: deque[float] = deque(
            maxlen=STAGING_VOLTAGE_WINDOW_SAMPLES
        )
        voltage_guard = SustainedLowVoltageGuard()
        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            for _timestamp, data, _name in logger:
                watchdog.assert_healthy()
                now = time.monotonic()
                elapsed = now - started
                voltage_history.append(float(data["pm.vbat"]))
                voltage = fmean(voltage_history)
                altitude = float(data["stateEstimate.z"])
                horizontal_distance = hypot(
                    float(data["stateEstimate.x"]),
                    float(data["stateEstimate.y"]),
                )
                horizontal_speed = hypot(
                    float(data["stateEstimate.vx"]),
                    float(data["stateEstimate.vy"]),
                )

                violation = staging_violation(
                    horizontal_distance=horizontal_distance,
                    horizontal_speed=horizontal_speed,
                    elapsed=elapsed,
                    settle_seconds=settings.settle_seconds,
                )
                if violation is not None:
                    raise SafetyError(violation)

                if horizontal_speed <= MAXIMUM_STAGING_SPEED:
                    stable_since = stable_since or now
                else:
                    stable_since = None
                stable_for = 0.0 if stable_since is None else now - stable_since
                learned = elapsed >= settings.settle_seconds
                battery_window_ready = (
                    len(voltage_history) == STAGING_VOLTAGE_WINDOW_SAMPLES
                )
                low_voltage_confirmed = voltage_guard.update(
                    voltage=voltage,
                    now=now,
                )
                battery_ready = (
                    battery_window_ready and voltage >= MINIMUM_FLIP_VOLTAGE
                )
                ready = (
                    learned
                    and stable_for >= STABLE_WINDOW_SECONDS
                    and battery_ready
                )

                if not battery_window_ready:
                    message = "Measuring the battery under motor load…"
                elif voltage < MINIMUM_FLIP_VOLTAGE:
                    message = (
                        f"Under-load battery is {voltage:.3f} V; "
                        f"{MINIMUM_FLIP_VOLTAGE:.2f} V is required. "
                        "Confirming before landing…"
                    )
                elif ready:
                    message = (
                        "Staging is stable. Sending the onboard flip trigger…"
                    )
                else:
                    message = (
                        "Holding at staging height. The flip requires "
                        f"{STABLE_WINDOW_SECONDS:.0f} second below "
                        f"{MAXIMUM_STAGING_SPEED:.2f} m/s. "
                        "Press Return to cancel."
                    )

                live.update(
                    build_flip_dashboard(
                        uri=context.uri,
                        phase="Staging",
                        direction=settings.direction,
                        target_height=settings.height,
                        altitude=altitude,
                        voltage=voltage,
                        battery_live=True,
                        minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
                        rotation=0.0,
                        raw_rotation=None,
                        peak_pitch_rate=None,
                        rotation_end_roll=None,
                        rotation_end_pitch=None,
                        recovery_altitude=None,
                        recovery_vertical_speed=None,
                        hover_thrust_ready=learned,
                        hover_thrust=None,
                        watchdog_active=True,
                        horizontal_distance=horizontal_distance,
                        horizontal_speed=horizontal_speed,
                        message=message,
                    ),
                    refresh=True,
                )

                assert_supervisor_safe(int(data["supervisor.info"]))
                if self._return_pressed():
                    raise SafetyError(
                        "Landing was requested before the flip trigger."
                    )
                if low_voltage_confirmed:
                    raise SafetyError(
                        "Acro cancelled before the flip: battery averaged "
                        f"{voltage:.3f} V under motor load "
                        f"for {LOW_VOLTAGE_CONFIRM_SECONDS:.1f} seconds; "
                        f"at least {MINIMUM_FLIP_VOLTAGE:.2f} V is required "
                        "for the flip. Landing without attempting the maneuver."
                    )
                if violation is not None:
                    raise SafetyError(violation)
                if ready:
                    return StagingSnapshot(
                        altitude=altitude,
                        voltage=voltage,
                        horizontal_distance=horizontal_distance,
                        horizontal_speed=horizontal_speed,
                    )

        raise SafetyError("Staging telemetry ended before the flip trigger.")

    def _monitor_flip(
        self,
        connection: Any,
        settings: FlipSettings,
        context: FlightContext,
        watchdog: EmergencyStopWatchdog,
        live: Live,
        staging: StagingSnapshot,
    ) -> None:
        control_config = LogConfig(name="Bumblebee flip", period_in_ms=50)
        control_config.add_variable("pm.vbat", "float")
        control_config.add_variable("supervisor.info", "uint16_t")
        control_config.add_variable("bumblebee.state", "uint8_t")
        control_config.add_variable("bumblebee.result", "uint8_t")
        control_config.add_variable("bumblebee.rotation", "float")
        control_config.add_variable("bumblebee.peakRate", "float")
        control_config.add_variable("stateEstimate.roll", "float")
        control_config.add_variable("stateEstimate.pitch", "float")

        dynamics_config = LogConfig(
            name="Bumblebee flip dynamics",
            period_in_ms=50,
        )
        dynamics_config.add_variable("stateEstimate.z", "float")
        dynamics_config.add_variable("stateEstimate.vz", "float")
        for motor in range(1, 5):
            dynamics_config.add_variable(f"motor.m{motor}req", "int32_t")

        triggered_at = time.monotonic()
        observed_maneuver = False
        final_result: int | None = None
        voltage = staging.voltage
        state = 0
        rotation = 0.0
        peak_pitch_rate = 0.0
        maximum_tilt = 0.0
        altitude = staging.altitude
        vertical_speed = 0.0
        minimum_altitude = staging.altitude
        minimum_voltage = staging.voltage
        maximum_motor_request = 0
        saturation_samples = 0
        sample_count = 0
        boost_end_altitude: float | None = None
        brake_start_altitude: float | None = None
        recovery_start_altitude: float | None = None
        recovery_start_vertical_speed: float | None = None
        previous_state = 0

        with (
            TimedTelemetryLogger(
                connection,
                control_config,
                context.safety_limits.telemetry_timeout,
            ) as control_logger,
            TimedTelemetryLogger(
                connection,
                dynamics_config,
                context.safety_limits.telemetry_timeout,
            ) as dynamics_logger,
        ):
            for _timestamp, data, _name in control_logger:
                _dynamics_timestamp, dynamics, _dynamics_name = next(
                    dynamics_logger
                )
                watchdog.assert_healthy()
                now = time.monotonic()
                voltage = float(data["pm.vbat"])
                state = int(data["bumblebee.state"])
                result = int(data["bumblebee.result"])
                rotation = float(data["bumblebee.rotation"])
                peak_pitch_rate = float(data["bumblebee.peakRate"])
                live_roll = float(data["stateEstimate.roll"])
                live_pitch = float(data["stateEstimate.pitch"])
                maximum_tilt = max(
                    maximum_tilt,
                    abs(live_roll),
                    abs(live_pitch),
                )
                altitude = float(dynamics["stateEstimate.z"])
                vertical_speed = float(dynamics["stateEstimate.vz"])
                motor_requests = tuple(
                    int(dynamics[f"motor.m{motor}req"])
                    for motor in range(1, 5)
                )
                maximum_motor_request = max(
                    maximum_motor_request,
                    max(motor_requests),
                )
                sample_count += 1
                if max(motor_requests) > MOTOR_PWM_MAX:
                    saturation_samples += 1
                minimum_altitude = min(minimum_altitude, altitude)
                minimum_voltage = min(minimum_voltage, voltage)

                if state != previous_state:
                    if state == 2:
                        boost_end_altitude = altitude
                    elif state == 4:
                        brake_start_altitude = altitude
                    elif state == 5:
                        recovery_start_altitude = altitude
                        recovery_start_vertical_speed = vertical_speed
                    previous_state = state

                message = (
                    "The onboard maneuver is running."
                    if result == 0
                    else FLIP_RESULT_MESSAGES.get(
                        result,
                        "The onboard maneuver is running.",
                    )
                )
                if state not in (0, 7, 8):
                    observed_maneuver = True
                if state in (7, 8):
                    final_result = result

                saturation_percent = (
                    saturation_samples / sample_count * 100.0
                )

                live.update(
                    build_flip_dashboard(
                        uri=context.uri,
                        phase=FLIP_STATE_NAMES.get(state, "Unknown"),
                        direction=settings.direction,
                        target_height=settings.height,
                        altitude=altitude,
                        voltage=voltage,
                        battery_live=True,
                        minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
                        rotation=rotation,
                        raw_rotation=None,
                        peak_pitch_rate=peak_pitch_rate,
                        rotation_end_roll=(
                            live_roll if state in (7, 8) else None
                        ),
                        rotation_end_pitch=(
                            live_pitch if state in (7, 8) else None
                        ),
                        recovery_altitude=None,
                        recovery_vertical_speed=None,
                        hover_thrust_ready=True,
                        hover_thrust=None,
                        watchdog_active=True,
                        horizontal_distance=staging.horizontal_distance,
                        horizontal_speed=staging.horizontal_speed,
                        message=message,
                        current_vertical_speed=vertical_speed,
                        minimum_altitude=minimum_altitude,
                        minimum_voltage=minimum_voltage,
                        maximum_motor_request=maximum_motor_request,
                        motor_saturation_percent=saturation_percent,
                        maximum_tilt=maximum_tilt,
                        boost_end_altitude=boost_end_altitude,
                        brake_start_altitude=brake_start_altitude,
                        recovery_start_altitude=recovery_start_altitude,
                        recovery_start_vertical_speed=(
                            recovery_start_vertical_speed
                        ),
                    ),
                    refresh=True,
                )

                if final_result is not None:
                    break

                assert_supervisor_safe(int(data["supervisor.info"]))

                since_trigger = now - triggered_at
                if not observed_maneuver and since_trigger > 2.0:
                    raise SafetyError(
                        "The onboard firmware did not accept the flip trigger."
                    )
                if since_trigger > 4.0:
                    raise SafetyError(
                        "The onboard flip did not finish within four seconds."
                    )

        if final_result is None:
            raise SafetyError("Flip telemetry ended before a final result arrived.")

        recovery = self._read_recovery_snapshot(connection, context)
        host_recovery_issue = recovery_violation(
            altitude=recovery.recovery_altitude,
            vertical_speed=recovery.recovery_vertical_speed,
        )
        if final_result == 1 and host_recovery_issue is not None:
            final_result = 11
            state = 8
        detail = FLIP_RESULT_MESSAGES.get(
            final_result,
            f"Unknown result code {final_result}.",
        )
        pre_rotation_abort = is_pre_rotation_abort(
            result=final_result,
            raw_rotation=recovery.raw_rotation,
        )
        if host_recovery_issue is not None:
            detail = f"{detail} {host_recovery_issue}"
        live.update(
            build_flip_dashboard(
                uri=context.uri,
                phase=FLIP_STATE_NAMES.get(state, "Unknown"),
                direction=settings.direction,
                target_height=settings.height,
                altitude=recovery.recovery_altitude,
                voltage=voltage,
                battery_live=True,
                minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
                rotation=rotation,
                raw_rotation=recovery.raw_rotation,
                peak_pitch_rate=peak_pitch_rate,
                rotation_end_roll=(
                    None
                    if pre_rotation_abort
                    else recovery.rotation_end_roll
                ),
                rotation_end_pitch=(
                    None
                    if pre_rotation_abort
                    else recovery.rotation_end_pitch
                ),
                recovery_altitude=recovery.recovery_altitude,
                recovery_vertical_speed=recovery.recovery_vertical_speed,
                hover_thrust_ready=True,
                hover_thrust=recovery.hover_thrust,
                watchdog_active=True,
                horizontal_distance=staging.horizontal_distance,
                horizontal_speed=staging.horizontal_speed,
                message=detail,
                current_vertical_speed=recovery.recovery_vertical_speed,
                minimum_altitude=(
                    minimum_altitude
                    if pre_rotation_abort
                    else recovery.minimum_altitude
                ),
                minimum_voltage=(
                    minimum_voltage
                    if pre_rotation_abort
                    else recovery.minimum_voltage
                ),
                maximum_motor_request=maximum_motor_request,
                motor_saturation_percent=(
                    0.0
                    if sample_count == 0
                    else saturation_samples / sample_count * 100.0
                ),
                maximum_tilt=maximum_tilt,
                boost_end_altitude=(
                    None
                    if pre_rotation_abort
                    else staging.altitude + recovery.boost_gain
                ),
                brake_start_altitude=brake_start_altitude,
                recovery_start_altitude=(
                    recovery.recovery_start_altitude
                    if final_result in (1, 11)
                    else recovery_start_altitude
                ),
                recovery_start_vertical_speed=(
                    recovery_start_vertical_speed
                ),
                brake_start_rotation=(
                    None
                    if pre_rotation_abort
                    else recovery.brake_start_rotation
                ),
                brake_end_rate=(
                    None if pre_rotation_abort else recovery.brake_end_rate
                ),
            ),
            refresh=True,
        )

        if final_result != 1:
            if pre_rotation_abort:
                raise PreRotationAbort(f"Flip rejected before rotation: {detail}")
            raise SafetyError(f"Flip aborted: {detail}")
        time.sleep(0.75)

    @staticmethod
    def _read_recovery_snapshot(
        connection: Any,
        context: FlightContext,
    ) -> RecoverySnapshot:
        recovery_config = LogConfig(
            name="Bumblebee recovery result",
            period_in_ms=50,
        )
        recovery_config.add_variable("bumblebee.rawRotation", "float")
        recovery_config.add_variable("bumblebee.hoverThrust", "float")
        recovery_config.add_variable("stateEstimate.z", "float")
        recovery_config.add_variable("stateEstimate.vz", "float")
        recovery_config.add_variable("bumblebee.endRoll", "float")
        recovery_config.add_variable("bumblebee.endPitch", "float")

        with TimedTelemetryLogger(
            connection,
            recovery_config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            _timestamp, data, _name = next(logger)

        profile_config = LogConfig(
            name="Bumblebee phase result",
            period_in_ms=50,
        )
        profile_config.add_variable("bumblebee.minAltitude", "float")
        profile_config.add_variable("bumblebee.minVoltage", "float")
        profile_config.add_variable("bumblebee.boostGain", "float")
        profile_config.add_variable("bumblebee.brakeStart", "float")
        profile_config.add_variable("bumblebee.brakeEndRate", "float")
        profile_config.add_variable("bumblebee.recoveryStart", "float")

        with TimedTelemetryLogger(
            connection,
            profile_config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            _timestamp, profile, _name = next(logger)

        return RecoverySnapshot(
            raw_rotation=float(data["bumblebee.rawRotation"]),
            recovery_altitude=float(data["stateEstimate.z"]),
            recovery_vertical_speed=float(data["stateEstimate.vz"]),
            hover_thrust=float(data["bumblebee.hoverThrust"]),
            minimum_altitude=float(profile["bumblebee.minAltitude"]),
            minimum_voltage=float(profile["bumblebee.minVoltage"]),
            boost_gain=float(profile["bumblebee.boostGain"]),
            brake_start_rotation=float(profile["bumblebee.brakeStart"]),
            brake_end_rate=float(profile["bumblebee.brakeEndRate"]),
            recovery_start_altitude=float(
                profile["bumblebee.recoveryStart"]
            ),
            rotation_end_roll=float(data["bumblebee.endRoll"]),
            rotation_end_pitch=float(data["bumblebee.endPitch"]),
        )

    @staticmethod
    def _trigger(
        connection: Any,
        direction: str,
        profile: FlipProfile,
    ) -> None:
        direction_value = 1 if direction == "backward" else -1
        values = (
            ("bumblebee.boostMm", profile.boost_height_mm),
            ("bumblebee.boostMs", profile.boost_duration_ms),
            ("bumblebee.brakeDeg", profile.brake_start_degrees),
            ("bumblebee.spinPct", profile.spin_thrust_percent),
            ("bumblebee.coastPct", profile.coast_thrust_percent),
            ("bumblebee.brakePct", profile.brake_thrust_percent),
            ("bumblebee.spinTorque", profile.spin_torque_percent),
            ("bumblebee.brakeTorque", profile.brake_torque_percent),
            ("bumblebee.recoverMs", profile.recovery_duration_ms),
            ("bumblebee.direction", direction_value),
        )
        for parameter, value in values:
            connection.cf.param.set_value(parameter, str(value))
        time.sleep(0.1)
        connection.cf.param.set_value(
            "bumblebee.flipRequest", str(FLIP_REQUEST_MAGIC)
        )

    @staticmethod
    def _return_pressed() -> bool:
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return False
        sys.stdin.readline()
        return True


FLIGHT = FlipFlight()
