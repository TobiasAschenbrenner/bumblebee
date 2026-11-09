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
    2: "Rotating",
    3: "Recovering",
    4: "Verifying recovery",
    5: "Complete",
    6: "Aborted",
}

FLIP_RESULT_MESSAGES = {
    0: "No result was reported.",
    1: "The rotation completed and stable-flight recovery was verified.",
    2: "The firmware did not consider the Crazyflie airborne.",
    3: "The under-load battery voltage was too low to safely begin the flip.",
    4: "The Crazyflie was not level enough to begin.",
    5: "The staging altitude was outside 0.75–1.25 m.",
    6: "The firmware could not learn a stable hover-thrust value.",
    7: "The requested rotation did not complete before its timeout.",
    8: "The supervisor stopped the maneuver.",
    9: "The supervisor denied the time-limited Acro permit.",
    10: "Net rotation was reached, but the Crazyflie was not upright.",
    11: "The rotation completed, but stable flight was not recovered.",
}


@dataclass(frozen=True)
class FlipSettings:
    height: float
    settle_seconds: float
    direction: str


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
        minimum_acro_firmware_version=5,
    )
    requires_acro = True

    def add_arguments(self, parser: ArgumentParser) -> None:
        parser.add_argument(
            "--flip-height",
            type=_bounded_float(0.80, 1.20),
            default=1.00,
            help="flip staging height in metres, 0.80-1.20 (default: 1.00)",
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
        )

    def plan(self, settings: FlipSettings) -> str:
        return (
            f"stage at {settings.height:.2f} m, then one "
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
                        self._trigger(connection, settings.direction)
                        self._monitor_flip(
                            connection,
                            settings,
                            context,
                            watchdog,
                            live,
                            staging,
                        )
                finally:
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
        finally:
            if not watchdog_started:
                stop_and_disarm(connection, context)
            context.console.print(
                "[yellow]Power-cycle the Crazyflie before another flight.[/yellow] "
                "The Acro radio-loss watchdog intentionally locks after its "
                "heartbeat ends."
            )

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
        config = LogConfig(name="Bumblebee flip", period_in_ms=50)
        config.add_variable("pm.vbat", "float")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("bumblebee.state", "uint8_t")
        config.add_variable("bumblebee.result", "uint8_t")
        config.add_variable("bumblebee.rotation", "float")
        config.add_variable("bumblebee.peakRate", "float")
        config.add_variable("bumblebee.endRoll", "float")
        config.add_variable("bumblebee.endPitch", "float")

        triggered_at = time.monotonic()
        observed_maneuver = False
        final_result: int | None = None
        voltage = staging.voltage
        state = 0
        rotation = 0.0
        peak_pitch_rate = 0.0
        rotation_end_roll = 0.0
        rotation_end_pitch = 0.0

        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            for _timestamp, data, _name in logger:
                watchdog.assert_healthy()
                now = time.monotonic()
                voltage = float(data["pm.vbat"])
                state = int(data["bumblebee.state"])
                result = int(data["bumblebee.result"])
                rotation = float(data["bumblebee.rotation"])
                peak_pitch_rate = float(data["bumblebee.peakRate"])
                rotation_end_roll = float(data["bumblebee.endRoll"])
                rotation_end_pitch = float(data["bumblebee.endPitch"])

                message = (
                    "The onboard maneuver is running."
                    if result == 0
                    else FLIP_RESULT_MESSAGES.get(
                        result,
                        "The onboard maneuver is running.",
                    )
                )
                if state not in (0, 5, 6):
                    observed_maneuver = True
                if state in (5, 6):
                    final_result = result

                live.update(
                    build_flip_dashboard(
                        uri=context.uri,
                        phase=FLIP_STATE_NAMES.get(state, "Unknown"),
                        direction=settings.direction,
                        target_height=settings.height,
                        altitude=staging.altitude,
                        voltage=voltage,
                        battery_live=True,
                        minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
                        rotation=rotation,
                        raw_rotation=None,
                        peak_pitch_rate=peak_pitch_rate,
                        rotation_end_roll=(
                            rotation_end_roll if state in (5, 6) else None
                        ),
                        rotation_end_pitch=(
                            rotation_end_pitch if state in (5, 6) else None
                        ),
                        recovery_altitude=None,
                        recovery_vertical_speed=None,
                        hover_thrust_ready=True,
                        hover_thrust=None,
                        watchdog_active=True,
                        horizontal_distance=staging.horizontal_distance,
                        horizontal_speed=staging.horizontal_speed,
                        message=message,
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
            state = 6
        detail = FLIP_RESULT_MESSAGES.get(
            final_result,
            f"Unknown result code {final_result}.",
        )
        if host_recovery_issue is not None:
            detail = f"{detail} {host_recovery_issue}"
        live.update(
            build_flip_dashboard(
                uri=context.uri,
                phase=FLIP_STATE_NAMES.get(state, "Unknown"),
                direction=settings.direction,
                target_height=settings.height,
                altitude=staging.altitude,
                voltage=voltage,
                battery_live=True,
                minimum_flip_voltage=MINIMUM_FLIP_VOLTAGE,
                rotation=rotation,
                raw_rotation=recovery.raw_rotation,
                peak_pitch_rate=peak_pitch_rate,
                rotation_end_roll=rotation_end_roll,
                rotation_end_pitch=rotation_end_pitch,
                recovery_altitude=recovery.recovery_altitude,
                recovery_vertical_speed=recovery.recovery_vertical_speed,
                hover_thrust_ready=True,
                hover_thrust=recovery.hover_thrust,
                watchdog_active=True,
                horizontal_distance=staging.horizontal_distance,
                horizontal_speed=staging.horizontal_speed,
                message=detail,
            ),
            refresh=True,
        )

        if final_result != 1:
            raise SafetyError(f"Flip aborted: {detail}")
        time.sleep(0.75)

    @staticmethod
    def _read_recovery_snapshot(
        connection: Any,
        context: FlightContext,
    ) -> RecoverySnapshot:
        config = LogConfig(name="Bumblebee recovery result", period_in_ms=50)
        config.add_variable("bumblebee.rawRotation", "float")
        config.add_variable("bumblebee.hoverThrust", "float")
        config.add_variable("stateEstimate.z", "float")
        config.add_variable("stateEstimate.vz", "float")

        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            _timestamp, data, _name = next(logger)

        return RecoverySnapshot(
            raw_rotation=float(data["bumblebee.rawRotation"]),
            recovery_altitude=float(data["stateEstimate.z"]),
            recovery_vertical_speed=float(data["stateEstimate.vz"]),
            hover_thrust=float(data["bumblebee.hoverThrust"]),
        )

    @staticmethod
    def _trigger(connection: Any, direction: str) -> None:
        direction_value = 1 if direction == "backward" else -1
        connection.cf.param.set_value(
            "bumblebee.direction", str(direction_value)
        )
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
