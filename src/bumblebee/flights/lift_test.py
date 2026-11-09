"""Measure upright lift and recovery performance without rotating."""

from __future__ import annotations

import select
import sys
import time
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from math import hypot
from typing import Any

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from rich.live import Live

from bumblebee.errors import SafetyError
from bumblebee.flights.base import FlightContext, stop_and_disarm
from bumblebee.flights.hover import _bounded_float
from bumblebee.models import FlightRequirements
from bumblebee.safety import check_position_safety
from bumblebee.supervisor import request_arming
from bumblebee.telemetry import TimedTelemetryLogger, assert_supervisor_safe
from bumblebee.ui.lift_test import build_lift_test_dashboard
from bumblebee.watchdog import EmergencyStopWatchdog

LIFT_TEST_REQUEST_MAGIC = 0xB11F
MINIMUM_TEST_VOLTAGE = 3.50
MAXIMUM_STAGING_DISTANCE = 0.35
MAXIMUM_STAGING_SPEED = 0.10
STABLE_WINDOW_SECONDS = 1.0
STAGING_TIMEOUT_SECONDS = 8.0
PULSE_DURATION_SECONDS = 0.20
POST_RESULT_OBSERVATION_SECONDS = 1.20

LIFT_STATE_NAMES = {
    0: "Staging",
    1: "Lift pulse",
    4: "Recovering",
    5: "Measurement complete",
    6: "Aborted",
}

LIFT_RESULT_MESSAGES = {
    0: "No result was reported.",
    1: "The upright lift pulse and onboard recovery observation completed.",
    2: "The firmware did not consider the Crazyflie airborne.",
    3: "The under-load battery voltage was too low for the test.",
    4: "The Crazyflie was not level enough to begin the test.",
    5: (
        "The test altitude was outside its start window or reached the "
        "1.10 m onboard pulse cutoff."
    ),
    6: "The firmware could not learn a stable hover-thrust value.",
    8: "The supervisor stopped the test.",
    12: "The installed firmware has disabled this request.",
}


@dataclass(frozen=True)
class LiftTestSettings:
    height: float
    settle_seconds: float


@dataclass(frozen=True)
class LiftStageSnapshot:
    altitude: float
    voltage: float


class LiftTestFlight:
    key = "lift-test"
    title = "Lift and recovery test"
    description = "Measure upright thrust reserve without attempting a flip."
    requirements_label = "Flow V2 · future firmware v9+"
    requirements = FlightRequirements(
        flow_deck=True,
        minimum_acro_firmware_version=10,
    )
    requires_acro = False
    confirmation_word = "LIFT"

    def add_arguments(self, parser: ArgumentParser) -> None:
        parser.add_argument(
            "--lift-height",
            type=_bounded_float(0.50, 0.70),
            default=0.60,
            help="lift-test staging height in metres, 0.50-0.70 (default: 0.60)",
        )
        parser.add_argument(
            "--lift-settle",
            type=_bounded_float(2.0, 5.0),
            default=3.0,
            help="stable hover time before the pulse, 2-5 seconds (default: 3)",
        )

    def settings_from(self, args: Namespace) -> LiftTestSettings:
        return LiftTestSettings(
            height=args.lift_height,
            settle_seconds=args.lift_settle,
        )

    def plan(self, settings: LiftTestSettings) -> str:
        return (
            f"stage at {settings.height:.2f} m, apply one "
            f"{PULSE_DURATION_SECONDS * 1000:.0f} ms upright lift pulse"
        )

    def run(
        self,
        connection: Any,
        settings: LiftTestSettings,
        context: FlightContext,
    ) -> None:
        commander = MotionCommander(connection, default_height=settings.height)
        dashboard = build_lift_test_dashboard(
            uri=context.uri,
            phase="Preparing",
            target_height=settings.height,
            current_altitude=None,
            start_altitude=None,
            peak_altitude=None,
            current_vertical_speed=None,
            peak_vertical_speed=None,
            minimum_vertical_speed=None,
            hover_thrust=None,
            current_voltage=context.preflight_voltage,
            minimum_voltage=context.preflight_voltage,
            maximum_tilt=None,
            elapsed=0.0,
            message="Arming and preparing the estimator…",
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
                        stage = self._wait_for_stable_stage(
                            connection,
                            settings,
                            context,
                            watchdog,
                            live,
                        )
                        self._monitor_test(
                            connection,
                            settings,
                            context,
                            watchdog,
                            live,
                            stage,
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
                "[yellow]Power-cycle the Crazyflie before another experimental "
                "test.[/yellow] The radio-loss watchdog intentionally locks after "
                "its heartbeat ends."
            )

    def _wait_for_stable_stage(
        self,
        connection: Any,
        settings: LiftTestSettings,
        context: FlightContext,
        watchdog: EmergencyStopWatchdog,
        live: Live,
    ) -> LiftStageSnapshot:
        config = LogConfig(name="Bumblebee lift staging", period_in_ms=50)
        config.add_variable("pm.vbat", "float")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("stateEstimate.x", "float")
        config.add_variable("stateEstimate.y", "float")
        config.add_variable("stateEstimate.z", "float")
        config.add_variable("stateEstimate.vx", "float")
        config.add_variable("stateEstimate.vy", "float")

        started = time.monotonic()
        stable_since: float | None = None
        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            for _timestamp, data, _name in logger:
                watchdog.assert_healthy()
                now = time.monotonic()
                elapsed = now - started
                voltage = float(data["pm.vbat"])
                x = float(data["stateEstimate.x"])
                y = float(data["stateEstimate.y"])
                altitude = float(data["stateEstimate.z"])
                horizontal_distance = hypot(x, y)
                horizontal_speed = hypot(
                    float(data["stateEstimate.vx"]),
                    float(data["stateEstimate.vy"]),
                )
                position = check_position_safety(
                    x=x,
                    y=y,
                    altitude=altitude,
                    limits=context.safety_limits,
                )

                if position.violation is not None:
                    raise SafetyError(position.violation)
                if horizontal_distance > MAXIMUM_STAGING_DISTANCE:
                    raise SafetyError(
                        f"Bumblebee drifted {horizontal_distance:.2f} m from "
                        "takeoff before the lift test."
                    )
                if voltage < MINIMUM_TEST_VOLTAGE:
                    raise SafetyError(
                        f"Battery fell below {MINIMUM_TEST_VOLTAGE:.2f} V "
                        "while staging."
                    )

                if horizontal_speed <= MAXIMUM_STAGING_SPEED:
                    stable_since = stable_since or now
                else:
                    stable_since = None
                stable_for = 0.0 if stable_since is None else now - stable_since
                ready = (
                    elapsed >= settings.settle_seconds
                    and stable_for >= STABLE_WINDOW_SECONDS
                )

                live.update(
                    build_lift_test_dashboard(
                        uri=context.uri,
                        phase="Staging",
                        target_height=settings.height,
                        current_altitude=altitude,
                        start_altitude=None,
                        peak_altitude=None,
                        current_vertical_speed=None,
                        peak_vertical_speed=None,
                        minimum_vertical_speed=None,
                        hover_thrust=None,
                        current_voltage=voltage,
                        minimum_voltage=voltage,
                        maximum_tilt=None,
                        elapsed=elapsed,
                        message=(
                            "Staging is stable. Sending the onboard lift pulse…"
                            if ready
                            else "Learning hover thrust and checking drift. "
                            "Press Return to land without running the pulse."
                        ),
                    ),
                    refresh=True,
                )

                assert_supervisor_safe(int(data["supervisor.info"]))
                if self._return_pressed():
                    raise SafetyError(
                        "Landing was requested before the lift pulse."
                    )
                if elapsed > settings.settle_seconds + STAGING_TIMEOUT_SECONDS:
                    raise SafetyError(
                        "Bumblebee could not hold a sufficiently stable staging "
                        "hover for the lift test."
                    )
                if ready:
                    return LiftStageSnapshot(
                        altitude=altitude,
                        voltage=voltage,
                    )

        raise SafetyError("Staging telemetry ended before the lift test.")

    def _monitor_test(
        self,
        connection: Any,
        settings: LiftTestSettings,
        context: FlightContext,
        watchdog: EmergencyStopWatchdog,
        live: Live,
        stage: LiftStageSnapshot,
    ) -> None:
        config = LogConfig(name="Bumblebee lift measurement", period_in_ms=20)
        config.add_variable("pm.vbat", "float")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("bumblebee.state", "uint8_t")
        config.add_variable("bumblebee.result", "uint8_t")
        config.add_variable("stateEstimate.z", "float")
        config.add_variable("stateEstimate.vz", "float")
        config.add_variable("bumblebee.hoverThrust", "float")
        config.add_variable("stateEstimate.roll", "FP16")
        config.add_variable("stateEstimate.pitch", "FP16")

        completed_at: float | None = None
        observed_pulse = False
        peak_altitude = stage.altitude
        peak_vertical_speed = 0.0
        minimum_vertical_speed = 0.0
        minimum_voltage = stage.voltage
        maximum_tilt = 0.0
        hover_thrust = 0.0
        last_result = 0

        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            self._trigger(connection)
            triggered_at = time.monotonic()
            for _timestamp, data, _name in logger:
                watchdog.assert_healthy()
                now = time.monotonic()
                elapsed = now - triggered_at
                state = int(data["bumblebee.state"])
                result = int(data["bumblebee.result"])
                altitude = float(data["stateEstimate.z"])
                vertical_speed = float(data["stateEstimate.vz"])
                voltage = float(data["pm.vbat"])
                tilt = max(
                    abs(float(data["stateEstimate.roll"])),
                    abs(float(data["stateEstimate.pitch"])),
                )

                peak_altitude = max(peak_altitude, altitude)
                peak_vertical_speed = max(peak_vertical_speed, vertical_speed)
                minimum_vertical_speed = min(
                    minimum_vertical_speed,
                    vertical_speed,
                )
                minimum_voltage = min(minimum_voltage, voltage)
                maximum_tilt = max(maximum_tilt, tilt)
                hover_thrust = float(data["bumblebee.hoverThrust"])
                last_result = result
                if state != 0:
                    observed_pulse = True
                if state in (5, 6) and completed_at is None:
                    completed_at = now

                if altitude > context.safety_limits.max_altitude:
                    raise SafetyError(
                        f"Lift test reached {altitude:.2f} m, above the "
                        f"{context.safety_limits.max_altitude:.2f} m ceiling."
                    )
                if voltage < context.landing_voltage:
                    raise SafetyError(
                        f"Battery fell below {context.landing_voltage:.2f} V "
                        "during the lift test."
                    )
                assert_supervisor_safe(int(data["supervisor.info"]))
                if self._return_pressed():
                    raise SafetyError("Landing was requested during the lift test.")

                message = (
                    LIFT_RESULT_MESSAGES.get(
                        result,
                        f"Unknown onboard result code {result}.",
                    )
                    if result != 0
                    else "The onboard upright pulse is running."
                )
                if completed_at is not None and result == 1:
                    message = (
                        "Measurement complete. Holding briefly to measure the "
                        "settled recovery before landing…"
                    )

                live.update(
                    build_lift_test_dashboard(
                        uri=context.uri,
                        phase=LIFT_STATE_NAMES.get(state, "Unknown"),
                        target_height=settings.height,
                        current_altitude=altitude,
                        start_altitude=stage.altitude,
                        peak_altitude=peak_altitude,
                        current_vertical_speed=vertical_speed,
                        peak_vertical_speed=peak_vertical_speed,
                        minimum_vertical_speed=minimum_vertical_speed,
                        hover_thrust=hover_thrust,
                        current_voltage=voltage,
                        minimum_voltage=minimum_voltage,
                        maximum_tilt=maximum_tilt,
                        elapsed=elapsed,
                        message=message,
                    ),
                    refresh=True,
                )

                if not observed_pulse and elapsed > 1.5:
                    raise SafetyError(
                        "The onboard firmware did not accept the lift-test trigger."
                    )
                if elapsed > 4.0 and completed_at is None:
                    raise SafetyError(
                        "The onboard lift test did not finish within four seconds."
                    )
                if completed_at is not None:
                    if last_result != 1:
                        detail = LIFT_RESULT_MESSAGES.get(
                            last_result,
                            f"Unknown onboard result code {last_result}.",
                        )
                        raise SafetyError(f"Lift test aborted: {detail}")
                    if now - completed_at >= POST_RESULT_OBSERVATION_SECONDS:
                        live.update(
                            build_lift_test_dashboard(
                                uri=context.uri,
                                phase="Complete",
                                target_height=settings.height,
                                current_altitude=altitude,
                                start_altitude=stage.altitude,
                                peak_altitude=peak_altitude,
                                current_vertical_speed=vertical_speed,
                                peak_vertical_speed=peak_vertical_speed,
                                minimum_vertical_speed=minimum_vertical_speed,
                                hover_thrust=hover_thrust,
                                current_voltage=voltage,
                                minimum_voltage=minimum_voltage,
                                maximum_tilt=maximum_tilt,
                                elapsed=elapsed,
                                message=(
                                    "Lift measurement captured. These values will "
                                    "be used to design the next flip profile."
                                ),
                            ),
                            refresh=True,
                        )
                        return

        raise SafetyError("Lift-test telemetry ended before completion.")

    @staticmethod
    def _trigger(connection: Any) -> None:
        connection.cf.param.set_value(
            "bumblebee.flipRequest",
            str(LIFT_TEST_REQUEST_MAGIC),
        )

    @staticmethod
    def _return_pressed() -> bool:
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return False
        sys.stdin.readline()
        return True


FLIGHT = LiftTestFlight()
