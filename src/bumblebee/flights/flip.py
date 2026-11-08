"""Experimental onboard flip with guarded staging and automatic recovery."""

from __future__ import annotations

import select
import sys
import time
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
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
MINIMUM_FLIP_VOLTAGE = 3.75

FLIP_STATE_NAMES = {
    0: "Staging",
    1: "Boosting",
    2: "Rotating",
    3: "Recovering",
    4: "Levelling",
    5: "Complete",
    6: "Aborted",
}

FLIP_RESULT_MESSAGES = {
    0: "No result was reported.",
    1: "The onboard flip sequence completed.",
    2: "The firmware did not consider the Crazyflie airborne.",
    3: "The battery was below the onboard Acro minimum.",
    4: "The Crazyflie was not level enough to begin.",
    5: "The staging altitude was outside 0.75–1.25 m.",
    6: "The firmware could not learn a stable hover-thrust value.",
    7: "The requested rotation did not complete before its timeout.",
    8: "The supervisor stopped the maneuver.",
    9: "The supervisor denied the time-limited Acro permit.",
}


@dataclass(frozen=True)
class FlipSettings:
    height: float
    settle_seconds: float
    direction: str


class FlipFlight:
    key = "flip"
    title = "Experimental flip"
    description = "Take off, perform one onboard flip, recover, and land."
    requirements_label = "Flow V2 · Acro firmware"
    requirements = FlightRequirements(flow_deck=True, acro_firmware=True)
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
            phase="Preparing",
            direction=settings.direction,
            target_height=settings.height,
            altitude=None,
            voltage=None,
            rotation=0.0,
            hover_thrust_ready=False,
            watchdog_active=False,
            message="Arming and starting the Acro safety heartbeat…",
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
                        self._stage_and_flip(
                            connection,
                            settings,
                            context,
                            watchdog,
                            live,
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

    def _stage_and_flip(
        self,
        connection: Any,
        settings: FlipSettings,
        context: FlightContext,
        watchdog: EmergencyStopWatchdog,
        live: Live,
    ) -> None:
        config = LogConfig(name="Bumblebee flip", period_in_ms=50)
        config.add_variable("pm.vbat", "float")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("stateEstimate.z", "float")
        config.add_variable("bumblebee.state", "uint8_t")
        config.add_variable("bumblebee.result", "uint8_t")
        config.add_variable("bumblebee.rotation", "float")
        config.add_variable("bumblebee.hoverThrust", "float")

        started = time.monotonic()
        triggered_at: float | None = None
        observed_maneuver = False
        final_result: int | None = None

        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            for _timestamp, data, _name in logger:
                watchdog.assert_healthy()
                now = time.monotonic()
                voltage = float(data["pm.vbat"])
                altitude = float(data["stateEstimate.z"])
                state = int(data["bumblebee.state"])
                result = int(data["bumblebee.result"])
                rotation = float(data["bumblebee.rotation"])
                hover_thrust_ready = float(data["bumblebee.hoverThrust"]) > 0.0

                assert_supervisor_safe(int(data["supervisor.info"]))

                if triggered_at is None:
                    elapsed = now - started
                    if self._return_pressed():
                        raise SafetyError(
                            "Landing was requested before the flip trigger."
                        )
                    if voltage < MINIMUM_FLIP_VOLTAGE:
                        raise SafetyError(
                            f"Battery fell below {MINIMUM_FLIP_VOLTAGE:.2f} V "
                            "while staging."
                        )

                    ready = elapsed >= settings.settle_seconds and hover_thrust_ready
                    message = (
                        "Holding steady while the firmware learns this "
                        "airframe's hover thrust. Press Return to cancel."
                    )
                    if ready:
                        self._trigger(connection, settings.direction)
                        triggered_at = now
                        message = (
                            "Flip trigger sent. The maneuver now runs onboard; "
                            "Bumblebee will recover and land automatically."
                        )
                else:
                    ready = True
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
                        altitude=altitude,
                        voltage=voltage,
                        rotation=rotation,
                        hover_thrust_ready=hover_thrust_ready,
                        watchdog_active=True,
                        message=message,
                    ),
                    refresh=True,
                )

                if final_result is not None:
                    if final_result != 1:
                        detail = FLIP_RESULT_MESSAGES.get(
                            final_result, f"Unknown result code {final_result}."
                        )
                        raise SafetyError(f"Flip aborted: {detail}")
                    time.sleep(0.75)
                    return

                if triggered_at is not None:
                    since_trigger = now - triggered_at
                    if not observed_maneuver and since_trigger > 2.0:
                        raise SafetyError(
                            "The onboard firmware did not accept the flip trigger."
                        )
                    if since_trigger > 4.0:
                        raise SafetyError(
                            "The onboard flip did not finish within four seconds."
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
