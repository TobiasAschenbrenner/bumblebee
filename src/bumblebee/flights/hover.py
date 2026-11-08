"""Low takeoff, short hover, and controlled landing using Flow deck V2."""

from __future__ import annotations

import select
import sys
import time
from argparse import ArgumentParser, ArgumentTypeError, Namespace
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from statistics import fmean
from typing import Any

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from rich.live import Live

from bumblebee.errors import SafetyError
from bumblebee.flights.base import FlightContext, stop_and_disarm
from bumblebee.models import FlightMode, FlightRequirements
from bumblebee.safety import DEFAULT_FLIGHT_SAFETY_LIMITS, check_position_safety
from bumblebee.telemetry import (
    TimedTelemetryLogger,
    assert_supervisor_safe,
    power_state_name,
)
from bumblebee.ui.hover import build_hover_dashboard


@dataclass(frozen=True)
class HoverSettings:
    height: float
    duration: float


def _bounded_float(minimum: float, maximum: float) -> Callable[[str], float]:
    def parse(value: str) -> float:
        parsed = float(value)
        if not minimum <= parsed <= maximum:
            raise ArgumentTypeError(f"must be between {minimum:g} and {maximum:g}")
        return parsed

    return parse


class HoverFlight:
    key = "hover"
    title = "First hover"
    description = "Low takeoff, short hover, and controlled landing."
    requirements_label = "Flow deck V2"
    requirements = FlightRequirements(flow_deck=True)
    requires_acro = False

    def add_arguments(self, parser: ArgumentParser) -> None:
        parser.add_argument(
            "--height",
            type=_bounded_float(0.20, DEFAULT_FLIGHT_SAFETY_LIMITS.max_altitude),
            default=0.30,
            help=(
                "hover height in metres, 0.20-"
                f"{DEFAULT_FLIGHT_SAFETY_LIMITS.max_altitude:.2f} (default: 0.30)"
            ),
        )
        parser.add_argument(
            "--seconds",
            type=_bounded_float(2.0, 15.0),
            default=5.0,
            help="hover duration, 2-15 seconds (default: 5)",
        )

    def settings_from(self, args: Namespace) -> HoverSettings:
        return HoverSettings(height=args.height, duration=args.seconds)

    def plan(self, settings: HoverSettings) -> str:
        return f"{settings.height:.2f} m for {settings.duration:.1f} seconds"

    def run(
        self,
        connection: Any,
        settings: HoverSettings,
        context: FlightContext,
    ) -> None:
        initial_dashboard = build_hover_dashboard(
            uri=context.uri,
            phase="Preparing",
            target_height=settings.height,
            altitude=None,
            elapsed=0.0,
            duration=settings.duration,
            voltage=None,
            power_state="waiting",
            horizontal_distance=None,
            safety_limits=context.safety_limits,
            message="Arming and preparing the estimator…",
        )
        commander = MotionCommander(connection, default_height=settings.height)

        try:
            connection.cf.supervisor.send_arming_request(True)
            time.sleep(0.5)

            with Live(
                initial_dashboard,
                console=context.console,
                refresh_per_second=8,
            ) as live:
                live.update(
                    build_hover_dashboard(
                        uri=context.uri,
                        phase="Taking off",
                        target_height=settings.height,
                        altitude=None,
                        elapsed=0.0,
                        duration=settings.duration,
                        voltage=None,
                        power_state="battery",
                        horizontal_distance=None,
                        safety_limits=context.safety_limits,
                        message="Taking off under Flow deck control…",
                    ),
                    refresh=True,
                )
                commander.take_off(
                    velocity=context.safety_limits.max_vertical_speed,
                )
                self._monitor_hover(connection, settings, context, live)
        finally:
            try:
                commander.land(
                    velocity=context.safety_limits.max_vertical_speed,
                )
            except Exception as exc:  # noqa: BLE001 - cflib raises generic flight errors.
                context.console.print(
                    f"[bold red]Controlled landing command failed:[/bold red] {exc}"
                )
            stop_and_disarm(connection, context)

    def _monitor_hover(
        self,
        connection: Any,
        settings: HoverSettings,
        context: FlightContext,
        live: Live,
    ) -> None:
        config = LogConfig(name="Hover monitor", period_in_ms=200)
        config.add_variable("pm.vbat", "float")
        config.add_variable("pm.state", "int8_t")
        config.add_variable("supervisor.info", "uint16_t")
        config.add_variable("stateEstimate.x", "float")
        config.add_variable("stateEstimate.y", "float")
        config.add_variable("stateEstimate.z", "float")

        started = time.monotonic()
        voltage_history: deque[float] = deque(maxlen=5)
        with TimedTelemetryLogger(
            connection,
            config,
            context.safety_limits.telemetry_timeout,
        ) as logger:
            for _timestamp, data, _name in logger:
                now = time.monotonic()
                elapsed = now - started
                voltage = float(data["pm.vbat"])
                voltage_history.append(voltage)
                altitude = float(data["stateEstimate.z"])
                if context.mode is FlightMode.SAFE:
                    position_check = check_position_safety(
                        x=float(data["stateEstimate.x"]),
                        y=float(data["stateEstimate.y"]),
                        altitude=altitude,
                        limits=context.safety_limits,
                    )
                else:
                    position_check = check_position_safety(
                        x=0.0,
                        y=0.0,
                        altitude=0.0,
                        limits=context.safety_limits,
                    )
                power_code = int(data["pm.state"])

                phase, message, should_land = self._landing_decision(
                    voltage=voltage,
                    power_code=power_code,
                    supervisor_info=int(data["supervisor.info"]),
                    elapsed=elapsed,
                    duration=settings.duration,
                    landing_voltage=context.landing_voltage,
                    position_violation=position_check.violation,
                )

                live.update(
                    build_hover_dashboard(
                        uri=context.uri,
                        phase=phase,
                        target_height=settings.height,
                        altitude=altitude,
                        elapsed=elapsed,
                        duration=settings.duration,
                        voltage=fmean(voltage_history),
                        power_state=power_state_name(power_code),
                        horizontal_distance=position_check.horizontal_distance,
                        safety_limits=context.safety_limits,
                        message=message,
                    ),
                    refresh=True,
                )
                if should_land:
                    return

    @staticmethod
    def _landing_decision(
        *,
        voltage: float,
        power_code: int,
        supervisor_info: int,
        elapsed: float,
        duration: float,
        landing_voltage: float,
        position_violation: str | None,
    ) -> tuple[str, str, bool]:
        if position_violation is not None:
            return (
                "Landing · boundary reached",
                f"{position_violation}. Starting a controlled landing.",
                True,
            )
        if voltage < landing_voltage:
            return (
                "Landing · low battery",
                f"Battery fell below {landing_voltage:.2f} V. Starting a controlled landing.",
                True,
            )
        if power_code != 0:
            return (
                "Landing · power changed",
                (
                    f"Power state changed to {power_state_name(power_code)}. "
                    "Starting a controlled landing."
                ),
                True,
            )

        try:
            assert_supervisor_safe(supervisor_info)
        except SafetyError as exc:
            return (
                "Landing · safety check",
                f"{exc} Starting a controlled landing.",
                True,
            )

        if select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.readline()
            return (
                "Landing · requested",
                "Return pressed. Starting a controlled landing.",
                True,
            )
        if elapsed >= duration:
            return (
                "Landing · flight complete",
                "Hover time complete. Starting a controlled landing.",
                True,
            )

        return (
            "Hovering",
            "Flight is stable. Press Return whenever you want to land.",
            False,
        )


FLIGHT = HoverFlight()
