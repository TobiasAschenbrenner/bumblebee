"""Measure motor balance and power reserve under the stock flight controller."""

from __future__ import annotations

import select
import sys
import time
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from math import hypot
from statistics import fmean
from typing import Any

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from rich.live import Live

from bumblebee.errors import SafetyError
from bumblebee.flights.base import FlightContext, stop_and_disarm
from bumblebee.models import FlightRequirements
from bumblebee.safety import check_position_safety
from bumblebee.supervisor import request_arming
from bumblebee.telemetry import TimedTelemetryLogger, assert_supervisor_safe
from bumblebee.ui.power_test import build_power_test_dashboard

MOTOR_PWM_MAX = 65_535
MOTOR_SATURATION_PWM = 65_500
TEST_HEIGHT = 0.40
SETTLE_SECONDS = 3.0
CLIMB_SECONDS = 1.0
RECOVERY_SECONDS = 2.0
MAXIMUM_TILT_DEG = 20.0


@dataclass(frozen=True)
class PowerTestSettings:
    height: float = TEST_HEIGHT


@dataclass(frozen=True)
class PowerTestSummary:
    hover_headroom_percent: float
    motor_imbalance_percent: float
    voltage_drop: float
    climb_gain: float
    peak_vertical_speed: float
    saturation_percent: float
    maximum_tilt: float
    status: str
    message: str


def evaluate_power_test(
    *,
    preflight_voltage: float,
    minimum_voltage: float,
    hover_requested_means: tuple[float, float, float, float],
    climb_gain: float,
    peak_vertical_speed: float,
    saturation_samples: int,
    sample_count: int,
    maximum_tilt: float,
) -> PowerTestSummary:
    """Turn measured values into a conservative tuning recommendation."""
    highest_request = max(hover_requested_means)
    average_request = fmean(hover_requested_means)
    hover_headroom = max(
        0.0,
        (MOTOR_PWM_MAX - highest_request) / MOTOR_PWM_MAX * 100.0,
    )
    imbalance = (
        0.0
        if average_request <= 0.0
        else (
            max(hover_requested_means) - min(hover_requested_means)
        )
        / average_request
        * 100.0
    )
    voltage_drop = max(0.0, preflight_voltage - minimum_voltage)
    saturation = (
        0.0 if sample_count == 0 else saturation_samples / sample_count * 100.0
    )

    unsafe_reasons: list[str] = []
    if saturation > 2.0:
        unsafe_reasons.append("the motor controller repeatedly saturated")
    if hover_headroom < 10.0:
        unsafe_reasons.append("less than 10% motor headroom remained in hover")
    if voltage_drop > 0.55:
        unsafe_reasons.append("battery voltage sag exceeded 0.55 V")
    if maximum_tilt > MAXIMUM_TILT_DEG:
        unsafe_reasons.append("the stock controller could not remain level")

    if unsafe_reasons:
        return PowerTestSummary(
            hover_headroom,
            imbalance,
            voltage_drop,
            climb_gain,
            peak_vertical_speed,
            saturation,
            maximum_tilt,
            "INSUFFICIENT",
            "; ".join(unsafe_reasons).capitalize() + ".",
        )

    marginal_reasons: list[str] = []
    if hover_headroom < 18.0:
        marginal_reasons.append("motor headroom is narrow")
    if voltage_drop > 0.40:
        marginal_reasons.append("battery sag is high")
    if imbalance > 12.0:
        marginal_reasons.append("the motors need noticeably different power")
    if climb_gain < 0.08 or peak_vertical_speed < 0.10:
        marginal_reasons.append("the controlled climb response is weak")

    if marginal_reasons:
        return PowerTestSummary(
            hover_headroom,
            imbalance,
            voltage_drop,
            climb_gain,
            peak_vertical_speed,
            saturation,
            maximum_tilt,
            "MARGINAL",
            "; ".join(marginal_reasons).capitalize() + ".",
        )

    return PowerTestSummary(
        hover_headroom,
        imbalance,
        voltage_drop,
        climb_gain,
        peak_vertical_speed,
        saturation,
        maximum_tilt,
        "HEALTHY",
        (
            "Normal-flight power and motor balance look healthy enough to "
            "continue with an instrumented flip profile."
        ),
    )


class PowerTestFlight:
    key = "power-test"
    title = "Motor and power test"
    description = "Measure motor balance, battery sag, and controlled climb reserve."
    requirements_label = "Flow deck V2"
    requirements = FlightRequirements(flow_deck=True)
    requires_acro = False
    confirmation_word = "TEST"

    def add_arguments(self, parser: ArgumentParser) -> None:
        del parser

    def settings_from(self, args: Namespace) -> PowerTestSettings:
        del args
        return PowerTestSettings()

    def plan(self, settings: PowerTestSettings) -> str:
        return (
            f"hover at {settings.height:.2f} m, then one "
            "1 s controlled climb at the Safe speed limit"
        )

    def run(
        self,
        connection: Any,
        settings: PowerTestSettings,
        context: FlightContext,
    ) -> None:
        commander = MotionCommander(connection, default_height=settings.height)
        request_arming(connection, True)
        time.sleep(0.5)

        try:
            commander.take_off(
                velocity=context.safety_limits.max_vertical_speed,
            )
            self._measure(connection, commander, settings, context)
        finally:
            try:
                commander.land(
                    velocity=context.safety_limits.max_vertical_speed,
                )
            except Exception as exc:  # noqa: BLE001 - cflib errors are generic.
                context.console.print(
                    "[bold red]Controlled landing command failed:[/bold red] "
                    f"{exc}"
                )
            stop_and_disarm(connection, context)

    def _measure(
        self,
        connection: Any,
        commander: MotionCommander,
        settings: PowerTestSettings,
        context: FlightContext,
    ) -> None:
        telemetry_config = LogConfig(
            name="Bumblebee power telemetry",
            period_in_ms=50,
        )
        telemetry_config.add_variable("pm.vbat", "float")
        telemetry_config.add_variable("supervisor.info", "uint16_t")
        telemetry_config.add_variable("stateEstimate.x", "float")
        telemetry_config.add_variable("stateEstimate.y", "float")
        telemetry_config.add_variable("stateEstimate.z", "float")
        telemetry_config.add_variable("stateEstimate.vz", "float")
        telemetry_config.add_variable("stateEstimate.roll", "FP16")
        telemetry_config.add_variable("stateEstimate.pitch", "FP16")

        motor_config = LogConfig(
            name="Bumblebee power motors",
            period_in_ms=50,
        )
        for motor in range(1, 5):
            motor_config.add_variable(f"motor.m{motor}", "uint16_t")
            motor_config.add_variable(f"motor.m{motor}req", "int32_t")

        started = time.monotonic()
        climb_started: float | None = None
        recovery_started: float | None = None
        climb_start_altitude: float | None = None
        phase = "Settling hover"
        preflight_voltage = context.preflight_voltage or 0.0
        minimum_voltage = preflight_voltage
        maximum_tilt = 0.0
        peak_vertical_speed = 0.0
        peak_altitude = settings.height
        saturation_samples = 0
        sample_count = 0
        hover_request_sums = [0.0, 0.0, 0.0, 0.0]
        hover_sample_count = 0
        current_motors = (0, 0, 0, 0)
        current_requests = (0, 0, 0, 0)

        dashboard = build_power_test_dashboard(
            uri=context.uri,
            phase="Preparing telemetry",
            altitude=None,
            vertical_speed=None,
            voltage=context.preflight_voltage,
            minimum_voltage=context.preflight_voltage,
            motors=None,
            requested_motors=None,
            elapsed=0.0,
            summary=None,
            message="The stock controller is holding the Crazyflie level.",
        )

        with (
            TimedTelemetryLogger(
                connection,
                telemetry_config,
                context.safety_limits.telemetry_timeout,
            ) as telemetry_logger,
            TimedTelemetryLogger(
                connection,
                motor_config,
                context.safety_limits.telemetry_timeout,
            ) as motor_logger,
            Live(
                dashboard,
                console=context.console,
                refresh_per_second=10,
            ) as live,
        ):
            for _timestamp, data, _name in telemetry_logger:
                _motor_timestamp, motor_data, _motor_name = next(motor_logger)
                now = time.monotonic()
                elapsed = now - started
                voltage = float(data["pm.vbat"])
                altitude = float(data["stateEstimate.z"])
                vertical_speed = float(data["stateEstimate.vz"])
                tilt = max(
                    abs(float(data["stateEstimate.roll"])),
                    abs(float(data["stateEstimate.pitch"])),
                )
                current_motors = tuple(
                    int(motor_data[f"motor.m{motor}"])
                    for motor in range(1, 5)
                )
                current_requests = tuple(
                    int(motor_data[f"motor.m{motor}req"])
                    for motor in range(1, 5)
                )

                minimum_voltage = min(minimum_voltage, voltage)
                maximum_tilt = max(maximum_tilt, tilt)
                peak_vertical_speed = max(peak_vertical_speed, vertical_speed)
                peak_altitude = max(peak_altitude, altitude)
                sample_count += 1
                if (
                    max(current_motors) >= MOTOR_SATURATION_PWM
                    or max(current_requests) > MOTOR_PWM_MAX
                ):
                    saturation_samples += 1

                position = check_position_safety(
                    x=float(data["stateEstimate.x"]),
                    y=float(data["stateEstimate.y"]),
                    altitude=altitude,
                    limits=context.safety_limits,
                )
                if position.violation is not None:
                    raise SafetyError(position.violation)
                if voltage < context.landing_voltage:
                    raise SafetyError(
                        f"Battery fell below {context.landing_voltage:.2f} V "
                        "during the motor test."
                    )
                if tilt > MAXIMUM_TILT_DEG:
                    raise SafetyError(
                        f"Tilt reached {tilt:.1f}°, above the "
                        f"{MAXIMUM_TILT_DEG:.0f}° motor-test limit."
                    )
                assert_supervisor_safe(int(data["supervisor.info"]))
                if self._return_pressed():
                    raise SafetyError("Landing was requested during the motor test.")

                if climb_started is None:
                    if elapsed >= 1.0:
                        for index, request in enumerate(current_requests):
                            hover_request_sums[index] += request
                        hover_sample_count += 1
                    if elapsed >= SETTLE_SECONDS:
                        climb_started = now
                        climb_start_altitude = altitude
                        phase = "Controlled climb"
                        commander.start_up(
                            context.safety_limits.max_vertical_speed
                        )
                elif recovery_started is None:
                    if now - climb_started >= CLIMB_SECONDS:
                        commander.stop()
                        recovery_started = now
                        phase = "Recovery hover"
                elif now - recovery_started >= RECOVERY_SECONDS:
                    if hover_sample_count == 0 or climb_start_altitude is None:
                        raise SafetyError(
                            "The motor test ended without enough hover samples."
                        )
                    hover_means = tuple(
                        total / hover_sample_count
                        for total in hover_request_sums
                    )
                    summary = evaluate_power_test(
                        preflight_voltage=preflight_voltage,
                        minimum_voltage=minimum_voltage,
                        hover_requested_means=hover_means,
                        climb_gain=peak_altitude - climb_start_altitude,
                        peak_vertical_speed=peak_vertical_speed,
                        saturation_samples=saturation_samples,
                        sample_count=sample_count,
                        maximum_tilt=maximum_tilt,
                    )
                    live.update(
                        build_power_test_dashboard(
                            uri=context.uri,
                            phase="Measurement complete",
                            altitude=altitude,
                            vertical_speed=vertical_speed,
                            voltage=voltage,
                            minimum_voltage=minimum_voltage,
                            motors=current_motors,
                            requested_motors=current_requests,
                            elapsed=elapsed,
                            summary=summary,
                            message=(
                                f"{summary.message} Landing under the stock "
                                "controller now."
                            ),
                        ),
                        refresh=True,
                    )
                    return

                live.update(
                    build_power_test_dashboard(
                        uri=context.uri,
                        phase=phase,
                        altitude=altitude,
                        vertical_speed=vertical_speed,
                        voltage=voltage,
                        minimum_voltage=minimum_voltage,
                        motors=current_motors,
                        requested_motors=current_requests,
                        elapsed=elapsed,
                        summary=None,
                        message=(
                            "Measuring motor balance and battery sag. "
                            "Press Return to land early."
                        ),
                    ),
                    refresh=True,
                )

        raise SafetyError("Motor-test telemetry ended before completion.")

    @staticmethod
    def _return_pressed() -> bool:
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return False
        sys.stdin.readline()
        return True


FLIGHT = PowerTestFlight()
