"""Collect and evaluate the safety state shared by diagnostics and flights."""

from __future__ import annotations

from typing import Any

from bumblebee.battery import battery_gauge
from bumblebee.config import (
    BATTERY_LAND_VOLTAGE,
    MINIMUM_TAKEOFF_PERCENTAGE,
    MINIMUM_TAKEOFF_VOLTAGE,
)
from bumblebee.errors import SafetyError
from bumblebee.models import FlightRequirements, PreflightSnapshot
from bumblebee.telemetry import (
    assert_supervisor_safe,
    power_state_name,
    read_deck_flag,
    read_low_voltage_setting,
    read_optional_parameter_int,
    read_telemetry,
)


def collect_preflight_snapshot(connection: Any) -> PreflightSnapshot:
    return PreflightSnapshot(
        flow_deck_attached=read_deck_flag(connection, "deck.bcFlow2"),
        ai_deck_attached=read_deck_flag(connection, "deck.bcAI"),
        firmware_low_voltage=read_low_voltage_setting(connection),
        telemetry=read_telemetry(connection),
        acro_firmware_version=read_optional_parameter_int(
            connection, "bumblebee.version"
        ),
    )


def preflight_issues(
    snapshot: PreflightSnapshot,
    requirements: FlightRequirements,
) -> list[str]:
    issues: list[str] = []
    telemetry = snapshot.telemetry

    if requirements.flow_deck and not snapshot.flow_deck_attached:
        issues.append("Flow deck V2 was not detected")
    if requirements.ai_deck and not snapshot.ai_deck_attached:
        issues.append("the AI-deck is required but was not detected")
    minimum_acro_version = requirements.minimum_acro_firmware_version
    installed_acro_version = snapshot.acro_firmware_version
    if minimum_acro_version is not None:
        if installed_acro_version is None:
            issues.append(
                "the Bumblebee acro firmware is required but was not detected"
            )
        elif installed_acro_version < minimum_acro_version:
            issues.append(
                f"Acro firmware version {installed_acro_version} is unsafe for "
                f"this routine; version {minimum_acro_version} or newer is required"
            )
    if telemetry.power_state != 0:
        issues.append(
            f"power state is {power_state_name(telemetry.power_state)}; "
            "disconnect the Crazyflie's USB cable"
        )

    takeoff_voltage = max(MINIMUM_TAKEOFF_VOLTAGE, snapshot.firmware_low_voltage + 0.20)
    gauge = battery_gauge(telemetry.voltage)
    if telemetry.voltage < takeoff_voltage:
        issues.append(
            f"battery is {telemetry.voltage:.2f} V; at least "
            f"{takeoff_voltage:.2f} V is required"
        )
    elif gauge.percentage < MINIMUM_TAKEOFF_PERCENTAGE:
        issues.append(
            f"battery bar is {gauge.percentage}% usable; "
            f"at least {MINIMUM_TAKEOFF_PERCENTAGE}% is required"
        )

    if telemetry.supervisor_info is None:
        issues.append(
            "firmware does not provide supervisor.info; update it in cfclient"
        )
    else:
        try:
            assert_supervisor_safe(telemetry.supervisor_info)
        except SafetyError as exc:
            issues.append(str(exc))

    return issues


def require_safe_preflight(
    snapshot: PreflightSnapshot,
    requirements: FlightRequirements,
) -> None:
    issues = preflight_issues(snapshot, requirements)
    if issues:
        raise SafetyError("; ".join(issues))


def landing_voltage(snapshot: PreflightSnapshot) -> float:
    return max(BATTERY_LAND_VOLTAGE, snapshot.firmware_low_voltage)
