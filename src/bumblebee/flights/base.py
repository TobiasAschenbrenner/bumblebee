"""Interface shared by selectable flight routines."""

from __future__ import annotations

from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from typing import Any, Protocol

from rich.console import Console

from bumblebee.models import FlightMode, FlightRequirements
from bumblebee.safety import DEFAULT_FLIGHT_SAFETY_LIMITS, FlightSafetyLimits
from bumblebee.supervisor import request_arming


@dataclass(frozen=True)
class FlightContext:
    console: Console
    uri: str
    landing_voltage: float
    preflight_voltage: float | None = None
    mode: FlightMode = FlightMode.SAFE
    safety_limits: FlightSafetyLimits = DEFAULT_FLIGHT_SAFETY_LIMITS


class FlightRoutine(Protocol):
    key: str
    title: str
    description: str
    requirements_label: str
    requirements: FlightRequirements
    requires_acro: bool

    def add_arguments(self, parser: ArgumentParser) -> None: ...

    def settings_from(self, args: Namespace) -> Any: ...

    def plan(self, settings: Any) -> str: ...

    def run(self, connection: Any, settings: Any, context: FlightContext) -> None: ...


def stop_and_disarm(connection: Any, context: FlightContext) -> None:
    operations = (
        ("motor stop", connection.cf.commander.send_stop_setpoint),
        ("setpoint release", connection.cf.commander.send_notify_setpoint_stop),
        ("disarm", lambda: request_arming(connection, False)),
    )
    failed_operations: list[str] = []
    for name, operation in operations:
        try:
            operation()
        except Exception:  # noqa: BLE001 - cflib transport failures are generic.
            failed_operations.append(name)

    if failed_operations:
        failures = ", ".join(failed_operations)
        context.console.print(f"[bold red]Radio cleanup failed:[/bold red] {failures}")
