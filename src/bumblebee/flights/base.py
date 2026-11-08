"""Interface shared by selectable flight routines."""

from __future__ import annotations

from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from typing import Any, Protocol

from rich.console import Console

from bumblebee.models import FlightRequirements
from bumblebee.safety import DEFAULT_FLIGHT_SAFETY_LIMITS, FlightSafetyLimits


@dataclass(frozen=True)
class FlightContext:
    console: Console
    uri: str
    landing_voltage: float
    safety_limits: FlightSafetyLimits = DEFAULT_FLIGHT_SAFETY_LIMITS


class FlightRoutine(Protocol):
    key: str
    title: str
    description: str
    requirements_label: str
    requirements: FlightRequirements

    def add_arguments(self, parser: ArgumentParser) -> None: ...

    def settings_from(self, args: Namespace) -> Any: ...

    def plan(self, settings: Any) -> str: ...

    def run(self, connection: Any, settings: Any, context: FlightContext) -> None: ...
