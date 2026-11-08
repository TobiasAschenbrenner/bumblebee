"""Flight selection and common preflight dashboards."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.flights.base import FlightRoutine
from bumblebee.models import FlightMode, PreflightSnapshot
from bumblebee.safety import DEFAULT_FLIGHT_SAFETY_LIMITS
from bumblebee.ui.common import battery_status_style, build_battery_meter


def build_flights_table(flights: Iterable[FlightRoutine]) -> Table:
    table = Table(box=box.SIMPLE, expand=True, show_header=True)
    table.add_column("Choice", width=8, style="bold cyan")
    table.add_column("Flight", style="bold")
    table.add_column("Mode", width=8)
    table.add_column("What it does", ratio=1)
    table.add_column("Needs", width=18)
    for number, flight in enumerate(flights, start=1):
        table.add_row(
            str(number),
            flight.title,
            Text(
                "ACRO" if flight.requires_acro else "SAFE",
                style="bold red" if flight.requires_acro else "bold green",
            ),
            flight.description,
            flight.requirements_label,
        )
    return table


def build_flight_selector(
    flights: Iterable[FlightRoutine], *, interactive: bool
) -> Panel:
    subtitle = (
        "Choose a number below · Q quits without connecting"
        if interactive
        else "Available flight routines"
    )
    return Panel(
        Group(
            Text("Pick a flight and Bumblebee will handle the checks.", style="bold"),
            Text(""),
            build_flights_table(flights),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Flight Console",
        subtitle=f"[dim]{subtitle}[/dim]",
        border_style="yellow",
        box=box.ROUNDED,
        padding=(1, 2),
    )


def build_preflight_dashboard(
    *,
    flight: FlightRoutine,
    settings: Any,
    uri: str,
    snapshot: PreflightSnapshot,
    mode: FlightMode,
) -> Panel:
    limits = DEFAULT_FLIGHT_SAFETY_LIMITS
    telemetry = snapshot.telemetry
    gauge = battery_gauge(telemetry.voltage)
    status_style = battery_status_style(gauge.status)

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=16, style="dim")
    details.add_column()
    details.add_row("Selected flight", Text(flight.title, style="bold"))
    details.add_row(
        "Flight mode",
        Text(
            mode.value.upper(),
            style="bold red" if mode is FlightMode.ACRO else "bold green",
        ),
    )
    details.add_row("Plan", flight.plan(settings))
    details.add_row("Battery", f"{telemetry.voltage:.3f} V")
    details.add_row(
        "Flow deck V2",
        "Detected" if snapshot.flow_deck_attached else "Not detected",
    )
    details.add_row(
        "AI-deck",
        "Detected · idle" if snapshot.ai_deck_attached else "Not detected",
    )
    details.add_row("Supervisor", Text("Ready to fly", style="bold green"))
    if mode is FlightMode.SAFE:
        details.add_row(
            "Safety boundary",
            f"{limits.max_altitude:.2f} m altitude · "
            f"{limits.max_distance:.2f} m radius",
        )
        details.add_row(
            "Speed limits",
            (
                f"{limits.max_horizontal_speed:.2f} m/s horizontal · "
                f"{limits.max_vertical_speed:.2f} m/s vertical"
            ),
        )
    else:
        details.add_row(
            "Safe boundaries",
            Text("DISABLED for this run", style="bold red"),
        )
    details.add_row("Radio", uri)

    if mode is FlightMode.ACRO:
        checklist = Text.from_markup(
            "[bold red]Acro check[/bold red]\n"
            "• Use the prepared padded area and eye protection\n"
            "• At least 4 × 4 m of floor and 3 m overhead are clear\n"
            "• Everyone else, pets, cables, and fragile objects are outside "
            "the area\n"
            "• The battery is freshly charged and the propellers are unobstructed\n"
            "• Be ready for an immediate uncontrolled landing"
        )
    else:
        checklist = Text.from_markup(
            "[bold]Visual check[/bold]\n"
            "• Propellers are fitted correctly and unobstructed\n"
            "• Crazyflie is level on a matte, textured floor\n"
            "• At least 4 × 4 m of floor space and 2 m overhead are clear\n"
            "• No people, pets, loose cables, or fragile objects are nearby"
        )
    return Panel(
        Group(
            build_battery_meter(telemetry.voltage),
            Text(""),
            details,
            Text(""),
            checklist,
        ),
        title=(
            "[bold red]Acro preflight checks passed[/bold red]"
            if mode is FlightMode.ACRO
            else "[bold green]Preflight checks passed[/bold green]"
        ),
        subtitle="[dim]Nothing is armed yet[/dim]",
        border_style=status_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
