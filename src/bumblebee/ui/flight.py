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
from bumblebee.models import PreflightSnapshot
from bumblebee.ui.common import battery_status_style, build_battery_meter


def build_flights_table(flights: Iterable[FlightRoutine]) -> Table:
    table = Table(box=box.SIMPLE, expand=True, show_header=True)
    table.add_column("Choice", width=8, style="bold cyan")
    table.add_column("Flight", style="bold")
    table.add_column("What it does", ratio=1)
    table.add_column("Needs", width=18)
    for number, flight in enumerate(flights, start=1):
        table.add_row(
            str(number), flight.title, flight.description, flight.requirements_label
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
) -> Panel:
    telemetry = snapshot.telemetry
    gauge = battery_gauge(telemetry.voltage)
    status_style = battery_status_style(gauge.status)

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=16, style="dim")
    details.add_column()
    details.add_row("Selected flight", Text(flight.title, style="bold"))
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
    details.add_row("Radio", uri)

    checklist = Text.from_markup(
        "[bold]Visual check[/bold]\n"
        "• Propellers are fitted correctly and unobstructed\n"
        "• Crazyflie is level on a matte, textured floor\n"
        "• At least 2 × 2 m around and above it is clear\n"
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
        title="[bold green]Preflight checks passed[/bold green]",
        subtitle="[dim]Nothing is armed yet[/dim]",
        border_style=status_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
