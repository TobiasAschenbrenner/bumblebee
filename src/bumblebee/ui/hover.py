"""Live dashboard for the hover flight."""

from __future__ import annotations

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.ui.common import battery_status_style, build_battery_meter


def build_hover_dashboard(
    *,
    uri: str,
    phase: str,
    target_height: float,
    altitude: float | None,
    elapsed: float,
    duration: float,
    voltage: float | None,
    power_state: str,
    message: str,
) -> Panel:
    if voltage is None:
        voltage_text = "Waiting for telemetry…"
        safety_text = Text("CHECKING", style="bold cyan")
        border_style = "cyan"
    else:
        gauge = battery_gauge(voltage)
        voltage_text = f"{gauge.voltage:.3f} V"
        border_style = battery_status_style(gauge.status)
        safety_text = Text(gauge.status.value, style=border_style)

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=14, style="dim")
    details.add_column()
    details.add_row("Flight phase", Text(phase, style="bold cyan"))
    details.add_row(
        "Altitude",
        "Waiting…"
        if altitude is None
        else f"{altitude:.2f} m / {target_height:.2f} m target",
    )
    details.add_row(
        "Hover timer", f"{min(elapsed, duration):.1f} / {duration:.1f} seconds"
    )
    details.add_row("Battery", Text(voltage_text, style="bold"))
    details.add_row("Power state", power_state)
    details.add_row("Safety", safety_text)
    details.add_row("Radio", uri)

    return Panel(
        Group(
            build_battery_meter(voltage),
            Text(""),
            details,
            Text(""),
            Text(message),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · First Hover",
        subtitle="[dim]Return: land early · Control-C: request landing[/dim]",
        border_style=border_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
