"""Battery monitor dashboard."""

from __future__ import annotations

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.models import BatteryStatus
from bumblebee.ui.common import battery_status_style, build_battery_meter


def build_battery_dashboard(
    *,
    uri: str,
    voltage: float,
    power_state: str,
    updated_at: str,
) -> Panel:
    gauge = battery_gauge(voltage)
    style = battery_status_style(gauge.status)

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=14, style="dim")
    details.add_column()
    details.add_row("Voltage", Text(f"{gauge.voltage:.3f} V", style="bold"))
    details.add_row("Power state", power_state)
    details.add_row("Safety", Text(gauge.status.value, style=style))
    details.add_row("Radio", uri)
    details.add_row("Updated", updated_at)

    guidance = {
        BatteryStatus.OK: "Battery voltage is in the normal operating range.",
        BatteryStatus.LOW: "Battery is low. Land soon and recharge.",
        BatteryStatus.LAND: "Do not take off. If airborne, land immediately.",
    }[gauge.status]

    return Panel(
        Group(
            build_battery_meter(voltage),
            Text(""),
            details,
            Text(""),
            Text(guidance, style=style),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Battery Monitor",
        subtitle="[dim]Control-C: close monitor · no motor commands[/dim]",
        border_style=style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
