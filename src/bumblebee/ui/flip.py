"""Live dashboard for the experimental flip routine."""

from __future__ import annotations

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.ui.common import build_battery_meter


def build_flip_dashboard(
    *,
    uri: str,
    phase: str,
    direction: str,
    target_height: float,
    altitude: float | None,
    voltage: float | None,
    rotation: float,
    hover_thrust_ready: bool,
    watchdog_active: bool,
    message: str,
) -> Panel:
    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=18, style="dim")
    details.add_column()
    details.add_row("Flight phase", Text(phase, style="bold yellow"))
    details.add_row("Direction", direction.title())
    details.add_row(
        "Altitude",
        "Waiting…"
        if altitude is None
        else f"{altitude:.2f} m · {target_height:.2f} m staging target",
    )
    details.add_row(
        "Rotation measured",
        f"{rotation:.0f}° / 340° target",
    )
    details.add_row(
        "Hover thrust",
        Text(
            "Learned" if hover_thrust_ready else "Learning…",
            style="green" if hover_thrust_ready else "cyan",
        ),
    )
    details.add_row(
        "Radio-loss watchdog",
        Text("ACTIVE" if watchdog_active else "Waiting", style="bold red"),
    )
    details.add_row(
        "Battery",
        "Waiting…" if voltage is None else f"{voltage:.3f} V",
    )
    details.add_row("Radio", uri)

    return Panel(
        Group(
            build_battery_meter(voltage),
            Text(""),
            details,
            Text(""),
            Text(message),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Experimental Flip",
        subtitle="[bold red]ACRO · padded test area only[/bold red]",
        border_style="red",
        box=box.ROUNDED,
        padding=(1, 2),
    )
