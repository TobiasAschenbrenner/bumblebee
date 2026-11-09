"""Live dashboard for the upright lift and recovery measurement."""

from __future__ import annotations

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.safety import DEFAULT_FLIGHT_SAFETY_LIMITS
from bumblebee.ui.common import battery_status_style, build_battery_meter


def build_lift_test_dashboard(
    *,
    uri: str,
    phase: str,
    target_height: float,
    current_altitude: float | None,
    start_altitude: float | None,
    peak_altitude: float | None,
    current_vertical_speed: float | None,
    peak_vertical_speed: float | None,
    minimum_vertical_speed: float | None,
    hover_thrust: float | None,
    current_voltage: float | None,
    minimum_voltage: float | None,
    maximum_tilt: float | None,
    elapsed: float,
    message: str,
) -> Panel:
    if current_voltage is None:
        border_style = "cyan"
        voltage_text = "Waiting…"
    else:
        gauge = battery_gauge(current_voltage)
        border_style = battery_status_style(gauge.status)
        voltage_text = f"{current_voltage:.3f} V"

    altitude_gain = (
        None
        if start_altitude is None or peak_altitude is None
        else peak_altitude - start_altitude
    )
    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=19, style="dim")
    details.add_column()
    details.add_row("Test phase", Text(phase, style="bold cyan"))
    details.add_row(
        "Altitude",
        "Waiting…"
        if current_altitude is None
        else (
            f"{current_altitude:.2f} m · {target_height:.2f} m staging · "
            f"{DEFAULT_FLIGHT_SAFETY_LIMITS.max_altitude:.2f} m ceiling"
        ),
    )
    details.add_row(
        "Peak altitude gain",
        "Waiting…" if altitude_gain is None else f"{altitude_gain:+.2f} m",
    )
    details.add_row(
        "Vertical speed",
        "Waiting…"
        if (
            current_vertical_speed is None
            or peak_vertical_speed is None
            or minimum_vertical_speed is None
        )
        else (
            f"{current_vertical_speed:+.2f} m/s live · "
            f"{peak_vertical_speed:+.2f} peak · "
            f"{minimum_vertical_speed:+.2f} lowest"
        ),
    )
    details.add_row(
        "Maximum tilt",
        "Waiting…" if maximum_tilt is None else f"{maximum_tilt:.1f}°",
    )
    details.add_row(
        "Learned hover thrust",
        "Waiting…" if hover_thrust is None else f"{hover_thrust:.0f}",
    )
    details.add_row("Lift pulse", "200 ms upright · thrust capped at 60000")
    details.add_row(
        "Battery",
        (
            voltage_text
            if minimum_voltage is None
            else f"{voltage_text} · {minimum_voltage:.3f} V minimum"
        ),
    )
    details.add_row("Elapsed", f"{elapsed:.1f} seconds")
    details.add_row("Radio-loss watchdog", "ACTIVE")
    details.add_row("Radio", uri)

    return Panel(
        Group(
            build_battery_meter(current_voltage),
            Text(""),
            details,
            Text(""),
            Text(message),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Lift and Recovery Test",
        subtitle="[dim]Upright measurement only · Return: land early[/dim]",
        border_style=border_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
