"""Live dashboard for the stock-controller motor and power test."""

from __future__ import annotations

from typing import TYPE_CHECKING

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.progress_bar import ProgressBar
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.ui.common import battery_status_style, build_battery_meter

if TYPE_CHECKING:
    from bumblebee.flights.power_test import PowerTestSummary

MOTOR_PWM_MAX = 65_535


def _motor_table(
    motors: tuple[int, int, int, int] | None,
    requested_motors: tuple[int, int, int, int] | None,
) -> Table:
    table = Table(box=box.SIMPLE, expand=True, show_header=True)
    table.add_column("Motor", width=7, style="bold")
    table.add_column("Actual", width=8, justify="right")
    table.add_column("Requested", width=10, justify="right")
    table.add_column("Load", ratio=1)

    for index in range(4):
        actual = None if motors is None else motors[index]
        requested = None if requested_motors is None else requested_motors[index]
        percent = 0.0 if actual is None else actual / MOTOR_PWM_MAX * 100.0
        table.add_row(
            f"M{index + 1}",
            "Waiting…" if actual is None else f"{actual}",
            "Waiting…" if requested is None else f"{requested:>6}",
            ProgressBar(
                total=100,
                completed=min(100.0, percent),
                width=None,
                pulse=actual is None,
            ),
        )
    return table


def build_power_test_dashboard(
    *,
    uri: str,
    phase: str,
    altitude: float | None,
    vertical_speed: float | None,
    voltage: float | None,
    minimum_voltage: float | None,
    motors: tuple[int, int, int, int] | None,
    requested_motors: tuple[int, int, int, int] | None,
    elapsed: float,
    summary: PowerTestSummary | None,
    message: str,
) -> Panel:
    if voltage is None:
        border_style = "cyan"
        voltage_text = "Waiting…"
    else:
        gauge = battery_gauge(voltage)
        border_style = battery_status_style(gauge.status)
        voltage_text = f"{voltage:.3f} V"

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=20, style="dim")
    details.add_column()
    details.add_row("Test phase", Text(phase, style="bold cyan"))
    details.add_row(
        "Altitude",
        "Waiting…" if altitude is None else f"{altitude:.2f} m",
    )
    details.add_row(
        "Vertical speed",
        "Waiting…" if vertical_speed is None else f"{vertical_speed:+.2f} m/s",
    )
    details.add_row(
        "Battery",
        voltage_text
        if minimum_voltage is None
        else f"{voltage_text} · {minimum_voltage:.3f} V minimum",
    )
    details.add_row("Elapsed", f"{elapsed:.1f} seconds")
    details.add_row("Radio", uri)

    if summary is not None:
        status_style = {
            "HEALTHY": "bold green",
            "MARGINAL": "bold yellow",
            "INSUFFICIENT": "bold red",
        }[summary.status]
        details.add_row("Result", Text(summary.status, style=status_style))
        details.add_row(
            "Hover headroom",
            f"{summary.hover_headroom_percent:.1f}%",
        )
        details.add_row(
            "Motor imbalance",
            f"{summary.motor_imbalance_percent:.1f}%",
        )
        details.add_row(
            "Battery sag",
            f"{summary.voltage_drop:.3f} V",
        )
        details.add_row(
            "Climb response",
            (
                f"+{summary.climb_gain:.2f} m · "
                f"{summary.peak_vertical_speed:+.2f} m/s peak"
            ),
        )
        details.add_row(
            "Motor saturation",
            f"{summary.saturation_percent:.1f}% of samples",
        )
        details.add_row("Maximum tilt", f"{summary.maximum_tilt:.1f}°")

    return Panel(
        Group(
            build_battery_meter(voltage),
            Text(""),
            details,
            Text(""),
            _motor_table(motors, requested_motors),
            Text(""),
            Text(message),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Motor and Power Test",
        subtitle="[dim]Stock stabilization · Return: land early[/dim]",
        border_style=border_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
