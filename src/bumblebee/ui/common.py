"""Reusable visual components for terminal dashboards."""

from __future__ import annotations

from rich.progress_bar import ProgressBar
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.models import BatteryStatus


def battery_status_style(status: BatteryStatus) -> str:
    return {
        BatteryStatus.OK: "bold green",
        BatteryStatus.LOW: "bold yellow",
        BatteryStatus.LAND: "bold red",
    }[status]


def build_battery_meter(voltage: float | None) -> Table:
    if voltage is None:
        percentage = 0
        style = "cyan"
        label = "waiting"
    else:
        gauge = battery_gauge(voltage)
        percentage = gauge.percentage
        style = battery_status_style(gauge.status)
        label = f"{percentage}% usable"

    meter = Table.grid(expand=True)
    meter.add_column(ratio=1)
    meter.add_column(width=14, justify="right")
    meter.add_row(
        ProgressBar(
            total=100,
            completed=percentage,
            style="grey35",
            complete_style=style,
            finished_style=style,
        ),
        Text(label, style=style),
    )
    return meter


def result_text(ok: bool, success: str, failure: str) -> Text:
    if ok:
        return Text.assemble(("✓ ", "bold green"), (success, "green"))
    return Text.assemble(("✗ ", "bold red"), (failure, "red"))
