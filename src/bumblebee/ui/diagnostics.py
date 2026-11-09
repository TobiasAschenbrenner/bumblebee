"""Diagnostic results dashboard."""

from __future__ import annotations

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.battery import battery_gauge
from bumblebee.errors import SafetyError
from bumblebee.models import PreflightSnapshot
from bumblebee.telemetry import (
    assert_supervisor_safe,
    describe_supervisor,
    power_state_name,
)
from bumblebee.ui.common import (
    battery_status_style,
    build_battery_meter,
    result_text,
)


def _supervisor_result(snapshot: PreflightSnapshot) -> Text:
    info = snapshot.telemetry.supervisor_info
    if info is None:
        return result_text(False, "", "Unavailable · firmware update required")

    states = ", ".join(describe_supervisor(info)) or "no flags set"
    try:
        assert_supervisor_safe(info)
    except SafetyError:
        return result_text(False, "", states)
    return result_text(True, states, "")


def _summary(issues: list[str]) -> Group:
    if not issues:
        return Group(
            Text("READY FOR FLIGHT", style="bold green"),
            Text("All automated checks passed. No motor commands were sent."),
        )

    lines = [Text("NOT READY FOR FLIGHT", style="bold red")]
    lines.extend(Text(f"• {issue}", style="red") for issue in issues)
    return Group(*lines)


def build_diagnostics_dashboard(
    *,
    uri: str,
    snapshot: PreflightSnapshot,
    issues: list[str],
) -> Panel:
    telemetry = snapshot.telemetry
    gauge = battery_gauge(telemetry.voltage)
    battery_style = battery_status_style(gauge.status)
    on_battery = telemetry.power_state == 0

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=20, style="dim")
    details.add_column()
    details.add_row(
        "Flow deck V2",
        result_text(snapshot.flow_deck_attached, "Detected", "Not detected"),
    )
    details.add_row(
        "AI-deck 1.1",
        Text("● Detected · idle", style="cyan")
        if snapshot.ai_deck_attached
        else Text("○ Not detected", style="dim"),
    )
    details.add_row(
        "Acro firmware",
        (
            Text(f"● Version {snapshot.acro_firmware_version}", style="yellow")
            if snapshot.acro_firmware_version is not None
            else Text("○ Not installed · Safe flights only", style="dim")
        ),
    )
    if snapshot.acro_firmware_version is not None:
        details.add_row(
            "Acro motor stop",
            result_text(
                snapshot.acro_immediate_stop_ready is True,
                "Verified",
                "Not verified",
            ),
        )
    details.add_row(
        "Battery",
        Text(
            f"{telemetry.voltage:.3f} V · {gauge.percentage}% usable · "
            f"{gauge.status.value}",
            style=battery_style,
        ),
    )
    details.add_row(
        "Power source",
        result_text(
            on_battery,
            "Battery",
            power_state_name(telemetry.power_state),
        ),
    )
    details.add_row("Downward range", f"{telemetry.downward_range_mm} mm")
    details.add_row("Supervisor", _supervisor_result(snapshot))
    details.add_row("Firmware low limit", f"{snapshot.firmware_low_voltage:.2f} V")
    details.add_row("Radio", uri)

    border_style = "green" if not issues else "red"
    return Panel(
        Group(
            build_battery_meter(telemetry.voltage),
            Text(""),
            details,
            Text(""),
            _summary(issues),
        ),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · Preflight Diagnostic",
        subtitle="[dim]Read-only check · no motor commands[/dim]",
        border_style=border_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
