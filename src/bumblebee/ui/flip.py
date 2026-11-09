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
    battery_live: bool,
    minimum_flip_voltage: float,
    rotation: float,
    raw_rotation: float | None,
    peak_pitch_rate: float | None,
    rotation_end_roll: float | None,
    rotation_end_pitch: float | None,
    recovery_altitude: float | None,
    recovery_vertical_speed: float | None,
    hover_thrust_ready: bool,
    hover_thrust: float | None,
    watchdog_active: bool,
    horizontal_distance: float | None,
    horizontal_speed: float | None,
    message: str,
    current_vertical_speed: float | None = None,
    minimum_altitude: float | None = None,
    minimum_voltage: float | None = None,
    maximum_motor_request: int | None = None,
    motor_saturation_percent: float | None = None,
    boost_end_altitude: float | None = None,
    brake_start_altitude: float | None = None,
    recovery_start_altitude: float | None = None,
    recovery_start_vertical_speed: float | None = None,
    brake_start_rotation: float | None = None,
    brake_end_rate: float | None = None,
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
    if current_vertical_speed is not None:
        details.add_row("Vertical speed", f"{current_vertical_speed:+.2f} m/s")
    if minimum_altitude is not None:
        details.add_row("Lowest altitude", f"{minimum_altitude:.2f} m")
    details.add_row(
        "Net rotation",
        f"{rotation:.0f}° / 335° target",
    )
    details.add_row(
        "Raw angular travel",
        "Waiting…" if raw_rotation is None else f"{raw_rotation:.0f}°",
    )
    details.add_row(
        "Peak pitch rate",
        "Waiting…"
        if peak_pitch_rate is None
        else f"{peak_pitch_rate:.0f}°/s",
    )
    details.add_row(
        "Rotation end attitude",
        "Waiting…"
        if rotation_end_roll is None or rotation_end_pitch is None
        else f"roll {rotation_end_roll:.0f}° · pitch {rotation_end_pitch:.0f}°",
    )
    details.add_row(
        "Recovery state",
        (
            "Waiting…"
            if recovery_altitude is None or recovery_vertical_speed is None
            else (
                f"{recovery_altitude:.2f} m · "
                f"{recovery_vertical_speed:+.2f} m/s vertical"
            )
        ),
    )
    phase_values: list[str] = []
    if boost_end_altitude is not None:
        phase_values.append(f"boost {boost_end_altitude:.2f} m")
    if brake_start_altitude is not None:
        phase_values.append(f"brake {brake_start_altitude:.2f} m")
    if recovery_start_altitude is not None:
        recovery_value = f"recovery {recovery_start_altitude:.2f} m"
        if recovery_start_vertical_speed is not None:
            recovery_value += f" at {recovery_start_vertical_speed:+.2f} m/s"
        phase_values.append(recovery_value)
    if phase_values:
        details.add_row("Phase altitudes", " · ".join(phase_values))
    if brake_start_rotation is not None or brake_end_rate is not None:
        brake_values: list[str] = []
        if brake_start_rotation is not None:
            brake_values.append(f"started at {brake_start_rotation:.0f}°")
        if brake_end_rate is not None:
            brake_values.append(f"ended at {brake_end_rate:.0f}°/s")
        details.add_row("Angular braking", " · ".join(brake_values))
    details.add_row(
        "Staging drift",
        "Waiting…"
        if horizontal_distance is None or horizontal_speed is None
        else f"{horizontal_distance:.2f} m away · {horizontal_speed:.2f} m/s",
    )
    details.add_row(
        "Hover thrust",
        Text(
            (
                "Learning…"
                if not hover_thrust_ready
                else (
                    "Learned"
                    if hover_thrust is None
                    else f"Learned · {hover_thrust:.0f}"
                )
            ),
            style="green" if hover_thrust_ready else "cyan",
        ),
    )
    details.add_row(
        "Radio-loss watchdog",
        Text("ACTIVE" if watchdog_active else "Waiting", style="bold red"),
    )
    details.add_row(
        "Battery",
        (
            "Waiting…"
            if voltage is None
            else f"{voltage:.3f} V · {'live' if battery_live else 'preflight'}"
        ),
    )
    if minimum_voltage is not None:
        details.add_row("Minimum battery", f"{minimum_voltage:.3f} V")
    if maximum_motor_request is not None:
        motor_percent = maximum_motor_request / 65_535 * 100.0
        details.add_row(
            "Peak motor request",
            f"{maximum_motor_request} · {motor_percent:.0f}%",
        )
    if motor_saturation_percent is not None:
        details.add_row(
            "Motor saturation",
            f"{motor_saturation_percent:.1f}% of samples",
        )
    details.add_row(
        "Acro start minimum",
        f"{minimum_flip_voltage:.3f} V under load",
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
