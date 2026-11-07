"""Terminal dashboard for the AI-deck camera viewer."""

from __future__ import annotations

from pathlib import Path

from rich import box
from rich.console import Group
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from bumblebee.camera.protocol import CameraFrame


def build_camera_dashboard(
    *,
    host: str,
    port: int,
    status: str,
    frame: CameraFrame | None,
    frames_per_second: float,
    frame_age_seconds: float | None,
    frames_received: int,
    frames_skipped: int,
    recording: bool,
    saved_frames: int,
    recording_directory: Path | None,
) -> Panel:
    waiting = frame is None
    status_style = "yellow" if waiting or "stalled" in status else "green"

    details = Table.grid(expand=True, padding=(0, 1))
    details.add_column(width=20, style="dim")
    details.add_column()
    details.add_row("Connection", Text(status, style=status_style))
    details.add_row("AI-deck", f"{host}:{port}")
    details.add_row(
        "Image",
        "Waiting for first frame"
        if waiting
        else f"{frame.width} × {frame.height} · {frame.encoding_name}",
    )
    details.add_row("Frame rate", f"{frames_per_second:.1f} FPS")
    details.add_row(
        "Frame transfer",
        "—" if waiting else f"{frame.transfer_seconds * 1000:.0f} ms",
    )
    details.add_row(
        "Latest frame",
        "—" if frame_age_seconds is None else f"{frame_age_seconds:.2f} s ago",
    )
    details.add_row("Frames received", str(frames_received))
    details.add_row("Viewer skipped", str(frames_skipped))
    details.add_row(
        "Recording",
        Text(
            f"● ON · {saved_frames} saved" if recording else "○ OFF",
            style="bold red" if recording else "dim",
        ),
    )
    if recording_directory is not None:
        details.add_row("Saved in", str(recording_directory))

    help_text = Text.assemble(
        ("R", "bold cyan"),
        "  start/stop recording     ",
        ("Q or Esc", "bold cyan"),
        "  close viewer",
    )
    return Panel(
        Group(details, Text(""), help_text),
        title="[bold yellow]Bumblebee 🐝[/bold yellow] · AI-deck Camera",
        subtitle="[dim]Camera only · no motor commands[/dim]",
        border_style=status_style,
        box=box.ROUNDED,
        padding=(1, 2),
    )
