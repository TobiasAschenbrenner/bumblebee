"""Continuously display battery telemetry without commanding motors."""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Sequence
from datetime import datetime
from statistics import fmean

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from rich import box
from rich.console import Console
from rich.live import Live
from rich.panel import Panel

from bumblebee.config import DEFAULT_RADIO_URI
from bumblebee.errors import SafetyError
from bumblebee.radio import connected_crazyflie, resolve_radio_uri
from bumblebee.telemetry import power_state_name
from bumblebee.ui.battery import build_battery_dashboard

CONSOLE = Console()


def _interval_seconds(value: str) -> float:
    interval = float(value)
    if not 0.2 <= interval <= 10.0:
        raise argparse.ArgumentTypeError("must be between 0.2 and 10 seconds")
    return interval


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--uri",
        help=f"override the configured radio URI (default: {DEFAULT_RADIO_URI})",
    )
    parser.add_argument(
        "--interval",
        type=_interval_seconds,
        default=1.0,
        help="display interval in seconds, 0.2-10 (default: 1)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    with CONSOLE.status("[cyan]Preparing Crazyradio PA…[/cyan]"):
        uri = resolve_radio_uri(args.uri)

    period_ms = int(round(args.interval * 1000 / 10) * 10)
    history_size = max(1, round(1.0 / args.interval))
    voltage_history: deque[float] = deque(maxlen=history_size)
    previous_state: int | None = None

    with connected_crazyflie(uri) as connection:
        config = LogConfig(name="Battery monitor", period_in_ms=period_ms)
        config.add_variable("pm.vbat", "float")
        config.add_variable("pm.state", "int8_t")

        initial = Panel(
            "Waiting for the first battery sample…",
            title="[bold yellow]Bumblebee 🐝[/bold yellow] · Battery Monitor",
            border_style="yellow",
            box=box.ROUNDED,
        )
        with (
            Live(initial, console=CONSOLE, refresh_per_second=4) as live,
            SyncLogger(connection, config) as logger,
        ):
            for _timestamp, data, _name in logger:
                state_code = int(data["pm.state"])
                if state_code != previous_state:
                    voltage_history.clear()
                    previous_state = state_code

                voltage_history.append(float(data["pm.vbat"]))
                live.update(
                    build_battery_dashboard(
                        uri=uri,
                        voltage=fmean(voltage_history),
                        power_state=power_state_name(state_code),
                        updated_at=datetime.now().strftime("%H:%M:%S"),
                    ),
                    refresh=True,
                )

    return 0


def cli() -> int:
    try:
        return main()
    except SafetyError as exc:
        CONSOLE.print(f"\n[bold red]Battery monitor stopped:[/bold red] {exc}")
        return 2
    except KeyboardInterrupt:
        CONSOLE.print(
            "\n[green]Battery monitor stopped.[/green] No motor commands were sent."
        )
        return 0


if __name__ == "__main__":
    raise SystemExit(cli())
