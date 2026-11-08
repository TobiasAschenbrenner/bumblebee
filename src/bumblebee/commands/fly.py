"""Choose and run guarded Crazyflie flight routines."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from rich.console import Console
from rich.prompt import Prompt

from bumblebee.config import DEFAULT_RADIO_URI
from bumblebee.errors import SafetyError
from bumblebee.flights import AVAILABLE_FLIGHTS, FLIGHTS_BY_KEY
from bumblebee.flights.base import FlightContext, FlightRoutine
from bumblebee.models import FlightMode
from bumblebee.preflight import (
    collect_preflight_snapshot,
    landing_voltage,
    require_safe_preflight,
)
from bumblebee.radio import connected_crazyflie, resolve_radio_uri
from bumblebee.ui.flight import build_flight_selector, build_preflight_dashboard

CONSOLE = Console()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--flight",
        choices=tuple(FLIGHTS_BY_KEY),
        help="run one flight directly instead of opening the selector",
    )
    parser.add_argument(
        "--list", action="store_true", help="list flights without connecting"
    )
    parser.add_argument(
        "--mode",
        choices=tuple(mode.value for mode in FlightMode),
        help="flight mode (default: safe; acro requires a separate unlock)",
    )
    parser.add_argument(
        "--uri",
        help=f"override the configured radio URI (default: {DEFAULT_RADIO_URI})",
    )
    for flight in AVAILABLE_FLIGHTS:
        flight.add_arguments(parser)
    return parser


def _choose_flight() -> FlightRoutine | None:
    CONSOLE.print(build_flight_selector(AVAILABLE_FLIGHTS, interactive=True))
    choices = [str(number) for number in range(1, len(AVAILABLE_FLIGHTS) + 1)] + ["q"]
    selection = Prompt.ask("Select a flight", choices=choices, case_sensitive=False)
    if selection.lower() == "q":
        return None
    return AVAILABLE_FLIGHTS[int(selection) - 1]


def main(
    argv: Sequence[str] | None = None,
    *,
    default_flight: str | None = None,
) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        CONSOLE.print(build_flight_selector(AVAILABLE_FLIGHTS, interactive=False))
        return 0

    flight = FLIGHTS_BY_KEY.get(args.flight or default_flight or "")
    if flight is None:
        flight = _choose_flight()
        if flight is None:
            CONSOLE.print("[green]Closed without connecting.[/green]")
            return 0

    settings = flight.settings_from(args)
    requested_mode = FlightMode(args.mode) if args.mode else None
    if flight.requires_acro and requested_mode is FlightMode.SAFE:
        raise SafetyError(f"{flight.title} requires --mode acro.")
    if not flight.requires_acro and requested_mode is FlightMode.ACRO:
        raise SafetyError(
            f"{flight.title} is a Safe routine and cannot disable its boundaries."
        )
    mode = (
        FlightMode.ACRO
        if flight.requires_acro
        else requested_mode or FlightMode.SAFE
    )
    with CONSOLE.status("[cyan]Preparing Crazyradio PA…[/cyan]"):
        uri = resolve_radio_uri(args.uri)

    with connected_crazyflie(uri) as connection:
        with CONSOLE.status(f"[cyan]Reading preflight data from {uri}…[/cyan]"):
            snapshot = collect_preflight_snapshot(connection)
        require_safe_preflight(snapshot, flight.requirements)
        CONSOLE.print(
            build_preflight_dashboard(
                flight=flight,
                settings=settings,
                uri=uri,
                snapshot=snapshot,
                mode=mode,
            )
        )

        if mode is FlightMode.ACRO:
            acro_confirmation = Prompt.ask(
                "Type [bold red]ACRO[/bold red] exactly to disable the normal "
                "flight boundaries for this run",
                default="",
                show_default=False,
            )
            if acro_confirmation != "ACRO":
                CONSOLE.print(
                    "[yellow]Cancelled.[/yellow] No motor command was sent."
                )
                return 0

        confirmation_word = "FLIP" if flight.requires_acro else "FLY"
        confirmation = Prompt.ask(
            f"Type [bold]{confirmation_word}[/bold] exactly to arm, "
            "or press Return to cancel",
            default="",
            show_default=False,
        )
        if confirmation != confirmation_word:
            CONSOLE.print("[yellow]Cancelled.[/yellow] No motor command was sent.")
            return 0

        flight.run(
            connection,
            settings,
            FlightContext(
                console=CONSOLE,
                uri=uri,
                landing_voltage=landing_voltage(snapshot),
                mode=mode,
            ),
        )

    CONSOLE.print("[bold green]Landed, disarmed, and disconnected.[/bold green]")
    return 0


def _run_cli(default_flight: str | None = None) -> int:
    try:
        return main(default_flight=default_flight)
    except SafetyError as exc:
        CONSOLE.print(f"\n[bold red]Safety check stopped:[/bold red] {exc}")
        return 2
    except KeyboardInterrupt:
        CONSOLE.print(
            "\n[yellow]Interrupted.[/yellow] Landing/disarm cleanup was requested."
        )
        return 130


def cli() -> int:
    return _run_cli()


def hover_cli() -> int:
    return _run_cli(default_flight="hover")


if __name__ == "__main__":
    raise SystemExit(cli())
