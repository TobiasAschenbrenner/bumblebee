"""Run a read-only Crazyflie preflight diagnostic."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from rich.console import Console

from bumblebee.config import DEFAULT_RADIO_URI
from bumblebee.errors import SafetyError
from bumblebee.models import FlightRequirements
from bumblebee.preflight import collect_preflight_snapshot, preflight_issues
from bumblebee.radio import connected_crazyflie, resolve_radio_uri
from bumblebee.ui.diagnostics import build_diagnostics_dashboard

CONSOLE = Console()
DIAGNOSTIC_REQUIREMENTS = FlightRequirements(flow_deck=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--uri",
        help=f"override the configured radio URI (default: {DEFAULT_RADIO_URI})",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    uri = resolve_radio_uri(args.uri)

    with (
        CONSOLE.status(f"[cyan]Connecting to {uri} and reading telemetry…[/cyan]"),
        connected_crazyflie(uri) as connection,
    ):
        snapshot = collect_preflight_snapshot(connection)

    issues = preflight_issues(snapshot, DIAGNOSTIC_REQUIREMENTS)
    CONSOLE.print(
        build_diagnostics_dashboard(uri=uri, snapshot=snapshot, issues=issues)
    )
    return 2 if issues else 0


def cli() -> int:
    try:
        return main()
    except SafetyError as exc:
        CONSOLE.print(f"\n[bold red]Diagnostic stopped:[/bold red] {exc}")
        return 2
    except KeyboardInterrupt:
        CONSOLE.print(
            "\n[yellow]Diagnostic cancelled.[/yellow] No motor commands were sent."
        )
        return 130


if __name__ == "__main__":
    raise SystemExit(cli())
