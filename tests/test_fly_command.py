from contextlib import contextmanager
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch

from bumblebee.commands import fly
from bumblebee.models import FlightMode


class FlyCommandTests(TestCase):
    def test_confirmation_happens_without_an_open_radio_link(self) -> None:
        flight = Mock()
        flight.title = "Test flip"
        flight.requirements = Mock()
        flight.requires_acro = True
        flight.confirmation_word = "FLIP"
        settings = Mock()
        flight.settings_from.return_value = settings

        args = SimpleNamespace(
            list=False,
            flight="test",
            mode=None,
            uri=None,
        )
        parser = Mock()
        parser.parse_args.return_value = args
        snapshot = Mock()
        snapshot.telemetry.voltage = 3.9
        radio_open = False
        connections = [Mock(), Mock()]

        @contextmanager
        def connection_context(_uri: str):
            nonlocal radio_open
            self.assertFalse(radio_open)
            radio_open = True
            try:
                yield connections.pop(0)
            finally:
                radio_open = False

        def confirm(*_args: object, **_kwargs: object) -> str:
            self.assertFalse(radio_open)
            return "ACRO" if confirm.call_count == 1 else "FLIP"

        confirm.call_count = 0

        def counted_confirm(*args: object, **kwargs: object) -> str:
            confirm.call_count += 1
            return confirm(*args, **kwargs)

        with (
            patch.object(fly, "build_parser", return_value=parser),
            patch.object(fly, "FLIGHTS_BY_KEY", {"test": flight}),
            patch.object(fly, "resolve_radio_uri", return_value="radio://test"),
            patch.object(fly, "connected_crazyflie", connection_context),
            patch.object(
                fly,
                "collect_preflight_snapshot",
                return_value=snapshot,
            ) as collect,
            patch.object(fly, "require_safe_preflight") as require,
            patch.object(fly, "build_preflight_dashboard", return_value="panel"),
            patch.object(fly, "landing_voltage", return_value=3.5),
            patch.object(fly.Prompt, "ask", side_effect=counted_confirm),
        ):
            result = fly.main([])

        self.assertEqual(0, result)
        self.assertEqual(2, collect.call_count)
        self.assertEqual(2, require.call_count)
        flight.run.assert_called_once()
        context = flight.run.call_args.args[2]
        self.assertEqual(FlightMode.ACRO, context.mode)
