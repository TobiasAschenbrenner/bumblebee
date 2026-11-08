from argparse import Namespace
from unittest import TestCase
from unittest.mock import Mock, call, patch

from bumblebee.flights.flip import FLIP_REQUEST_MAGIC, FlipFlight
from bumblebee.models import FlightMode


class FlipFlightTests(TestCase):
    def test_flip_requires_acro_firmware_and_mode(self) -> None:
        flight = FlipFlight()

        self.assertTrue(flight.requires_acro)
        self.assertTrue(flight.requirements.acro_firmware)
        self.assertEqual(FlightMode.ACRO.value, "acro")

    def test_settings_are_isolated_from_hover_arguments(self) -> None:
        settings = FlipFlight().settings_from(
            Namespace(
                flip_height=1.0,
                flip_settle=3.0,
                flip_direction="backward",
            )
        )

        self.assertEqual(1.0, settings.height)
        self.assertEqual("backward", settings.direction)

    @patch("bumblebee.flights.flip.time.sleep")
    def test_trigger_sets_direction_before_magic_request(self, sleep: Mock) -> None:
        connection = Mock()

        FlipFlight._trigger(connection, "backward")

        connection.cf.param.set_value.assert_has_calls(
            [
                call("bumblebee.direction", "1"),
                call("bumblebee.flipRequest", str(FLIP_REQUEST_MAGIC)),
            ],
        )
        sleep.assert_called_once_with(0.1)
