from argparse import Namespace
from unittest import TestCase
from unittest.mock import Mock, call, patch

from bumblebee.flights.flip import (
    FLIP_REQUEST_MAGIC,
    MAXIMUM_STAGING_DISTANCE,
    FlipFlight,
    staging_violation,
)
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

    def test_slow_reported_drift_is_accepted_while_staging(self) -> None:
        self.assertIsNone(
            staging_violation(
                horizontal_distance=0.32,
                horizontal_speed=0.06,
                elapsed=3.0,
                settle_seconds=3.0,
            )
        )

    def test_excessive_staging_distance_aborts_before_flip(self) -> None:
        violation = staging_violation(
            horizontal_distance=MAXIMUM_STAGING_DISTANCE + 0.01,
            horizontal_speed=0.06,
            elapsed=3.0,
            settle_seconds=3.0,
        )

        self.assertIn("drifted", violation or "")

    def test_unstable_staging_eventually_times_out(self) -> None:
        violation = staging_violation(
            horizontal_distance=0.2,
            horizontal_speed=0.2,
            elapsed=12.0,
            settle_seconds=3.0,
        )

        self.assertIn("staging hover", violation or "")
