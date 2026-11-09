from argparse import Namespace
from unittest import TestCase
from unittest.mock import Mock, call

from bumblebee.flights.lift_test import (
    LIFT_TEST_REQUEST_MAGIC,
    LiftTestFlight,
)


class LiftTestFlightTests(TestCase):
    def test_lift_test_keeps_safe_mode_and_rejects_v8(self) -> None:
        flight = LiftTestFlight()

        self.assertFalse(flight.requires_acro)
        self.assertEqual("LIFT", flight.confirmation_word)
        self.assertEqual(9, flight.requirements.minimum_acro_firmware_version)

    def test_settings_are_isolated_from_other_flights(self) -> None:
        settings = LiftTestFlight().settings_from(
            Namespace(lift_height=0.6, lift_settle=3.0)
        )

        self.assertEqual(0.6, settings.height)
        self.assertEqual(3.0, settings.settle_seconds)

    def test_trigger_sends_only_the_lift_test_magic_request(self) -> None:
        connection = Mock()

        LiftTestFlight._trigger(connection)

        connection.cf.param.set_value.assert_has_calls(
            [
                call(
                    "bumblebee.flipRequest",
                    str(LIFT_TEST_REQUEST_MAGIC),
                )
            ]
        )
