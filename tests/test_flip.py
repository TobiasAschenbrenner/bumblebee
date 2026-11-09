from argparse import Namespace
from unittest import TestCase
from unittest.mock import Mock, call, patch

from bumblebee.flights.flip import (
    FLIP_REQUEST_MAGIC,
    LOW_VOLTAGE_CONFIRM_SECONDS,
    MINIMUM_RECOVERY_ALTITUDE,
    MINIMUM_RECOVERY_VERTICAL_SPEED,
    MINIMUM_FLIP_VOLTAGE,
    MAXIMUM_STAGING_DISTANCE,
    PHASED_PROFILE,
    FlipFlight,
    SustainedLowVoltageGuard,
    recovery_violation,
    staging_violation,
)
from bumblebee.models import FlightMode


class FlipFlightTests(TestCase):
    def test_flip_requires_acro_firmware_and_mode(self) -> None:
        flight = FlipFlight()

        self.assertTrue(flight.requires_acro)
        self.assertEqual(7, flight.requirements.minimum_acro_firmware_version)
        self.assertEqual("FLIP", flight.confirmation_word)
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

        FlipFlight._trigger(connection, "backward", PHASED_PROFILE)

        connection.cf.param.set_value.assert_has_calls(
            [
                call("bumblebee.boostMm", "200"),
                call("bumblebee.boostMs", "250"),
                call("bumblebee.brakeDeg", "270"),
                call("bumblebee.spinPct", "40"),
                call("bumblebee.coastPct", "20"),
                call("bumblebee.brakePct", "45"),
                call("bumblebee.recoverMs", "400"),
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

    def test_stable_recovery_is_accepted(self) -> None:
        self.assertIsNone(
            recovery_violation(
                altitude=MINIMUM_RECOVERY_ALTITUDE + 0.10,
                vertical_speed=0.0,
            )
        )

    def test_low_recovery_altitude_is_rejected(self) -> None:
        violation = recovery_violation(
            altitude=MINIMUM_RECOVERY_ALTITUDE - 0.01,
            vertical_speed=0.0,
        )

        self.assertIn("altitude", violation or "")

    def test_fast_descent_after_recovery_is_rejected(self) -> None:
        violation = recovery_violation(
            altitude=MINIMUM_RECOVERY_ALTITUDE + 0.10,
            vertical_speed=MINIMUM_RECOVERY_VERTICAL_SPEED - 0.01,
        )

        self.assertIn("vertical speed", violation or "")

    def test_single_low_voltage_sample_does_not_stop_staging(self) -> None:
        guard = SustainedLowVoltageGuard()
        low_voltage = MINIMUM_FLIP_VOLTAGE - 0.05

        self.assertFalse(guard.update(voltage=low_voltage, now=10.0))
        self.assertFalse(guard.update(voltage=3.80, now=10.1))
        self.assertIsNone(guard.below_since)

    def test_sustained_low_voltage_stops_staging(self) -> None:
        guard = SustainedLowVoltageGuard()
        low_voltage = MINIMUM_FLIP_VOLTAGE - 0.05

        self.assertFalse(guard.update(voltage=low_voltage, now=10.0))
        self.assertFalse(
            guard.update(
                voltage=low_voltage,
                now=10.0 + LOW_VOLTAGE_CONFIRM_SECONDS - 0.01,
            )
        )
        self.assertTrue(
            guard.update(
                voltage=low_voltage,
                now=10.0 + LOW_VOLTAGE_CONFIRM_SECONDS,
            )
        )
