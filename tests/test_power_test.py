from unittest import TestCase

from bumblebee.flights.power_test import PowerTestFlight, evaluate_power_test


class PowerTestFlightTests(TestCase):
    def test_power_test_uses_safe_mode_and_stock_firmware(self) -> None:
        flight = PowerTestFlight()

        self.assertFalse(flight.requires_acro)
        self.assertEqual("TEST", flight.confirmation_word)
        self.assertIsNone(flight.requirements.minimum_acro_firmware_version)

    def test_healthy_measurement_allows_instrumented_tuning(self) -> None:
        summary = evaluate_power_test(
            preflight_voltage=4.10,
            minimum_voltage=3.78,
            hover_requested_means=(45_000, 46_000, 45_500, 46_500),
            climb_gain=0.14,
            peak_vertical_speed=0.15,
            saturation_samples=0,
            sample_count=100,
            maximum_tilt=3.0,
        )

        self.assertEqual("HEALTHY", summary.status)

    def test_narrow_hover_headroom_is_insufficient(self) -> None:
        summary = evaluate_power_test(
            preflight_voltage=4.10,
            minimum_voltage=3.70,
            hover_requested_means=(60_000, 60_500, 61_000, 61_500),
            climb_gain=0.14,
            peak_vertical_speed=0.15,
            saturation_samples=0,
            sample_count=100,
            maximum_tilt=3.0,
        )

        self.assertEqual("INSUFFICIENT", summary.status)
        self.assertIn("headroom", summary.message)

    def test_repeated_saturation_is_insufficient(self) -> None:
        summary = evaluate_power_test(
            preflight_voltage=4.10,
            minimum_voltage=3.72,
            hover_requested_means=(45_000, 46_000, 45_500, 46_500),
            climb_gain=0.14,
            peak_vertical_speed=0.15,
            saturation_samples=3,
            sample_count=100,
            maximum_tilt=3.0,
        )

        self.assertEqual("INSUFFICIENT", summary.status)
        self.assertIn("saturated", summary.message)

    def test_high_battery_sag_is_insufficient(self) -> None:
        summary = evaluate_power_test(
            preflight_voltage=4.15,
            minimum_voltage=3.55,
            hover_requested_means=(45_000, 46_000, 45_500, 46_500),
            climb_gain=0.14,
            peak_vertical_speed=0.15,
            saturation_samples=0,
            sample_count=100,
            maximum_tilt=3.0,
        )

        self.assertEqual("INSUFFICIENT", summary.status)
        self.assertIn("sag", summary.message)
