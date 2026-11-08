from argparse import ArgumentParser
from math import hypot
from unittest import TestCase

from bumblebee.flights.hover import HoverFlight
from bumblebee.safety import (
    DEFAULT_FLIGHT_SAFETY_LIMITS,
    check_position_safety,
    limit_velocity,
)


class FlightSafetyTests(TestCase):
    def test_configured_flight_boundaries_are_one_and_a_half_metres(self) -> None:
        self.assertEqual(1.5, DEFAULT_FLIGHT_SAFETY_LIMITS.max_altitude)
        self.assertEqual(1.5, DEFAULT_FLIGHT_SAFETY_LIMITS.max_distance)

    def test_altitude_above_ceiling_is_rejected(self) -> None:
        check = check_position_safety(x=0.0, y=0.0, altitude=1.51)

        self.assertIn("safety ceiling", check.violation or "")

    def test_position_outside_takeoff_radius_is_rejected(self) -> None:
        check = check_position_safety(x=1.1, y=1.1, altitude=0.5)

        self.assertGreater(check.horizontal_distance, 1.5)
        self.assertIn("safety radius", check.violation or "")

    def test_velocity_is_limited_without_changing_direction(self) -> None:
        x, y, z = limit_velocity(x=0.3, y=0.4, z=-0.4)

        limits = DEFAULT_FLIGHT_SAFETY_LIMITS
        self.assertAlmostEqual(limits.max_horizontal_speed, hypot(x, y))
        self.assertAlmostEqual(-limits.max_vertical_speed, z)
        self.assertAlmostEqual(0.75, x / y)

    def test_invalid_velocity_request_stops_motion(self) -> None:
        self.assertEqual(
            (0.0, 0.0, 0.0),
            limit_velocity(x=float("nan"), y=0.1, z=0.1),
        )

    def test_hover_target_accepts_safety_ceiling(self) -> None:
        parser = ArgumentParser()
        HoverFlight().add_arguments(parser)

        settings = HoverFlight().settings_from(parser.parse_args(["--height", "1.5"]))

        self.assertEqual(1.5, settings.height)

    def test_position_violation_requests_landing(self) -> None:
        phase, message, should_land = HoverFlight._landing_decision(
            voltage=3.9,
            power_code=0,
            supervisor_info=1 << 3,
            elapsed=1.0,
            duration=5.0,
            landing_voltage=3.5,
            position_violation="Altitude exceeded the safety ceiling",
        )

        self.assertTrue(should_land)
        self.assertIn("boundary", phase)
        self.assertIn("safety ceiling", message)
