from unittest import TestCase

from bumblebee.battery import battery_gauge
from bumblebee.models import BatteryStatus


class BatteryGaugeTests(TestCase):
    def test_percentage_is_clamped_to_valid_range(self) -> None:
        self.assertEqual(0, battery_gauge(3.0).percentage)
        self.assertEqual(100, battery_gauge(4.5).percentage)

    def test_status_changes_at_safety_thresholds(self) -> None:
        self.assertEqual(BatteryStatus.LAND, battery_gauge(3.50).status)
        self.assertEqual(BatteryStatus.LOW, battery_gauge(3.60).status)
        self.assertEqual(BatteryStatus.OK, battery_gauge(3.80).status)

    def test_client_style_percentage_is_voltage_based(self) -> None:
        self.assertEqual(54, battery_gauge(3.82).percentage)
