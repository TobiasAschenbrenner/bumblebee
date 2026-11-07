"""Human-readable battery calculations."""

from __future__ import annotations

from bumblebee.config import (
    BATTERY_EMPTY_VOLTAGE,
    BATTERY_FULL_VOLTAGE,
    BATTERY_LAND_VOLTAGE,
    BATTERY_LOW_VOLTAGE,
)
from bumblebee.models import BatteryGauge, BatteryStatus


def battery_gauge(voltage: float) -> BatteryGauge:
    span = BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE
    fraction = (voltage - BATTERY_EMPTY_VOLTAGE) / span
    percentage = round(min(1.0, max(0.0, fraction)) * 100)

    if voltage <= BATTERY_LAND_VOLTAGE:
        status = BatteryStatus.LAND
    elif voltage <= BATTERY_LOW_VOLTAGE:
        status = BatteryStatus.LOW
    else:
        status = BatteryStatus.OK

    return BatteryGauge(voltage=voltage, percentage=percentage, status=status)
