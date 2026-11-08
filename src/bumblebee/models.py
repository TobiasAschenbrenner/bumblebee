"""Data shared between hardware, safety, flight, and presentation layers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class BatteryStatus(str, Enum):
    OK = "OK"
    LOW = "LOW"
    LAND = "LAND"


@dataclass(frozen=True)
class BatteryGauge:
    voltage: float
    percentage: int
    status: BatteryStatus


@dataclass(frozen=True)
class Telemetry:
    voltage: float
    power_state: int
    supervisor_info: int | None
    downward_range_mm: int


@dataclass(frozen=True)
class PreflightSnapshot:
    flow_deck_attached: bool
    ai_deck_attached: bool
    firmware_low_voltage: float
    telemetry: Telemetry


@dataclass(frozen=True)
class FlightRequirements:
    flow_deck: bool = False
    ai_deck: bool = False
