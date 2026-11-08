"""Shared flight boundaries and command limiting."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite

from bumblebee.config import (
    FLIGHT_TELEMETRY_TIMEOUT_SECONDS,
    MAX_FLIGHT_ALTITUDE_METERS,
    MAX_FLIGHT_DISTANCE_METERS,
    MAX_HORIZONTAL_SPEED_METERS_PER_SECOND,
    MAX_VERTICAL_SPEED_METERS_PER_SECOND,
)


@dataclass(frozen=True)
class FlightSafetyLimits:
    max_altitude: float = MAX_FLIGHT_ALTITUDE_METERS
    max_distance: float = MAX_FLIGHT_DISTANCE_METERS
    max_horizontal_speed: float = MAX_HORIZONTAL_SPEED_METERS_PER_SECOND
    max_vertical_speed: float = MAX_VERTICAL_SPEED_METERS_PER_SECOND
    telemetry_timeout: float = FLIGHT_TELEMETRY_TIMEOUT_SECONDS


@dataclass(frozen=True)
class PositionSafetyCheck:
    horizontal_distance: float
    violation: str | None


DEFAULT_FLIGHT_SAFETY_LIMITS = FlightSafetyLimits()


def check_position_safety(
    *,
    x: float,
    y: float,
    altitude: float,
    limits: FlightSafetyLimits = DEFAULT_FLIGHT_SAFETY_LIMITS,
) -> PositionSafetyCheck:
    values = (x, y, altitude)
    if not all(isfinite(value) for value in values):
        return PositionSafetyCheck(0.0, "Position telemetry became invalid")

    horizontal_distance = hypot(x, y)
    if altitude > limits.max_altitude:
        return PositionSafetyCheck(
            horizontal_distance,
            (
                f"Altitude reached {altitude:.2f} m, above the "
                f"{limits.max_altitude:.2f} m safety ceiling"
            ),
        )
    if horizontal_distance > limits.max_distance:
        return PositionSafetyCheck(
            horizontal_distance,
            (
                f"Distance from takeoff reached {horizontal_distance:.2f} m, above "
                f"the {limits.max_distance:.2f} m safety radius"
            ),
        )

    return PositionSafetyCheck(horizontal_distance, None)


def limit_velocity(
    *,
    x: float,
    y: float,
    z: float,
    limits: FlightSafetyLimits = DEFAULT_FLIGHT_SAFETY_LIMITS,
) -> tuple[float, float, float]:
    """Clamp a requested velocity while preserving its horizontal direction."""
    if not all(isfinite(value) for value in (x, y, z)):
        return 0.0, 0.0, 0.0

    horizontal_speed = hypot(x, y)
    if horizontal_speed > limits.max_horizontal_speed:
        scale = limits.max_horizontal_speed / horizontal_speed
        x *= scale
        y *= scale

    z = max(-limits.max_vertical_speed, min(z, limits.max_vertical_speed))
    return x, y, z
