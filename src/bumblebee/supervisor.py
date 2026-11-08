"""Compatibility helpers for the pinned Crazyflie supervisor protocol."""

from __future__ import annotations

import warnings
from collections.abc import Callable
from typing import Any

_LEGACY_WARNING = (
    r"The supervisor subsystem requires CRTP protocol version 12 or later\. "
    r"Connected Crazyflie reports version \d+\. Update your Crazyflie firmware\. "
    r"Using legacy fallback\."
)


def _using_supported_fallback(operation: Callable[[], Any]) -> Any:
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message=_LEGACY_WARNING,
            category=UserWarning,
        )
        return operation()


def request_arming(connection: Any, do_arm: bool) -> None:
    _using_supported_fallback(
        lambda: connection.cf.supervisor.send_arming_request(do_arm)
    )


def send_emergency_stop_watchdog(connection: Any) -> None:
    _using_supported_fallback(
        connection.cf.supervisor.send_emergency_stop_watchdog
    )
