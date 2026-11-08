"""Host heartbeat that makes a lost Acro radio link fail closed."""

from __future__ import annotations

import threading
from types import TracebackType
from typing import Any

from bumblebee.errors import SafetyError


class EmergencyStopWatchdog:
    """Repeat the Crazyflie emergency watchdog until landing is complete."""

    def __init__(self, connection: Any, interval: float = 0.25) -> None:
        self._connection = connection
        self._interval = interval
        self._stop = threading.Event()
        self._error: Exception | None = None
        self._thread: threading.Thread | None = None

    def __enter__(self) -> EmergencyStopWatchdog:
        self._send()
        self._thread = threading.Thread(
            target=self._run,
            name="bumblebee-emergency-watchdog",
            daemon=True,
        )
        self._thread.start()
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self._interval * 2)

    def assert_healthy(self) -> None:
        if self._error is not None:
            raise SafetyError(
                "The Acro safety heartbeat stopped; the onboard watchdog is "
                "cutting motor power."
            ) from self._error

    def _send(self) -> None:
        self._connection.cf.supervisor.send_emergency_stop_watchdog()

    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            try:
                self._send()
            except Exception as exc:  # noqa: BLE001 - cflib errors are generic.
                self._error = exc
                return
