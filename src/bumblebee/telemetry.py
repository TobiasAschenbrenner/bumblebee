"""Read Crazyflie parameters, telemetry, and supervisor state."""

from __future__ import annotations

from contextlib import suppress
from queue import Empty, Queue
from statistics import fmean
from typing import Any, cast

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from bumblebee.config import TELEMETRY_SAMPLE_COUNT, TELEMETRY_WARMUP_SAMPLES
from bumblebee.errors import SafetyError
from bumblebee.models import Telemetry

POWER_STATE_NAMES = {
    0: "battery",
    1: "charging",
    2: "charged while connected to USB",
    3: "low power",
    4: "shutdown",
}


class TimedTelemetryLogger:
    """Yield log samples while turning a stalled stream into a safety error."""

    _DISCONNECTED = object()

    def __init__(self, connection: Any, config: LogConfig, timeout: float) -> None:
        self._connection = connection
        self._config = config
        self._timeout = timeout
        self._queue: Queue[object] = Queue()
        self._running = False

    def __enter__(self) -> TimedTelemetryLogger:  # noqa: PYI034 - Python 3.10.
        crazyflie = self._connection.cf
        crazyflie.disconnected.add_callback(self._on_disconnected)
        try:
            crazyflie.log.add_config(self._config)
            self._config.data_received_cb.add_callback(self._on_data)
            self._config.start()
        except Exception:
            with suppress(Exception):
                self._config.stop()
            with suppress(Exception):
                self._config.delete()
            with suppress(Exception):
                self._config.data_received_cb.remove_callback(self._on_data)
            with suppress(Exception):
                crazyflie.disconnected.remove_callback(self._on_disconnected)
            raise
        self._running = True
        return self

    def __exit__(self, *_args: object) -> None:
        if not self._running:
            return
        crazyflie = self._connection.cf
        with suppress(Exception):
            self._config.stop()
        with suppress(Exception):
            self._config.delete()
        with suppress(Exception):
            self._config.data_received_cb.remove_callback(self._on_data)
        with suppress(Exception):
            crazyflie.disconnected.remove_callback(self._on_disconnected)
        self._running = False

    def __iter__(self) -> TimedTelemetryLogger:
        return self

    def __next__(self) -> tuple[int, dict[str, Any], Any]:
        try:
            sample = self._queue.get(timeout=self._timeout)
        except Empty as exc:
            raise SafetyError(
                f"No flight telemetry arrived for {self._timeout:.1f} seconds."
            ) from exc
        if sample is self._DISCONNECTED:
            raise SafetyError("The radio link disconnected during flight.")
        return cast(tuple[int, dict[str, Any], Any], sample)

    def _on_data(self, timestamp: int, data: dict[str, Any], logblock: Any) -> None:
        self._queue.put((timestamp, data, logblock))

    def _on_disconnected(self, _uri: str) -> None:
        self._queue.put(self._DISCONNECTED)


def power_state_name(state: int) -> str:
    return POWER_STATE_NAMES.get(state, "unknown")


def read_deck_flag(connection: Any, parameter: str) -> bool:
    try:
        return int(connection.cf.param.get_value(parameter)) != 0
    except (KeyError, TypeError, ValueError) as exc:
        raise SafetyError(
            f"Firmware does not expose {parameter}. "
            "Update the Crazyflie firmware first."
        ) from exc


def read_low_voltage_setting(connection: Any) -> float:
    try:
        return float(connection.cf.param.get_value("pm.lowVoltage"))
    except (KeyError, TypeError, ValueError) as exc:
        raise SafetyError(
            "Firmware does not expose pm.lowVoltage. "
            "Update the Crazyflie firmware first."
        ) from exc


def read_optional_parameter_int(connection: Any, parameter: str) -> int | None:
    try:
        return int(connection.cf.param.get_value(parameter), 0)
    except (KeyError, TypeError, ValueError):
        return None


def read_telemetry(connection: Any) -> Telemetry:
    config = LogConfig(name="Preflight telemetry", period_in_ms=100)
    config.add_variable("pm.vbat", "float")
    config.add_variable("pm.state", "int8_t")
    config.add_variable("range.zrange", "uint16_t")

    log_toc = connection.cf.log.toc
    has_supervisor = log_toc.get_element_by_complete_name("supervisor.info") is not None

    if has_supervisor:
        config.add_variable("supervisor.info", "uint16_t")

    with SyncLogger(connection, config) as logger:
        for _ in range(TELEMETRY_WARMUP_SAMPLES):
            next(logger)
        samples = [next(logger)[1] for _ in range(TELEMETRY_SAMPLE_COUNT)]

    latest = samples[-1]
    return Telemetry(
        voltage=float(fmean(float(sample["pm.vbat"]) for sample in samples)),
        power_state=int(latest["pm.state"]),
        supervisor_info=(
            int(latest["supervisor.info"]) if "supervisor.info" in latest else None
        ),
        downward_range_mm=round(
            fmean(int(sample["range.zrange"]) for sample in samples)
        ),
    )


def describe_supervisor(info: int) -> list[str]:
    labels = (
        (0, "can be armed"),
        (1, "armed"),
        (2, "auto-arm enabled"),
        (3, "ready to fly"),
        (4, "flying"),
        (5, "tumbled"),
        (6, "locked"),
        (7, "crashed"),
        (8, "high-level control active"),
        (9, "trajectory finished"),
        (10, "high-level control idle"),
        (11, "deck fault"),
    )
    return [label for bit, label in labels if info & (1 << bit)]


def assert_supervisor_safe(info: int) -> None:
    if any(info & (1 << bit) for bit in (5, 6, 7, 11)):
        states = ", ".join(describe_supervisor(info))
        raise SafetyError(f"The supervisor reports an unsafe state: {states}.")

    if not info & (1 << 3):
        states = ", ".join(describe_supervisor(info)) or "no ready flags"
        raise SafetyError(f"The Crazyflie is not ready to fly ({states}).")
