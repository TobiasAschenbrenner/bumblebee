"""Configured Crazyradio link and connection lifecycle."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from contextlib import contextmanager

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from bumblebee.config import DEFAULT_RADIO_URI, cache_directory
from bumblebee.errors import SafetyError


def resolve_radio_uri(explicit_uri: str | None = None) -> str:
    uri = explicit_uri or DEFAULT_RADIO_URI
    if not uri.startswith("radio://"):
        raise SafetyError("Only a radio:// URI is permitted for these tools.")

    cflib.crtp.init_drivers()
    return uri


@contextmanager
def connected_crazyflie(uri: str) -> Iterator[SyncCrazyflie]:
    cache = cache_directory()
    cache.mkdir(parents=True, exist_ok=True)
    crazyflie = Crazyflie(rw_cache=str(cache))

    # Bumblebee does not consume cflib's optional latency measurements. Their
    # 10 Hz ping worker can fill the radio queue after a link interruption and
    # then tries to join itself from its own error callback.
    crazyflie.link_statistics.start = lambda: None
    crazyflie.link_statistics.stop = lambda: None

    connection = SyncCrazyflie(uri, cf=crazyflie)
    cflib_logger = logging.getLogger("cflib.crazyflie")
    logger_was_disabled = cflib_logger.disabled
    try:
        cflib_logger.disabled = True
        try:
            connection.open_link()
        finally:
            cflib_logger.disabled = logger_was_disabled
    except Exception as exc:
        details = str(exc).splitlines()[0]
        raise SafetyError(f"Could not connect to {uri}: {details}") from exc

    try:
        yield connection
    finally:
        connection.close_link()
