"""Project-wide operating limits and hardware defaults."""

from __future__ import annotations

import os
import sys
from pathlib import Path

DEFAULT_RADIO_URI = "radio://0/80/2M"
MINIMUM_TAKEOFF_VOLTAGE = 3.70
MINIMUM_TAKEOFF_PERCENTAGE = 20
BATTERY_EMPTY_VOLTAGE = 3.40
BATTERY_FULL_VOLTAGE = 4.18
BATTERY_LAND_VOLTAGE = 3.50
BATTERY_LOW_VOLTAGE = 3.65
TELEMETRY_WARMUP_SAMPLES = 5
TELEMETRY_SAMPLE_COUNT = 20


def cache_directory() -> Path:
    override = os.environ.get("BUMBLEBEE_CACHE_DIR")
    if override:
        return Path(override).expanduser()

    if sys.platform == "darwin":
        return Path.home() / "Library" / "Caches" / "bumblebee"

    root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return root / "bumblebee"
