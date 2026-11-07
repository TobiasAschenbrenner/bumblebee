"""Camera streaming, decoding, and recording for the AI-deck."""

from bumblebee.camera.protocol import (
    AIdeckCameraClient,
    CameraEncoding,
    CameraFrame,
    CameraStreamError,
)

__all__ = [
    "AIdeckCameraClient",
    "CameraEncoding",
    "CameraFrame",
    "CameraStreamError",
]
