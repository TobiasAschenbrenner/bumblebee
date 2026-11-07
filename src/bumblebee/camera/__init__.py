"""Camera streaming, decoding, and recording for the AI-deck."""

from bumblebee.camera.protocol import (
    AIdeckCameraClient,
    AIdeckUDPCameraClient,
    CameraEncoding,
    CameraFrame,
    CameraStreamError,
)

__all__ = [
    "AIdeckCameraClient",
    "AIdeckUDPCameraClient",
    "CameraEncoding",
    "CameraFrame",
    "CameraStreamError",
]
