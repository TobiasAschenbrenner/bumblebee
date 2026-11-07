"""Decode AI-deck frames for display and computer-vision work."""

from __future__ import annotations

import cv2
import numpy as np
from numpy.typing import NDArray

from bumblebee.camera.protocol import CameraEncoding, CameraFrame, CameraStreamError

Image = NDArray[np.uint8]


def decode_frame(frame: CameraFrame) -> Image:
    """Return an OpenCV BGR image for either supported stream encoding."""
    encoded = np.frombuffer(frame.data, dtype=np.uint8)
    if frame.encoding is CameraEncoding.JPEG:
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise CameraStreamError("OpenCV could not decode the AI-deck JPEG frame.")
        return image

    if frame.depth == 1:
        bayer = encoded.reshape((frame.height, frame.width))
        return cv2.cvtColor(bayer, cv2.COLOR_BayerBG2BGR)
    if frame.depth == 3:
        return encoded.reshape((frame.height, frame.width, 3))
    if frame.depth == 4:
        bgra = encoded.reshape((frame.height, frame.width, 4))
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

    raise CameraStreamError(f"Unsupported image depth: {frame.depth}.")
