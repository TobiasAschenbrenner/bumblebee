"""Run camera reception away from the viewer's main thread."""

from __future__ import annotations

import queue
import threading

from bumblebee.camera.protocol import (
    AIdeckCameraClient,
    CameraFrame,
    CameraStreamError,
    _StreamStopped,
)


class CameraStream:
    """Keep only the newest frame so the preview stays responsive."""

    def __init__(self, client: AIdeckCameraClient) -> None:
        self.client = client
        self.frames_received = 0
        self.frames_skipped = 0
        self.error: CameraStreamError | None = None
        self._frames: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._receive,
            name="bumblebee-camera",
            daemon=True,
        )
        self._thread.start()

    def latest(self, timeout: float = 0.05) -> CameraFrame | None:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty:
            return None

    def stop(self) -> None:
        self._stop_event.set()
        self.client.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _receive(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self.client.read_frame(self._stop_event)
            except _StreamStopped:
                return
            except CameraStreamError as exc:
                self.error = exc
                return

            self.frames_received += 1
            if self._frames.full():
                try:
                    self._frames.get_nowait()
                    self.frames_skipped += 1
                except queue.Empty:
                    pass
            self._frames.put_nowait(frame)
