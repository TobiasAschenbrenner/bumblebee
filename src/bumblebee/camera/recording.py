"""Save camera frames as a small, replayable image dataset."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import cv2

from bumblebee.camera.images import Image
from bumblebee.camera.protocol import CameraFrame, CameraStreamError


class CameraRecorder:
    """Write PNG frames and newline-delimited metadata for one session."""

    def __init__(self, recordings_root: Path) -> None:
        self.recordings_root = recordings_root
        self.session_directory: Path | None = None
        self.saved_frames = 0

    def start(self) -> Path:
        if self.session_directory is not None:
            return self.session_directory

        timestamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
        session_directory = self.recordings_root / timestamp
        suffix = 1
        while session_directory.exists():
            session_directory = self.recordings_root / f"{timestamp}_{suffix}"
            suffix += 1

        (session_directory / "frames").mkdir(parents=True)
        self.session_directory = session_directory
        return session_directory

    def save(self, image: Image, frame: CameraFrame) -> Path:
        session_directory = self.start()
        frame_number = self.saved_frames + 1
        image_path = session_directory / "frames" / f"frame_{frame_number:06d}.png"
        if not cv2.imwrite(str(image_path), image):
            raise CameraStreamError(f"Could not save camera frame to {image_path}.")

        metadata = {
            "frame": frame_number,
            "file": str(image_path.relative_to(session_directory)),
            "captured_at": datetime.now().astimezone().isoformat(),
            "width": frame.width,
            "height": frame.height,
            "depth": frame.depth,
            "encoding": frame.encoding_name,
            "transfer_ms": round(frame.transfer_seconds * 1000, 2),
        }
        with (session_directory / "metadata.jsonl").open("a", encoding="utf-8") as file:
            file.write(json.dumps(metadata) + "\n")

        self.saved_frames = frame_number
        return image_path
