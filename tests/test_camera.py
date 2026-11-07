import struct
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import Mock, patch

import numpy as np

from bumblebee.camera.images import decode_frame
from bumblebee.camera.protocol import (
    AIdeckCameraClient,
    CameraEncoding,
    CameraFrame,
    CameraStreamError,
)
from bumblebee.camera.recording import CameraRecorder


def _packet(payload: bytes) -> bytes:
    return struct.pack("<HBB", len(payload) + 2, 0, 0) + payload


def _stream_bytes(
    *,
    width: int = 2,
    height: int = 2,
    encoding: CameraEncoding = CameraEncoding.RAW_BAYER,
    image: bytes = b"\x01\x02\x03\x04",
) -> bytes:
    header = struct.pack(
        "<BHHBBI",
        0xBC,
        width,
        height,
        1,
        int(encoding),
        len(image),
    )
    return _packet(header) + _packet(image[:2]) + _packet(image[2:])


class CameraProtocolTests(TestCase):
    @staticmethod
    def _client(stream_data: bytes, chunk_size: int = 3) -> AIdeckCameraClient:
        connection = Mock()
        remaining = bytearray(stream_data)

        def recv(size: int) -> bytes:
            count = min(size, chunk_size, len(remaining))
            chunk = bytes(remaining[:count])
            del remaining[:count]
            return chunk

        connection.recv.side_effect = recv
        with patch(
            "bumblebee.camera.protocol.socket.create_connection",
            return_value=connection,
        ):
            client = AIdeckCameraClient()
            client.connect()
        return client

    def test_reads_fragmented_raw_frame(self) -> None:
        client = self._client(_stream_bytes())

        frame = client.read_frame()

        self.assertEqual((2, 2, 1), (frame.width, frame.height, frame.depth))
        self.assertEqual(CameraEncoding.RAW_BAYER, frame.encoding)
        self.assertEqual(b"\x01\x02\x03\x04", frame.data)

    def test_rejects_raw_frame_with_inconsistent_size(self) -> None:
        stream = _stream_bytes(image=b"\x01\x02\x03")
        client = self._client(stream)

        with self.assertRaisesRegex(CameraStreamError, "does not match"):
            client.read_frame()

    def test_connection_error_has_wifi_guidance(self) -> None:
        with (
            patch(
                "bumblebee.camera.protocol.socket.create_connection",
                side_effect=TimeoutError("timed out"),
            ),
            self.assertRaisesRegex(CameraStreamError, "AI-deck Wi-Fi network"),
        ):
            AIdeckCameraClient().connect()


class CameraImageTests(TestCase):
    def test_decodes_raw_bayer_frame_for_preview(self) -> None:
        frame = CameraFrame(
            width=4,
            height=4,
            depth=1,
            encoding=CameraEncoding.RAW_BAYER,
            data=bytes(range(16)),
            received_at=time.monotonic(),
            transfer_seconds=0.02,
        )

        image = decode_frame(frame)

        self.assertEqual((4, 4, 3), image.shape)

    def test_recorder_writes_image_and_metadata(self) -> None:
        frame = CameraFrame(
            width=4,
            height=4,
            depth=1,
            encoding=CameraEncoding.RAW_BAYER,
            data=bytes(range(16)),
            received_at=time.monotonic(),
            transfer_seconds=0.02,
        )
        image = np.zeros((4, 4, 3), dtype=np.uint8)

        with TemporaryDirectory() as temporary_directory:
            recorder = CameraRecorder(Path(temporary_directory))
            image_path = recorder.save(image, frame)

            self.assertTrue(image_path.exists())
            self.assertEqual(1, recorder.saved_frames)
            assert recorder.session_directory is not None
            metadata = recorder.session_directory / "metadata.jsonl"
            self.assertIn('"encoding": "Raw Bayer"', metadata.read_text())
