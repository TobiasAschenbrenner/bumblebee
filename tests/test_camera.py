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
    AIdeckUDPCameraClient,
    CameraEncoding,
    CameraFrame,
    CameraStreamError,
    UDPFrameAssembler,
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


class UDPFrameAssemblerTests(TestCase):
    @staticmethod
    def _header(image_size: int) -> bytes:
        payload = struct.pack(
            "<BHHBBI",
            0xBC,
            324,
            244,
            1,
            int(CameraEncoding.JPEG),
            image_size,
        )
        return _packet(payload)

    def test_reassembles_jpeg_from_datagrams(self) -> None:
        assembler = UDPFrameAssembler()
        image = b"\xff\xd8camera-data\xff\xd9"

        self.assertIsNone(assembler.add_datagram(self._header(len(image)), 1.0))
        self.assertIsNone(assembler.add_datagram(_packet(image[:5]), 1.1))
        frame = assembler.add_datagram(_packet(image[5:]), 1.2)

        assert frame is not None
        self.assertEqual(image, frame.data)
        self.assertEqual(CameraEncoding.JPEG, frame.encoding)
        self.assertAlmostEqual(0.2, frame.transfer_seconds)

    def test_new_header_drops_incomplete_frame(self) -> None:
        assembler = UDPFrameAssembler()
        assembler.add_datagram(self._header(20))
        assembler.add_datagram(_packet(b"partial"))

        assembler.add_datagram(self._header(10))

        self.assertEqual(1, assembler.dropped_frames)

    def test_ignores_malformed_datagram(self) -> None:
        assembler = UDPFrameAssembler()

        self.assertIsNone(assembler.add_datagram(b"bad"))

        self.assertEqual(1, assembler.malformed_packets)

    def test_payload_starting_with_magic_is_not_mistaken_for_header(self) -> None:
        assembler = UDPFrameAssembler()
        image = b"\xbc" + b"camera-data-longer-than-header"

        assembler.add_datagram(self._header(len(image)), 1.0)
        frame = assembler.add_datagram(_packet(image), 1.1)

        assert frame is not None
        self.assertEqual(image, frame.data)

    def test_ignores_invalid_header_without_stopping_stream(self) -> None:
        assembler = UDPFrameAssembler()
        invalid_header = struct.pack(
            "<BHHBBI",
            0xBC,
            324,
            21300,
            1,
            int(CameraEncoding.JPEG),
            100,
        )

        self.assertIsNone(assembler.add_datagram(_packet(invalid_header)))

        self.assertEqual(1, assembler.malformed_packets)

    def test_udp_client_sends_handshake_and_receives_frame(self) -> None:
        image = b"\xff\xd8camera-data\xff\xd9"
        connection = Mock()
        connection.recvfrom.side_effect = [
            (self._header(len(image)), ("192.168.4.1", 5000)),
            (_packet(image), ("192.168.4.1", 5000)),
        ]
        with patch(
            "bumblebee.camera.protocol.socket.socket",
            return_value=connection,
        ):
            client = AIdeckUDPCameraClient(handshake_interval=60)
            client.connect()
            frame = client.read_frame()

        connection.sendto.assert_called_once_with(b"FER", ("192.168.4.1", 5000))
        self.assertEqual(image, frame.data)

    def test_udp_client_ignores_datagrams_from_wrong_port(self) -> None:
        image = b"\xff\xd8camera-data\xff\xd9"
        connection = Mock()
        connection.recvfrom.side_effect = [
            (self._header(len(image)), ("192.168.4.1", 4999)),
            (_packet(image), ("192.168.4.1", 4999)),
            (self._header(len(image)), ("192.168.4.1", 5000)),
            (_packet(image), ("192.168.4.1", 5000)),
        ]
        with patch(
            "bumblebee.camera.protocol.socket.socket",
            return_value=connection,
        ):
            client = AIdeckUDPCameraClient(handshake_interval=60)
            client.connect()
            frame = client.read_frame()

        self.assertEqual(4, connection.recvfrom.call_count)
        self.assertEqual(image, frame.data)


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
