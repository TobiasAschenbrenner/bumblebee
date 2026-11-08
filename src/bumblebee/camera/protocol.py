"""Read frames from the original AI-deck Wi-Fi image streamer."""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol

DEFAULT_CAMERA_HOST = "192.168.4.1"
DEFAULT_CAMERA_PORT = 5000

_CPX_HEADER = struct.Struct("<HBB")
_IMAGE_HEADER = struct.Struct("<BHHBBI")
_IMAGE_MAGIC = 0xBC
_MAX_CPX_PAYLOAD = 4096
_MAX_IMAGE_BYTES = 8 * 1024 * 1024
_MAX_IMAGE_DIMENSION = 4096


class CameraStreamError(RuntimeError):
    """Raised when the AI-deck stream cannot provide a valid frame."""


class _StreamStopped(CameraStreamError):
    """Internal signal used while intentionally closing the stream."""


class CameraEncoding(IntEnum):
    """Image encodings produced by the original AI-deck streamer."""

    RAW_BAYER = 0
    JPEG = 1


@dataclass(frozen=True)
class CameraFrame:
    """One complete image received from the AI-deck."""

    width: int
    height: int
    depth: int
    encoding: CameraEncoding
    data: bytes
    received_at: float
    transfer_seconds: float

    @property
    def encoding_name(self) -> str:
        return "Raw Bayer" if self.encoding is CameraEncoding.RAW_BAYER else "JPEG"


class CameraClient(Protocol):
    """Transport-independent interface consumed by the camera worker."""

    host: str
    port: int
    transport_name: str
    dropped_frames: int

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def read_frame(
        self,
        stop_event: threading.Event | None = None,
    ) -> CameraFrame: ...


class AIdeckCameraClient:
    """TCP client for Bitcraze's original AI-deck image streamer."""

    transport_name = "TCP"

    def __init__(
        self,
        host: str = DEFAULT_CAMERA_HOST,
        port: int = DEFAULT_CAMERA_PORT,
        *,
        connect_timeout: float = 5.0,
        read_timeout: float = 0.5,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.read_timeout = read_timeout
        self.dropped_frames = 0
        self._socket: socket.socket | None = None

    def connect(self) -> None:
        if self._socket is not None:
            return

        try:
            connection = socket.create_connection(
                (self.host, self.port),
                timeout=self.connect_timeout,
            )
            connection.settimeout(self.read_timeout)
        except OSError as exc:
            raise CameraStreamError(
                f"Could not connect to the AI-deck at {self.host}:{self.port}. "
                "Connect the Mac to the AI-deck Wi-Fi network and try again. "
                f"Details: {exc}"
            ) from exc

        self._socket = connection

    def close(self) -> None:
        connection, self._socket = self._socket, None
        if connection is None:
            return
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()

    def read_frame(
        self,
        stop_event: threading.Event | None = None,
    ) -> CameraFrame:
        started_at = time.monotonic()
        header = self._read_image_header(stop_event)
        magic, width, height, depth, encoding_value, image_size = header
        _validate_image_header(
            magic=magic,
            width=width,
            height=height,
            depth=depth,
            encoding_value=encoding_value,
            image_size=image_size,
        )

        image = bytearray()
        while len(image) < image_size:
            payload = self._read_packet(stop_event)
            remaining = image_size - len(image)
            if len(payload) > remaining:
                raise CameraStreamError(
                    "The AI-deck sent more image data than its frame header declared."
                )
            image.extend(payload)

        received_at = time.monotonic()
        return CameraFrame(
            width=width,
            height=height,
            depth=depth,
            encoding=CameraEncoding(encoding_value),
            data=bytes(image),
            received_at=received_at,
            transfer_seconds=received_at - started_at,
        )

    def _read_image_header(
        self,
        stop_event: threading.Event | None,
    ) -> tuple[int, int, int, int, int, int]:
        while True:
            payload = self._read_packet(stop_event)
            if len(payload) == _IMAGE_HEADER.size and payload[0] == _IMAGE_MAGIC:
                return _IMAGE_HEADER.unpack(payload)

    def _read_packet(self, stop_event: threading.Event | None) -> bytes:
        packet_header = self._read_exact(_CPX_HEADER.size, stop_event)
        packet_length, _routing, _function = _CPX_HEADER.unpack(packet_header)
        payload_length = packet_length - 2
        if not 0 <= payload_length <= _MAX_CPX_PAYLOAD:
            raise CameraStreamError(
                f"The AI-deck sent an invalid packet length ({packet_length})."
            )
        return self._read_exact(payload_length, stop_event)

    def _read_exact(
        self,
        size: int,
        stop_event: threading.Event | None,
    ) -> bytes:
        if self._socket is None:
            raise CameraStreamError("The AI-deck camera is not connected.")

        data = bytearray()
        while len(data) < size:
            if stop_event is not None and stop_event.is_set():
                raise _StreamStopped("The camera stream was stopped.")
            try:
                chunk = self._socket.recv(size - len(data))
            except TimeoutError:
                continue
            except OSError as exc:
                if stop_event is not None and stop_event.is_set():
                    raise _StreamStopped("The camera stream was stopped.") from exc
                raise CameraStreamError(
                    f"The AI-deck camera connection failed: {exc}"
                ) from exc

            if not chunk:
                raise CameraStreamError("The AI-deck closed the camera connection.")
            data.extend(chunk)

        return bytes(data)


@dataclass
class _PendingUDPFrame:
    width: int
    height: int
    depth: int
    encoding: CameraEncoding
    image_size: int
    data: bytearray
    started_at: float


class UDPFrameAssembler:
    """Reassemble CPX packets while dropping incomplete UDP frames."""

    def __init__(self) -> None:
        self.dropped_frames = 0
        self.malformed_packets = 0
        self._pending: _PendingUDPFrame | None = None

    def add_datagram(
        self,
        datagram: bytes,
        received_at: float | None = None,
    ) -> CameraFrame | None:
        received_at = time.monotonic() if received_at is None else received_at
        payload = self._payload(datagram)
        if payload is None:
            return None

        new_frame = self._new_frame_if_header(payload, received_at)
        if new_frame is not None:
            if self._pending is not None:
                self.dropped_frames += 1
            self._pending = new_frame
        elif self._pending is not None:
            self._pending.data.extend(payload)

        pending = self._pending
        if pending is None or len(pending.data) < pending.image_size:
            return None
        if len(pending.data) > pending.image_size:
            self.malformed_packets += 1
            self.dropped_frames += 1
            self._pending = None
            return None

        self._pending = None
        return CameraFrame(
            width=pending.width,
            height=pending.height,
            depth=pending.depth,
            encoding=pending.encoding,
            data=bytes(pending.data),
            received_at=received_at,
            transfer_seconds=received_at - pending.started_at,
        )

    def _payload(self, datagram: bytes) -> bytes | None:
        if len(datagram) < _CPX_HEADER.size:
            self.malformed_packets += 1
            return None
        packet_length, _routing, _function = _CPX_HEADER.unpack_from(datagram)
        expected_length = packet_length + 2
        if packet_length < 2 or expected_length != len(datagram):
            self.malformed_packets += 1
            return None
        return datagram[_CPX_HEADER.size :]

    def _new_frame_if_header(
        self,
        payload: bytes,
        received_at: float,
    ) -> _PendingUDPFrame | None:
        if len(payload) != _IMAGE_HEADER.size or payload[0] != _IMAGE_MAGIC:
            return None
        try:
            return self._new_frame(payload, received_at)
        except CameraStreamError:
            if self._pending is None:
                self.malformed_packets += 1
            return None

    def _new_frame(self, payload: bytes, received_at: float) -> _PendingUDPFrame:
        magic, width, height, depth, encoding_value, image_size = (
            _IMAGE_HEADER.unpack_from(payload)
        )
        _validate_image_header(
            magic=magic,
            width=width,
            height=height,
            depth=depth,
            encoding_value=encoding_value,
            image_size=image_size,
        )
        return _PendingUDPFrame(
            width=width,
            height=height,
            depth=depth,
            encoding=CameraEncoding(encoding_value),
            image_size=image_size,
            data=bytearray(payload[_IMAGE_HEADER.size :]),
            started_at=received_at,
        )


class AIdeckUDPCameraClient:
    """UDP client for the LARICS community AI-deck streamer."""

    transport_name = "UDP"
    _HANDSHAKE = b"FER"

    def __init__(
        self,
        host: str = DEFAULT_CAMERA_HOST,
        port: int = DEFAULT_CAMERA_PORT,
        *,
        listen_port: int = 5001,
        read_timeout: float = 0.5,
        handshake_interval: float = 2.0,
    ) -> None:
        self.host = host
        self.port = port
        self.listen_port = listen_port
        self.read_timeout = read_timeout
        self.handshake_interval = handshake_interval
        self._socket: socket.socket | None = None
        self._assembler = UDPFrameAssembler()
        self._last_handshake_at = 0.0

    @property
    def dropped_frames(self) -> int:
        return self._assembler.dropped_frames

    def connect(self) -> None:
        if self._socket is not None:
            return
        connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            connection.bind(("", self.listen_port))
            connection.settimeout(self.read_timeout)
        except OSError as exc:
            connection.close()
            raise CameraStreamError(
                f"Could not open UDP camera port {self.listen_port}: {exc}"
            ) from exc
        self._socket = connection
        self._send_handshake()

    def close(self) -> None:
        connection, self._socket = self._socket, None
        if connection is not None:
            connection.close()

    def read_frame(
        self,
        stop_event: threading.Event | None = None,
    ) -> CameraFrame:
        if self._socket is None:
            raise CameraStreamError("The UDP camera receiver is not open.")

        while True:
            if stop_event is not None and stop_event.is_set():
                raise _StreamStopped("The camera stream was stopped.")
            if time.monotonic() - self._last_handshake_at >= self.handshake_interval:
                self._send_handshake()
            try:
                datagram, source = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError as exc:
                if stop_event is not None and stop_event.is_set():
                    raise _StreamStopped("The camera stream was stopped.") from exc
                raise CameraStreamError(
                    f"The UDP camera receiver failed: {exc}"
                ) from exc

            if source[0] != self.host:
                continue
            frame = self._assembler.add_datagram(datagram)
            if frame is not None:
                return frame

    def _send_handshake(self) -> None:
        if self._socket is None:
            return
        try:
            self._socket.sendto(self._HANDSHAKE, (self.host, self.port))
        except OSError as exc:
            raise CameraStreamError(
                f"Could not contact the UDP streamer at {self.host}:{self.port}: {exc}"
            ) from exc
        self._last_handshake_at = time.monotonic()


def _validate_image_header(
    *,
    magic: int,
    width: int,
    height: int,
    depth: int,
    encoding_value: int,
    image_size: int,
) -> None:
    if magic != _IMAGE_MAGIC:
        raise CameraStreamError("The AI-deck sent an invalid image marker.")
    if not 0 < width <= _MAX_IMAGE_DIMENSION:
        raise CameraStreamError(f"Invalid image width: {width}.")
    if not 0 < height <= _MAX_IMAGE_DIMENSION:
        raise CameraStreamError(f"Invalid image height: {height}.")
    if depth not in (1, 3, 4):
        raise CameraStreamError(f"Unsupported image depth: {depth}.")
    if encoding_value not in set(CameraEncoding):
        raise CameraStreamError(
            f"Unsupported AI-deck image encoding: {encoding_value}."
        )
    if not 0 < image_size <= _MAX_IMAGE_BYTES:
        raise CameraStreamError(f"Invalid image size: {image_size} bytes.")
    if (
        encoding_value == CameraEncoding.RAW_BAYER
        and image_size != width * height * depth
    ):
        raise CameraStreamError("The raw image size does not match its dimensions.")
