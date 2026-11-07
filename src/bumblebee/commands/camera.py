"""View and optionally record the original AI-deck Wi-Fi camera stream."""

from __future__ import annotations

import argparse
import time
from collections import deque
from collections.abc import Sequence
from pathlib import Path

import cv2
from rich.console import Console
from rich.live import Live

from bumblebee.camera.images import Image, decode_frame
from bumblebee.camera.protocol import (
    DEFAULT_CAMERA_HOST,
    DEFAULT_CAMERA_PORT,
    AIdeckCameraClient,
    CameraFrame,
    CameraStreamError,
)
from bumblebee.camera.recording import CameraRecorder
from bumblebee.camera.stream import CameraStream
from bumblebee.ui.camera import build_camera_dashboard

CONSOLE = Console()
WINDOW_NAME = "Bumblebee · AI-deck Camera"


def _positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return number


def _scale(value: str) -> float:
    number = float(value)
    if not 0.5 <= number <= 5.0:
        raise argparse.ArgumentTypeError("must be between 0.5 and 5")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=DEFAULT_CAMERA_HOST,
        help=f"AI-deck address (default: {DEFAULT_CAMERA_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_CAMERA_PORT,
        help=f"AI-deck TCP port (default: {DEFAULT_CAMERA_PORT})",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="start saving frames immediately; R toggles recording",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("recordings"),
        help="recording directory (default: ./recordings)",
    )
    parser.add_argument(
        "--scale",
        type=_scale,
        default=2.0,
        help="preview scale, 0.5-5 (default: 2)",
    )
    parser.add_argument(
        "--stream-timeout",
        type=_positive_float,
        default=10.0,
        help="seconds without a complete frame before stopping (default: 10)",
    )
    return parser


def _preview(image: Image, scale: float, fps: float, recording: bool) -> Image:
    interpolation = cv2.INTER_NEAREST if scale >= 1 else cv2.INTER_AREA
    preview = cv2.resize(
        image,
        None,
        fx=scale,
        fy=scale,
        interpolation=interpolation,
    )
    label = f"{fps:.1f} FPS"
    if recording:
        label += "  REC"
    cv2.putText(
        preview,
        label,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255) if recording else (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return preview


def _frame_rate(timestamps: deque[float]) -> float:
    if len(timestamps) < 2:
        return 0.0
    elapsed = timestamps[-1] - timestamps[0]
    return 0.0 if elapsed <= 0 else (len(timestamps) - 1) / elapsed


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    recorder = CameraRecorder(args.output.expanduser())
    recording = bool(args.record)
    latest_frame: CameraFrame | None = None
    latest_image: Image | None = None
    timestamps: deque[float] = deque(maxlen=60)
    started_at = time.monotonic()
    window_opened = False

    with CONSOLE.status(
        f"[cyan]Connecting to the AI-deck camera at {args.host}:{args.port}…[/cyan]"
    ):
        client = AIdeckCameraClient(args.host, args.port)
        client.connect()

    stream = CameraStream(client)
    stream.start()
    try:
        with Live(
            build_camera_dashboard(
                host=args.host,
                port=args.port,
                status="Connected · waiting for images",
                frame=None,
                frames_per_second=0.0,
                frame_age_seconds=None,
                frames_received=0,
                frames_skipped=0,
                recording=recording,
                saved_frames=0,
                recording_directory=None,
            ),
            console=CONSOLE,
            refresh_per_second=8,
        ) as live:
            while True:
                frame = stream.latest()
                if frame is not None:
                    latest_frame = frame
                    latest_image = decode_frame(frame)
                    timestamps.append(frame.received_at)
                    if recording:
                        recorder.save(latest_image, frame)

                now = time.monotonic()
                frames_per_second = _frame_rate(timestamps)
                frame_age = (
                    None if latest_frame is None else now - latest_frame.received_at
                )
                elapsed_without_frame = (
                    now - started_at if frame_age is None else frame_age
                )

                if stream.error is not None:
                    raise stream.error
                if elapsed_without_frame > args.stream_timeout:
                    raise CameraStreamError(
                        "Connected, but no complete camera frame arrived for "
                        f"{args.stream_timeout:g} seconds. Power-cycle the Crazyflie, "
                        "reconnect to its Wi-Fi, and check that the original image "
                        "streamer firmware is running."
                    )

                if latest_image is not None:
                    preview = _preview(
                        latest_image,
                        args.scale,
                        frames_per_second,
                        recording,
                    )
                    cv2.imshow(WINDOW_NAME, preview)
                    window_opened = True

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("r"), ord("R")):
                    recording = not recording
                    if recording:
                        recorder.start()
                if window_opened:
                    try:
                        if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                            break
                    except cv2.error:
                        break

                status = "Streaming"
                if latest_frame is None:
                    status = "Connected · waiting for images"
                elif frame_age is not None and frame_age > 2.0:
                    status = "Connected · stream appears stalled"

                live.update(
                    build_camera_dashboard(
                        host=args.host,
                        port=args.port,
                        status=status,
                        frame=latest_frame,
                        frames_per_second=frames_per_second,
                        frame_age_seconds=frame_age,
                        frames_received=stream.frames_received,
                        frames_skipped=stream.frames_skipped,
                        recording=recording,
                        saved_frames=recorder.saved_frames,
                        recording_directory=recorder.session_directory,
                    )
                )
    finally:
        stream.stop()
        cv2.destroyAllWindows()

    CONSOLE.print(
        "[bold green]Camera viewer closed.[/bold green] No motor commands were sent."
    )
    if recorder.session_directory is not None:
        CONSOLE.print(
            f"Saved {recorder.saved_frames} frame(s) in "
            f"[cyan]{recorder.session_directory}[/cyan]."
        )
    return 0


def cli() -> int:
    try:
        return main()
    except CameraStreamError as exc:
        CONSOLE.print(f"\n[bold red]Camera viewer stopped:[/bold red] {exc}")
        return 2
    except KeyboardInterrupt:
        CONSOLE.print(
            "\n[yellow]Camera viewer stopped.[/yellow] No motor commands were sent."
        )
        return 130


if __name__ == "__main__":
    raise SystemExit(cli())
