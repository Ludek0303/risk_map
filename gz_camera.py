"""Camera frame grabber for a PX4/Gazebo SITL simulation.

Subscribes directly to the Gazebo Transport image topic published by the
sensor (no ROS 2 bridge required) and exposes frames as OpenCV BGR numpy
arrays.
"""

from __future__ import annotations

import os

# OpenCV's bundled Qt GUI backend only ships an xcb platform plugin. On a
# Wayland session (WAYLAND_DISPLAY set) Qt otherwise tries to auto-detect a
# wayland plugin from elsewhere on the system, which is a common cause of
# cv2.imshow rendering a black/blank window. Force xcb (via XWayland) unless
# the user already set something explicitly.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import queue
import threading
import time

import cv2

# cv2's own import (cv2/config-3.py) unconditionally overwrites
# QT_QPA_FONTDIR to point at its bundled (and in this install, missing)
# fonts dir, which is what triggers the "QFontDatabase: Cannot find font
# directory" warning - so this has to be set/fixed *after* importing cv2,
# not before.
os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"

import numpy as np
from gz.msgs10.image_pb2 import (
    BGR_INT8,
    BGRA_INT8,
    L_INT8,
    RGB_INT8,
    RGBA_INT8,
    Image,
)
from gz.transport13 import Node

DEFAULT_WORLD = "baylands"
DEFAULT_MODEL = "x500_gimbal_0"


def build_camera_topic(world_name: str, model_name: str) -> str:
    return (
        f"/world/{world_name}/model/{model_name}"
        "/link/camera_link/sensor/camera/image"
    )


# List available topics with: gz topic -l | grep camera
DEFAULT_TOPIC = build_camera_topic(DEFAULT_WORLD, DEFAULT_MODEL)

_PIXEL_FORMAT_CHANNELS = {
    L_INT8: 1,
    RGB_INT8: 3,
    RGBA_INT8: 4,
    BGRA_INT8: 4,
    BGR_INT8: 3,
}


def _to_bgr(msg: Image) -> np.ndarray:
    channels = _PIXEL_FORMAT_CHANNELS.get(msg.pixel_format_type)
    if channels is None:
        raise ValueError(
            f"Unsupported pixel_format_type={msg.pixel_format_type}"
        )

    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, channels
    )

    if msg.pixel_format_type == RGB_INT8:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if msg.pixel_format_type == RGBA_INT8:
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if msg.pixel_format_type == L_INT8:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()  # already BGR / BGRA_INT8


class GzCameraStream:
    """Subscribes to a Gazebo image topic and yields the latest BGR frame.

    Only the most recent frame is kept (queue of size 1) so consumers always
    see the freshest image instead of lagging behind a backlog.
    """

    def __init__(self, topic: str = DEFAULT_TOPIC) -> None:
        self.topic = topic
        self._node = Node()
        self._queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()

        ok = self._node.subscribe(Image, topic, self._on_image)
        if not ok:
            raise RuntimeError(f"Failed to subscribe to topic '{topic}'")

    def _on_image(self, msg: Image) -> None:
        frame = _to_bgr(msg)
        with self._lock:
            self._latest = frame
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(frame)

    def read(self, timeout: float | None = None) -> np.ndarray | None:
        """Block until a new frame arrives (or timeout), return it."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def latest(self) -> np.ndarray | None:
        """Return the most recent frame without blocking (may be None)."""
        with self._lock:
            return self._latest

    def frames(self):
        """Generator yielding frames as they arrive, forever."""
        while True:
            frame = self.read(timeout=None)
            if frame is not None:
                yield frame


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", default=None,
        help="Full Gazebo image topic. Overrides --world/--model if given.",
    )
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--point-down", action="store_true",
        help="Command the gimbal straight down (nadir) via MAVLink before "
             "showing the preview.",
    )
    args = parser.parse_args()
    topic = args.topic or build_camera_topic(args.world, args.model)

    if args.point_down:
        from gimbal_control import GimbalController

        print("Pointing gimbal down ...")
        with GimbalController() as gimbal:
            gimbal.point_down()

    print(f"Subscribing to: {topic}")
    stream = GzCameraStream(topic=topic)

    window = "PX4/Gazebo camera"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    last_frame_time = time.time()
    try:
        while True:
            frame = stream.read(timeout=1.0)
            if frame is None:
                if time.time() - last_frame_time > 5:
                    print("No frames received yet - is the sim running "
                          "and the topic name correct? (gz topic -l)")
                    last_frame_time = time.time()
                continue
            last_frame_time = time.time()

            cv2.imshow(window, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
