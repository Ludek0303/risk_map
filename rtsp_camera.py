"""Low-latency RTSP frame source used by live mapping capture."""

from __future__ import annotations

import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np


def validate_rtsp_url(url: str) -> str:
    parsed = urlparse(url)
    if parsed.scheme != "rtsp" or not parsed.hostname or not parsed.path:
        raise ValueError("RTSP URL must have the form rtsp://host:port/path")
    return url


class RtspCameraStream:
    """Continuously drain an RTSP stream and retain only its freshest BGR frame."""

    def __init__(self, url: str, reconnect_seconds: float = 1.0, capture_factory=cv2.VideoCapture):
        self.url = validate_rtsp_url(url)
        self._reconnect_seconds = reconnect_seconds
        self._capture_factory = capture_factory
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._capture = None
        self._thread = threading.Thread(target=self._run, name="rtsp-camera", daemon=True)
        self._thread.start()

    def _open(self):
        capture = self._capture_factory(self.url, cv2.CAP_FFMPEG)
        if not capture.isOpened():
            capture.release()
            return None
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _run(self):
        while not self._stop.is_set():
            capture = self._open()
            if capture is None:
                self._stop.wait(self._reconnect_seconds)
                continue
            self._capture = capture
            while not self._stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None or frame.size == 0:
                    break
                with self._lock:
                    self._latest = frame
                self._ready.set()
            capture.release()
            self._capture = None
            if not self._stop.is_set():
                self._stop.wait(self._reconnect_seconds)

    def wait_for_frame(self, timeout: float = 10.0) -> bool:
        return self._ready.wait(timeout)

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def close(self) -> None:
        self._stop.set()
        capture = self._capture
        if capture is not None:
            capture.release()
        self._thread.join(timeout=max(2.0, self._reconnect_seconds + 1.0))
