import unittest

import numpy as np

from rtsp_camera import RtspCameraStream, validate_rtsp_url


class FakeCapture:
    def __init__(self, frame):
        self.frame = frame
        self.released = False
        self.calls = 0

    def isOpened(self):
        return True

    def set(self, *_):
        pass

    def read(self):
        self.calls += 1
        if self.calls == 1:
            return True, self.frame.copy()
        return False, None

    def release(self):
        self.released = True


class RtspCameraTests(unittest.TestCase):
    def test_validates_rtsp_url(self):
        self.assertEqual(validate_rtsp_url("rtsp://192.168.144.25:8554/main.264"),
                         "rtsp://192.168.144.25:8554/main.264")
        for value in ("http://camera/live", "rtsp://", "rtsp://camera"):
            with self.assertRaises(ValueError):
                validate_rtsp_url(value)

    def test_retains_latest_frame_and_releases_capture(self):
        captures = []
        frame = np.full((4, 6, 3), 7, np.uint8)

        def factory(*_):
            capture = FakeCapture(frame)
            captures.append(capture)
            return capture

        stream = RtspCameraStream("rtsp://camera:8554/main.264", .01, factory)
        try:
            self.assertTrue(stream.wait_for_frame(.5))
            received = stream.latest()
            self.assertTrue(np.array_equal(received, frame))
            received[:] = 0
            self.assertTrue(np.array_equal(stream.latest(), frame))
        finally:
            stream.close()
        self.assertTrue(any(capture.released for capture in captures))


if __name__ == "__main__":
    unittest.main()
