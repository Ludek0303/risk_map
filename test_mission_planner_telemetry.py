import tempfile
import unittest
from pathlib import Path

import cv2
import piexif
import numpy as np

from mapping_capture import _write_geotagged_jpeg, lr900_usb_connection, mission_planner_udp_connection


class MissionPlannerTelemetryTests(unittest.TestCase):
    def test_uses_a_local_udp_receiver(self):
        self.assertEqual(mission_planner_udp_connection("127.0.0.1", 14551),
                         "udpin:127.0.0.1:14551")
        self.assertEqual(mission_planner_udp_connection("localhost", 14550),
                         "udpin:127.0.0.1:14550")

    def test_rejects_network_facing_or_invalid_endpoints(self):
        for host, port in (("0.0.0.0", 14551), ("192.168.1.9", 14551),
                           ("127.0.0.1", 0), ("127.0.0.1", 65536)):
            with self.assertRaises(ValueError):
                mission_planner_udp_connection(host, port)

    def test_lr900_serial_port_and_baud(self):
        self.assertEqual(lr900_usb_connection(" /dev/ttyUSB0 ", 57600),
                         ("/dev/ttyUSB0", 57600))
        self.assertEqual(lr900_usb_connection("COM3", 115200), ("COM3", 115200))
        for port, baud in (("", 57600), ("COM3", 0), ("COM3", -1)):
            with self.assertRaises(ValueError):
                lr900_usb_connection(port, baud)

    def test_real_camera_metadata_is_written_to_geotagged_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "photo.jpg"
            _write_geotagged_jpeg(np.zeros((12, 16, 3), np.uint8), path,
                                  52.1, 21.0, 45.0, 180.0,
                                  "SIYI", "A8", 24)
            exif = piexif.load(str(path))
            self.assertEqual(exif["0th"][piexif.ImageIFD.Make], b"SIYI")
            self.assertEqual(exif["0th"][piexif.ImageIFD.Model], b"A8")
            self.assertEqual(exif["Exif"][piexif.ExifIFD.FocalLengthIn35mmFilm], 24)


if __name__ == "__main__":
    unittest.main()
