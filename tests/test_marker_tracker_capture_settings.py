import unittest
from types import SimpleNamespace

from modules.perception.marker_tracker import apply_capture_settings, frame_size_matches_config, open_first_readable_capture


class FakeCapture:
    def __init__(self):
        self.values = {}

    def set(self, prop, value):
        self.values[prop] = value
        return True

    def get(self, prop):
        return self.values.get(prop, 0)

    def isOpened(self):
        return True

    def read(self):
        return True, object()

    def release(self):
        self.released = True


class MarkerTrackerCaptureSettingsTests(unittest.TestCase):
    def test_apply_capture_settings_sets_requested_resolution_and_reports_actual(self):
        cv2_stub = SimpleNamespace(CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4)
        cap = FakeCapture()

        actual = apply_capture_settings(
            cap,
            {"frame_width": 1920, "frame_height": 1080},
            cv2_module=cv2_stub,
        )

        self.assertEqual(cap.values[3], 1920)
        self.assertEqual(cap.values[4], 1080)
        self.assertEqual(actual, {"frame_width": 1920, "frame_height": 1080})

    def test_apply_capture_settings_sets_fourcc_fps_and_reports_actual_values(self):
        cv2_stub = SimpleNamespace(
            CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4,
            CAP_PROP_FPS=5,
            CAP_PROP_FOURCC=6,
            VideoWriter_fourcc=lambda *chars: sum(ord(char) << (8 * idx) for idx, char in enumerate(chars)),
        )
        cap = FakeCapture()

        actual = apply_capture_settings(
            cap,
            {"frame_width": 1920, "frame_height": 1080, "fourcc": "MJPG", "fps": 30},
            cv2_module=cv2_stub,
        )

        self.assertEqual(cap.values[6], cv2_stub.VideoWriter_fourcc(*"MJPG"))
        self.assertEqual(cap.values[5], 30)
        self.assertEqual(actual["frame_fourcc"], "MJPG")
        self.assertEqual(actual["frame_fps"], 30.0)

    def test_frame_size_matches_config_detects_resolution_mismatch(self):
        self.assertTrue(frame_size_matches_config({}, {"frame_width": 1280, "frame_height": 720}))
        self.assertTrue(
            frame_size_matches_config(
                {"frame_width": 1920, "frame_height": 1080},
                {"frame_width": 1920, "frame_height": 1080},
            )
        )
        self.assertFalse(
            frame_size_matches_config(
                {"frame_width": 1920, "frame_height": 1080},
                {"frame_width": 1280, "frame_height": 720},
            )
        )

    def test_open_first_readable_capture_skips_open_device_that_cannot_read_frame(self):
        cv2_stub = SimpleNamespace(CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4)
        first = FakeCapture()
        first.read = lambda: (False, None)
        second = FakeCapture()
        created = []

        def capture_factory(device):
            created.append(device)
            return first if device == "/dev/video1" else second

        cap, device, actual = open_first_readable_capture(
            ["/dev/video1", "/dev/video0"],
            {"frame_width": 1920, "frame_height": 1080},
            cv2_module=cv2_stub,
            capture_factory=capture_factory,
        )

        self.assertIs(cap, second)
        self.assertEqual(device, "/dev/video0")
        self.assertEqual(actual, {"frame_width": 1920, "frame_height": 1080})
        self.assertEqual(created, ["/dev/video1", "/dev/video0"])
        self.assertTrue(first.released)


if __name__ == "__main__":
    unittest.main()
