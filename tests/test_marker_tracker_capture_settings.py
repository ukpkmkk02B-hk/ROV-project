import unittest
from types import SimpleNamespace

from modules.perception.marker_tracker import apply_capture_settings, frame_size_matches_config


class FakeCapture:
    def __init__(self):
        self.values = {}

    def set(self, prop, value):
        self.values[prop] = value
        return True

    def get(self, prop):
        return self.values.get(prop, 0)


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


if __name__ == "__main__":
    unittest.main()
