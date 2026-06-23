import unittest

from tools.analyze_aruco_tracking_log import build_report


class AnalyzeTrackingLogScriptTests(unittest.TestCase):
    def test_build_report_returns_text_summary_for_log_path(self):
        report = build_report(
            {
                "sample_count": 2,
                "detected_count": 1,
                "detected_rate": 0.5,
                "pre_dock_ready_count": 0,
                "max_lost_frames": 3,
                "status_counts": {"tracking": 1, "lost": 1},
                "ranges": {},
            }
        )

        self.assertIn("samples: 2", report)
        self.assertIn("detected: 1 (50.0%)", report)


if __name__ == "__main__":
    unittest.main()
