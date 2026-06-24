import csv
import tempfile
import unittest
from pathlib import Path

from modules.perception.tracking_dryrun_logger import TrackingDryRunLogger
from modules.perception.tracking_log_analyzer import analyze_tracking_log, format_analysis_report


class TrackingLogAnalyzerTests(unittest.TestCase):
    def _write_rows(self, path, rows):
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=TrackingDryRunLogger.FIELDNAMES)
            writer.writeheader()
            writer.writerows(rows)

    def test_analyze_tracking_log_reports_detection_status_ranges_and_control_limits(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "dryrun.csv"
            self._write_rows(
                log_path,
                [
                    {
                        "timestamp": "1.0",
                        "marker_id": "20",
                        "detected": "1",
                        "tracking_status": "tracking",
                        "lost_frames": "0",
                        "pre_dock_ready": "0",
                        "pre_dock_block_reason": "yaw_error_high",
                        "pose_z": "1.20",
                        "pose_yaw": "10.0",
                        "filtered_z": "1.15",
                        "filtered_yaw": "8.0",
                        "body_forward_m": "1.15",
                        "yaw_error_deg": "8.0",
                        "cmd_vx": "0.16",
                        "cmd_vy": "-0.02",
                        "cmd_vz": "0.01",
                        "cmd_yaw_rate": "-0.10",
                        "rc_ch3": "1500",
                        "rc_ch4": "1490",
                        "rc_ch5": "1540",
                        "rc_ch6": "1505",
                        "pose_valid": "1",
                        "reject_reason": "",
                        "marker_pixel_size_px": "85.0",
                        "reprojection_error_px": "1.2",
                        "tracker_frames_processed": "10",
                        "tracker_marker_frames": "8",
                        "tracker_target_frames": "7",
                        "tracker_valid_pose_frames": "6",
                        "tracker_invalid_pose_frames": "1",
                        "tracker_no_marker_frames": "2",
                    },
                    {
                        "timestamp": "1.1",
                        "marker_id": "20",
                        "detected": "1",
                        "tracking_status": "tracking",
                        "lost_frames": "0",
                        "pre_dock_ready": "1",
                        "pre_dock_block_reason": "",
                        "pose_z": "0.82",
                        "pose_yaw": "2.0",
                        "filtered_z": "0.82",
                        "filtered_yaw": "2.0",
                        "cmd_vx": "0.01",
                        "cmd_vy": "0.00",
                        "cmd_vz": "0.00",
                        "cmd_yaw_rate": "-0.03",
                        "rc_ch3": "1501",
                        "rc_ch4": "1500",
                        "rc_ch5": "1502",
                        "rc_ch6": "1500",
                        "pose_valid": "0",
                        "reject_reason": "reprojection_error_too_high",
                        "marker_pixel_size_px": "84.0",
                        "reprojection_error_px": "8.5",
                        "tracker_frames_processed": "20",
                        "tracker_marker_frames": "16",
                        "tracker_target_frames": "14",
                        "tracker_valid_pose_frames": "12",
                        "tracker_invalid_pose_frames": "2",
                        "tracker_no_marker_frames": "4",
                    },
                    {
                        "timestamp": "1.2",
                        "detected": "0",
                        "tracking_status": "predicted",
                        "lost_frames": "1",
                        "pre_dock_ready": "0",
                        "pre_dock_block_reason": "recent_observation_expired",
                        "filtered_z": "0.84",
                        "filtered_yaw": "2.5",
                        "cmd_vx": "0.02",
                        "cmd_vy": "0.01",
                        "cmd_vz": "0.00",
                        "cmd_yaw_rate": "-0.04",
                        "pose_valid": "0",
                        "reject_reason": "no_pose",
                        "yaw_error_deg": "0.0",
                        "tracker_frames_processed": "30",
                        "tracker_marker_frames": "24",
                        "tracker_target_frames": "21",
                        "tracker_valid_pose_frames": "18",
                        "tracker_invalid_pose_frames": "3",
                        "tracker_no_marker_frames": "6",
                    },
                    {
                        "timestamp": "1.3",
                        "detected": "0",
                        "tracking_status": "lost",
                        "lost_frames": "10",
                        "pre_dock_ready": "0",
                        "pre_dock_block_reason": "lost",
                        "cmd_vx": "0.00",
                        "cmd_vy": "0.00",
                        "cmd_vz": "0.00",
                        "cmd_yaw_rate": "0.00",
                        "pose_valid": "0",
                        "reject_reason": "no_pose",
                        "yaw_error_deg": "0.0",
                        "tracker_frames_processed": "40",
                        "tracker_marker_frames": "32",
                        "tracker_target_frames": "28",
                        "tracker_valid_pose_frames": "24",
                        "tracker_invalid_pose_frames": "4",
                        "tracker_no_marker_frames": "8",
                    },
                    {
                        "timestamp": "1.4",
                        "detected": "0",
                        "tracking_status": "predicted",
                        "lost_frames": "1",
                        "pre_dock_ready": "0",
                        "pre_dock_block_reason": "valid_frame_count_low",
                        "pose_valid": "1",
                        "reject_reason": "",
                        "yaw_error_deg": "0.0",
                        "tracker_frames_processed": "50",
                        "tracker_marker_frames": "40",
                        "tracker_target_frames": "35",
                        "tracker_valid_pose_frames": "30",
                        "tracker_invalid_pose_frames": "5",
                        "tracker_no_marker_frames": "10",
                    },
                ],
            )

            summary = analyze_tracking_log(log_path)

        self.assertEqual(summary["sample_count"], 5)
        self.assertEqual(summary["detected_count"], 2)
        self.assertAlmostEqual(summary["detected_rate"], 0.4)
        self.assertEqual(summary["status_counts"]["tracking"], 2)
        self.assertEqual(summary["status_counts"]["predicted"], 2)
        self.assertEqual(summary["status_counts"]["lost"], 1)
        self.assertEqual(summary["pre_dock_ready_count"], 1)
        self.assertEqual(summary["pre_dock_block_reason_counts"]["recent_observation_expired"], 1)
        self.assertEqual(summary["pre_dock_block_reason_counts"]["valid_frame_count_low"], 1)
        self.assertEqual(summary["pre_dock_block_reason_counts"]["yaw_error_high"], 1)
        self.assertEqual(summary["max_lost_frames"], 10)
        self.assertEqual(summary["valid_pose_count"], 1)
        self.assertAlmostEqual(summary["valid_pose_rate"], 0.2)
        self.assertEqual(summary["reject_reason_counts"]["no_pose"], 2)
        self.assertEqual(summary["reject_reason_counts"]["reprojection_error_too_high"], 1)
        self.assertEqual(summary["ranges"]["filtered_z"], {"min": 1.15, "max": 1.15})
        self.assertEqual(summary["ranges"]["body_forward_m"], {"min": 1.15, "max": 1.15})
        self.assertEqual(summary["ranges"]["yaw_error_deg"], {"min": 8.0, "max": 8.0})
        self.assertEqual(summary["ranges"]["cmd_vx"], {"min": 0.0, "max": 0.16})
        self.assertEqual(summary["ranges"]["rc_ch5"], {"min": 1502.0, "max": 1540.0})
        self.assertEqual(summary["ranges"]["rc_ch6"], {"min": 1500.0, "max": 1505.0})
        self.assertEqual(summary["ranges"]["marker_pixel_size_px"], {"min": 85.0, "max": 85.0})
        self.assertEqual(summary["ranges"]["reprojection_error_px"], {"min": 1.2, "max": 1.2})
        self.assertEqual(summary["tracker_frames_processed"], 50)
        self.assertEqual(summary["tracker_valid_pose_frames"], 30)
        self.assertAlmostEqual(summary["tracker_valid_pose_rate"], 0.6)
        self.assertAlmostEqual(summary["tracker_marker_rate"], 0.8)
        self.assertAlmostEqual(summary["tracker_fps"], 125.0)

    def test_format_analysis_report_contains_key_metrics(self):
        report = format_analysis_report(
            {
                "sample_count": 4,
                "detected_count": 2,
                "detected_rate": 0.5,
                "valid_pose_count": 1,
                "valid_pose_rate": 0.25,
                "pre_dock_ready_count": 1,
                "max_lost_frames": 10,
                "status_counts": {"tracking": 2, "predicted": 1, "lost": 1},
                "reject_reason_counts": {"no_pose": 2, "reprojection_error_too_high": 1},
                "pre_dock_block_reason_counts": {"recent_observation_expired": 2, "yaw_error_high": 1},
                "ranges": {"filtered_z": {"min": 0.82, "max": 1.15}, "rc_ch5": {"min": 1500, "max": 1530}},
                "tracker_frames_processed": 50,
                "tracker_marker_frames": 40,
                "tracker_marker_rate": 0.8,
                "tracker_target_frames": 35,
                "tracker_target_rate": 0.7,
                "tracker_valid_pose_frames": 30,
                "tracker_valid_pose_rate": 0.6,
                "tracker_fps": 12.5,
            }
        )

        self.assertIn("samples: 4", report)
        self.assertIn("detected: 2 (50.0%)", report)
        self.assertIn("valid_pose: 1 (25.0%)", report)
        self.assertIn("reject_reasons:", report)
        self.assertIn("no_pose: 2", report)
        self.assertIn("pre_dock_ready: 1", report)
        self.assertIn("pre_dock_block_reasons:", report)
        self.assertIn("recent_observation_expired: 2", report)
        self.assertIn("tracker_frames: 50", report)
        self.assertIn("tracker_fps: 12.50", report)
        self.assertIn("tracker_valid_pose_frames: 30 (60.0%)", report)
        self.assertIn("filtered_z: 0.820 .. 1.150", report)
        self.assertIn("rc_ch5: 1500.000 .. 1530.000", report)


if __name__ == "__main__":
    unittest.main()
