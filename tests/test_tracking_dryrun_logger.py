import csv
import tempfile
import unittest
from pathlib import Path

from modules.perception.tracking_dryrun_logger import TrackingDryRunLogger


class TrackingDryRunLoggerTests(unittest.TestCase):
    def test_log_sample_writes_pose_filter_command_and_ready_flag_to_csv(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "tracking.csv"
            logger = TrackingDryRunLogger(log_path)

            logger.log_sample(
                pose={
                    "id": 20,
                    "x": 0.1,
                    "y": -0.2,
                    "z": 0.8,
                    "yaw": 3.0,
                    "center_u": 960.0,
                    "center_v": 540.0,
                    "marker_pixel_size_px": 82.5,
                    "reprojection_error_px": 1.3,
                    "pose_valid": True,
                    "reject_reason": "",
                },
                filtered_state={
                    "x": 0.11,
                    "y": -0.19,
                    "z": 0.81,
                    "yaw": 2.5,
                    "status": "tracking",
                    "lost_frames": 0,
                    "forward_m": 0.81,
                    "right_m": 0.11,
                    "up_m": 0.19,
                    "yaw_raw_deg": 182.5,
                    "yaw_error_deg": 2.5,
                    "pre_dock_valid_frame_count": 4,
                    "has_recent_valid_observation": True,
                    "pre_dock_recent_observation_max_age_s": 0.5,
                    "pre_dock_recent_ok": True,
                    "pre_dock_valid_frames_ok": True,
                    "pre_dock_position_ok": False,
                    "pre_dock_distance_ok": True,
                    "pre_dock_yaw_ok": True,
                    "pre_dock_block_reason": "position_error_high",
                },
                control_cmd={
                    "forward_m_s": 0.01,
                    "right_m_s": -0.02,
                    "up_m_s": 0.03,
                    "yaw_rate_rad_s": -0.04,
                    "raw_motion_forward_m_s": 0.02,
                    "raw_motion_right_m_s": -0.04,
                    "raw_motion_up_m_s": 0.06,
                    "raw_motion_yaw_rate_rad_s": -0.08,
                    "raw_forward_error_m": 0.021,
                    "raw_right_error_m": -0.022,
                    "raw_up_error_m": 0.023,
                    "raw_yaw_error_deg": 2.4,
                    "deadbanded_forward_error_m": 0.011,
                    "deadbanded_right_error_m": -0.012,
                    "deadbanded_up_error_m": 0.013,
                    "deadbanded_yaw_error_deg": 1.4,
                    "control_deadband_m": 0.01,
                    "yaw_deadband_deg": 1.0,
                    "command_smoothing_alpha": 0.6,
                    "vx": 0.01,
                    "vy": -0.02,
                    "vz": 0.03,
                    "yaw_rate": -0.04,
                    "pid_forward_error": 0.01,
                    "pid_forward_p": 0.011,
                    "pid_forward_i": 0.012,
                    "pid_forward_d": 0.013,
                    "pid_forward_output": 0.01,
                    "pid_right_error": -0.02,
                    "pid_right_p": -0.021,
                    "pid_right_i": -0.022,
                    "pid_right_d": -0.023,
                    "pid_right_output": -0.02,
                    "pid_up_error": 0.03,
                    "pid_up_p": 0.031,
                    "pid_up_i": 0.032,
                    "pid_up_d": 0.033,
                    "pid_up_output": 0.03,
                    "pid_yaw_error": -0.04,
                    "pid_yaw_p": -0.041,
                    "pid_yaw_i": -0.042,
                    "pid_yaw_d": -0.043,
                    "pid_yaw_output": -0.04,
                },
                pre_dock_ready=True,
                diagnostics={
                    "device": "/dev/video0",
                    "frame_width": 1920,
                    "frame_height": 1080,
                    "frame_fourcc": "MJPG",
                    "frame_fps": 30.0,
                    "detected_ids": [20],
                    "rejected_count": 4,
                    "tracker_frames_processed": 30,
                    "tracker_marker_frames": 28,
                    "tracker_target_frames": 27,
                    "tracker_valid_pose_frames": 26,
                    "tracker_invalid_pose_frames": 1,
                    "tracker_no_marker_frames": 2,
                    "tracker_target_id_missing_frames": 1,
                    "tracker_pnp_failed_frames": 0,
                    "tracker_quality_rejected_frames": 1,
                },
                new_pose=True,
                latest_pose_age_s=0.012,
                output_backend="mavlink_velocity",
                mavlink_command={"vx": 0.01, "vy": -0.02, "vz": -0.03, "yaw_rate": -0.04},
                rc_override={"ch1": 1500, "ch2": 1500, "ch3": 1500, "ch4": 1500},
                timestamp=123.456,
            )
            logger.close()

            with open(log_path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))

        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["marker_id"], "20")
        self.assertEqual(row["tracking_status"], "tracking")
        self.assertEqual(row["pre_dock_ready"], "1")
        self.assertEqual(row["pose_x"], "0.1")
        self.assertEqual(row["filtered_x"], "0.11")
        self.assertEqual(row["body_forward_m"], "0.81")
        self.assertEqual(row["body_right_m"], "0.11")
        self.assertEqual(row["body_up_m"], "0.19")
        self.assertEqual(row["yaw_raw_deg"], "182.5")
        self.assertEqual(row["yaw_error_deg"], "2.5")
        self.assertEqual(row["cmd_yaw_rate"], "-0.04")
        self.assertEqual(row["output_backend"], "mavlink_velocity")
        self.assertEqual(row["motion_forward_m_s"], "0.01")
        self.assertEqual(row["motion_right_m_s"], "-0.02")
        self.assertEqual(row["motion_up_m_s"], "0.03")
        self.assertEqual(row["motion_yaw_rate_rad_s"], "-0.04")
        self.assertEqual(row["raw_motion_forward_m_s"], "0.02")
        self.assertEqual(row["raw_motion_right_m_s"], "-0.04")
        self.assertEqual(row["raw_motion_up_m_s"], "0.06")
        self.assertEqual(row["raw_motion_yaw_rate_rad_s"], "-0.08")
        self.assertEqual(row["raw_forward_error_m"], "0.021")
        self.assertEqual(row["raw_right_error_m"], "-0.022")
        self.assertEqual(row["raw_up_error_m"], "0.023")
        self.assertEqual(row["raw_yaw_error_deg"], "2.4")
        self.assertEqual(row["deadbanded_forward_error_m"], "0.011")
        self.assertEqual(row["deadbanded_right_error_m"], "-0.012")
        self.assertEqual(row["deadbanded_up_error_m"], "0.013")
        self.assertEqual(row["deadbanded_yaw_error_deg"], "1.4")
        self.assertEqual(row["control_deadband_m"], "0.01")
        self.assertEqual(row["yaw_deadband_deg"], "1.0")
        self.assertEqual(row["command_smoothing_alpha"], "0.6")
        self.assertEqual(row["pid_forward_error"], "0.01")
        self.assertEqual(row["pid_forward_p"], "0.011")
        self.assertEqual(row["pid_forward_i"], "0.012")
        self.assertEqual(row["pid_forward_d"], "0.013")
        self.assertEqual(row["pid_forward_output"], "0.01")
        self.assertEqual(row["pid_right_error"], "-0.02")
        self.assertEqual(row["pid_up_output"], "0.03")
        self.assertEqual(row["pid_yaw_d"], "-0.043")
        self.assertEqual(row["pid_yaw_output"], "-0.04")
        self.assertEqual(row["mavlink_vz"], "-0.03")
        self.assertEqual(row["rc_ch1"], "1500")
        self.assertEqual(row["device"], "/dev/video0")
        self.assertEqual(row["frame_width"], "1920")
        self.assertEqual(row["frame_fourcc"], "MJPG")
        self.assertEqual(row["frame_fps"], "30.0")
        self.assertEqual(row["detected_ids"], "20")
        self.assertEqual(row["rejected_count"], "4")
        self.assertEqual(row["marker_pixel_size_px"], "82.5")
        self.assertEqual(row["reprojection_error_px"], "1.3")
        self.assertEqual(row["pose_valid"], "1")
        self.assertEqual(row["reject_reason"], "")
        self.assertEqual(row["new_pose"], "1")
        self.assertEqual(row["latest_pose_age_s"], "0.012")
        self.assertEqual(row["pre_dock_valid_frame_count"], "4")
        self.assertEqual(row["has_recent_valid_observation"], "1")
        self.assertEqual(row["pre_dock_recent_observation_max_age_s"], "0.5")
        self.assertEqual(row["pre_dock_recent_ok"], "1")
        self.assertEqual(row["pre_dock_valid_frames_ok"], "1")
        self.assertEqual(row["pre_dock_position_ok"], "0")
        self.assertEqual(row["pre_dock_distance_ok"], "1")
        self.assertEqual(row["pre_dock_yaw_ok"], "1")
        self.assertEqual(row["pre_dock_block_reason"], "position_error_high")
        self.assertEqual(row["tracker_frames_processed"], "30")
        self.assertEqual(row["tracker_marker_frames"], "28")
        self.assertEqual(row["tracker_target_frames"], "27")
        self.assertEqual(row["tracker_valid_pose_frames"], "26")
        self.assertEqual(row["tracker_invalid_pose_frames"], "1")
        self.assertEqual(row["tracker_no_marker_frames"], "2")
        self.assertEqual(row["tracker_target_id_missing_frames"], "1")
        self.assertEqual(row["tracker_pnp_failed_frames"], "0")
        self.assertEqual(row["tracker_quality_rejected_frames"], "1")

    def test_log_sample_does_not_copy_stale_pose_diagnostics_without_pose(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "tracking.csv"
            logger = TrackingDryRunLogger(log_path)

            logger.log_sample(
                pose=None,
                filtered_state={"status": "lost", "lost_frames": 12},
                diagnostics={
                    "device": "/dev/video0",
                    "frame_width": 1920,
                    "frame_height": 1080,
                    "detected_ids": [],
                    "rejected_count": 7,
                    "marker_pixel_size_px": 279.2,
                    "reprojection_error_px": 2.4,
                    "pose_valid": True,
                    "reject_reason": "no_marker",
                },
                timestamp=200.0,
            )
            logger.close()

            with open(log_path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))

        row = rows[0]
        self.assertEqual(row["detected"], "0")
        self.assertEqual(row["pose_valid"], "0")
        self.assertEqual(row["reject_reason"], "no_marker")
        self.assertEqual(row["marker_pixel_size_px"], "")
        self.assertEqual(row["reprojection_error_px"], "")


if __name__ == "__main__":
    unittest.main()
