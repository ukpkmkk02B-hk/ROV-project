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
                pose={"id": 20, "x": 0.1, "y": -0.2, "z": 0.8, "yaw": 3.0, "center_u": 960.0, "center_v": 540.0},
                filtered_state={"x": 0.11, "y": -0.19, "z": 0.81, "yaw": 2.5, "status": "tracking", "lost_frames": 0},
                control_cmd={"vx": 0.01, "vy": -0.02, "vz": 0.03, "yaw_rate": -0.04},
                pre_dock_ready=True,
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
        self.assertEqual(row["cmd_yaw_rate"], "-0.04")


if __name__ == "__main__":
    unittest.main()
