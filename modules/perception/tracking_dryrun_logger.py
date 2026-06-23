import csv
from pathlib import Path


class TrackingDryRunLogger:
    FIELDNAMES = [
        "timestamp",
        "marker_id",
        "detected",
        "tracking_status",
        "lost_frames",
        "pre_dock_ready",
        "pose_x",
        "pose_y",
        "pose_z",
        "pose_roll",
        "pose_pitch",
        "pose_yaw",
        "center_u",
        "center_v",
        "filtered_x",
        "filtered_y",
        "filtered_z",
        "filtered_yaw",
        "cmd_vx",
        "cmd_vy",
        "cmd_vz",
        "cmd_yaw_rate",
    ]

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()

    def log_sample(self, pose=None, filtered_state=None, control_cmd=None, pre_dock_ready=False, timestamp=None):
        pose = pose or {}
        filtered_state = filtered_state or {}
        control_cmd = control_cmd or {}
        timestamp = timestamp if timestamp is not None else pose.get("timestamp", filtered_state.get("timestamp", ""))

        row = {
            "timestamp": timestamp,
            "marker_id": pose.get("id", ""),
            "detected": 1 if pose.get("detected") else 0,
            "tracking_status": filtered_state.get("status", ""),
            "lost_frames": filtered_state.get("lost_frames", ""),
            "pre_dock_ready": 1 if pre_dock_ready else 0,
            "pose_x": pose.get("x", ""),
            "pose_y": pose.get("y", ""),
            "pose_z": pose.get("z", ""),
            "pose_roll": pose.get("roll", ""),
            "pose_pitch": pose.get("pitch", ""),
            "pose_yaw": pose.get("yaw", ""),
            "center_u": pose.get("center_u", ""),
            "center_v": pose.get("center_v", ""),
            "filtered_x": filtered_state.get("x", ""),
            "filtered_y": filtered_state.get("y", ""),
            "filtered_z": filtered_state.get("z", ""),
            "filtered_yaw": filtered_state.get("yaw", ""),
            "cmd_vx": control_cmd.get("vx", ""),
            "cmd_vy": control_cmd.get("vy", ""),
            "cmd_vz": control_cmd.get("vz", ""),
            "cmd_yaw_rate": control_cmd.get("yaw_rate", ""),
        }
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        if not self._file.closed:
            self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
