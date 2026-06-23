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
        "device",
        "frame_width",
        "frame_height",
        "detected_ids",
        "rejected_count",
        "marker_pixel_size_px",
        "reprojection_error_px",
        "pose_valid",
        "reject_reason",
        "filtered_x",
        "filtered_y",
        "filtered_z",
        "filtered_yaw",
        "cmd_vx",
        "cmd_vy",
        "cmd_vz",
        "cmd_yaw_rate",
        "output_backend",
        "motion_forward_m_s",
        "motion_right_m_s",
        "motion_up_m_s",
        "motion_yaw_rate_rad_s",
        "mavlink_vx",
        "mavlink_vy",
        "mavlink_vz",
        "mavlink_yaw_rate",
        "rc_ch1",
        "rc_ch2",
        "rc_ch3",
        "rc_ch4",
        "rc_ch5",
        "rc_ch6",
        "rc_ch7",
        "rc_ch8",
    ]

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()

    def log_sample(
        self,
        pose=None,
        filtered_state=None,
        control_cmd=None,
        pre_dock_ready=False,
        diagnostics=None,
        output_backend="",
        mavlink_command=None,
        rc_override=None,
        timestamp=None,
    ):
        pose = pose or {}
        filtered_state = filtered_state or {}
        control_cmd = control_cmd or {}
        diagnostics = diagnostics or {}
        mavlink_command = mavlink_command or {}
        rc_override = rc_override or {}
        timestamp = timestamp if timestamp is not None else pose.get("timestamp", filtered_state.get("timestamp", ""))
        detected_ids = diagnostics.get("detected_ids", "")
        if isinstance(detected_ids, (list, tuple)):
            detected_ids = ",".join(str(v) for v in detected_ids)

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
            "device": diagnostics.get("device", ""),
            "frame_width": diagnostics.get("frame_width", ""),
            "frame_height": diagnostics.get("frame_height", ""),
            "detected_ids": detected_ids,
            "rejected_count": diagnostics.get("rejected_count", ""),
            "marker_pixel_size_px": pose.get("marker_pixel_size_px", diagnostics.get("marker_pixel_size_px", "")),
            "reprojection_error_px": pose.get("reprojection_error_px", diagnostics.get("reprojection_error_px", "")),
            "pose_valid": 1 if pose.get("pose_valid", diagnostics.get("pose_valid", False)) else 0,
            "reject_reason": pose.get("reject_reason", diagnostics.get("reject_reason", "")),
            "filtered_x": filtered_state.get("x", ""),
            "filtered_y": filtered_state.get("y", ""),
            "filtered_z": filtered_state.get("z", ""),
            "filtered_yaw": filtered_state.get("yaw", ""),
            "cmd_vx": control_cmd.get("vx", ""),
            "cmd_vy": control_cmd.get("vy", ""),
            "cmd_vz": control_cmd.get("vz", ""),
            "cmd_yaw_rate": control_cmd.get("yaw_rate", ""),
            "output_backend": output_backend,
            "motion_forward_m_s": control_cmd.get("forward_m_s", ""),
            "motion_right_m_s": control_cmd.get("right_m_s", ""),
            "motion_up_m_s": control_cmd.get("up_m_s", ""),
            "motion_yaw_rate_rad_s": control_cmd.get("yaw_rate_rad_s", ""),
            "mavlink_vx": mavlink_command.get("vx", ""),
            "mavlink_vy": mavlink_command.get("vy", ""),
            "mavlink_vz": mavlink_command.get("vz", ""),
            "mavlink_yaw_rate": mavlink_command.get("yaw_rate", ""),
            "rc_ch1": rc_override.get("ch1", ""),
            "rc_ch2": rc_override.get("ch2", ""),
            "rc_ch3": rc_override.get("ch3", ""),
            "rc_ch4": rc_override.get("ch4", ""),
            "rc_ch5": rc_override.get("ch5", ""),
            "rc_ch6": rc_override.get("ch6", ""),
            "rc_ch7": rc_override.get("ch7", ""),
            "rc_ch8": rc_override.get("ch8", ""),
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
