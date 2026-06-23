import math

from modules.controller.motion_command import MotionCommand, camera_state_to_body_error


def _clamp(value, low, high):
    return max(low, min(high, value))


def _optional_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class VisualTrackingController:
    """PD-style visual tracking controller with safe velocity limits."""

    def __init__(
        self,
        desired_z_m=0.8,
        max_v_m_s=0.4,
        max_yaw_rate_deg_s=25.0,
        kp_lateral=0.4,
        kp_vertical=0.3,
        kp_distance=0.4,
        kp_yaw=1.0,
        pre_dock_position_tolerance_m=0.05,
        pre_dock_distance_tolerance_m=0.05,
        pre_dock_yaw_tolerance_deg=5.0,
        camera_to_body=None,
        min_pre_dock_valid_frames=1,
        pre_dock_recent_observation_max_age_s=0.5,
    ):
        self.desired_z_m = float(desired_z_m)
        self.max_v_m_s = float(max_v_m_s)
        self.max_yaw_rate_rad_s = math.radians(float(max_yaw_rate_deg_s))
        self.kp_lateral = float(kp_lateral)
        self.kp_vertical = float(kp_vertical)
        self.kp_distance = float(kp_distance)
        self.kp_yaw = float(kp_yaw)
        self.pre_dock_position_tolerance_m = float(pre_dock_position_tolerance_m)
        self.pre_dock_distance_tolerance_m = float(pre_dock_distance_tolerance_m)
        self.pre_dock_yaw_tolerance_deg = float(pre_dock_yaw_tolerance_deg)
        self.camera_to_body = camera_to_body or {}
        self.min_pre_dock_valid_frames = int(min_pre_dock_valid_frames)
        self.pre_dock_recent_observation_max_age_s = float(pre_dock_recent_observation_max_age_s)

    def compute_command(self, state):
        state = self._body_error(state)
        distance_error = float(state["forward_m"]) - self.desired_z_m
        lateral_error = float(state["right_m"])
        vertical_error = float(state.get("up_m", 0.0))
        yaw_error_deg = float(state.get("yaw_error_deg", 0.0))

        forward = _clamp(self.kp_distance * distance_error, -self.max_v_m_s, self.max_v_m_s)
        right = _clamp(self.kp_lateral * lateral_error, -self.max_v_m_s, self.max_v_m_s)
        up = _clamp(self.kp_vertical * vertical_error, -self.max_v_m_s, self.max_v_m_s)
        yaw_rate = _clamp(
            -self.kp_yaw * math.radians(yaw_error_deg),
            -self.max_yaw_rate_rad_s,
            self.max_yaw_rate_rad_s,
        )

        return MotionCommand(forward, right, up, yaw_rate).as_dict()

    def neutral_command(self):
        return MotionCommand.neutral().as_dict()

    def is_pre_dock_ready(self, state):
        return self.pre_dock_diagnostics(state)["pre_dock_ready"]

    def pre_dock_diagnostics(self, state):
        if not state:
            return self._pre_dock_result(False, "no_state")

        if state.get("status") == "lost":
            return self._pre_dock_result(False, "lost")

        has_recent = bool(state.get("has_recent_valid_observation", state.get("has_valid_observation", False)))
        latest_pose_age = _optional_float(state.get("latest_pose_age_s"))
        if latest_pose_age is not None and latest_pose_age > self.pre_dock_recent_observation_max_age_s:
            has_recent = False
        if not has_recent:
            return self._pre_dock_result(False, "recent_observation_expired", latest_pose_age=latest_pose_age)

        valid_count = int(
            state.get("pre_dock_valid_frame_count", state.get("valid_observation_count", self.min_pre_dock_valid_frames))
        )
        valid_frames_ok = valid_count >= self.min_pre_dock_valid_frames
        if not valid_frames_ok:
            return self._pre_dock_result(
                False,
                "valid_frame_count_low",
                latest_pose_age=latest_pose_age,
                valid_count=valid_count,
                valid_frames_ok=False,
            )

        state = self._body_error(state)
        right = abs(float(state["right_m"]))
        up = abs(float(state.get("up_m", 0.0)))
        distance_error = abs(float(state["forward_m"]) - self.desired_z_m)
        yaw_error = abs(float(state.get("yaw_error_deg", 0.0)))

        position_ok = right <= self.pre_dock_position_tolerance_m and up <= self.pre_dock_position_tolerance_m
        distance_ok = distance_error <= self.pre_dock_distance_tolerance_m
        yaw_ok = yaw_error <= self.pre_dock_yaw_tolerance_deg

        if not position_ok:
            return self._pre_dock_result(
                False,
                "position_error_high",
                latest_pose_age=latest_pose_age,
                valid_count=valid_count,
                position_ok=False,
                distance_ok=distance_ok,
                yaw_ok=yaw_ok,
            )
        if not distance_ok:
            return self._pre_dock_result(
                False,
                "distance_error_high",
                latest_pose_age=latest_pose_age,
                valid_count=valid_count,
                position_ok=position_ok,
                distance_ok=False,
                yaw_ok=yaw_ok,
            )
        if not yaw_ok:
            return self._pre_dock_result(
                False,
                "yaw_error_high",
                latest_pose_age=latest_pose_age,
                valid_count=valid_count,
                position_ok=position_ok,
                distance_ok=distance_ok,
                yaw_ok=False,
            )

        return self._pre_dock_result(
            True,
            "",
            latest_pose_age=latest_pose_age,
            valid_count=valid_count,
            position_ok=True,
            distance_ok=True,
            yaw_ok=True,
        )

    def _pre_dock_result(
        self,
        ready,
        reason,
        latest_pose_age=None,
        valid_count=None,
        valid_frames_ok=True,
        position_ok=True,
        distance_ok=True,
        yaw_ok=True,
    ):
        if valid_count is None:
            valid_count = self.min_pre_dock_valid_frames
        recent_ok = reason not in {"no_state", "lost", "recent_observation_expired"}
        return {
            "pre_dock_ready": bool(ready),
            "pre_dock_block_reason": reason,
            "pre_dock_valid_frame_count": int(valid_count),
            "pre_dock_recent_observation_max_age_s": self.pre_dock_recent_observation_max_age_s,
            "pre_dock_recent_ok": bool(recent_ok),
            "pre_dock_valid_frames_ok": bool(valid_frames_ok),
            "pre_dock_position_ok": bool(position_ok),
            "pre_dock_distance_ok": bool(distance_ok),
            "pre_dock_yaw_ok": bool(yaw_ok),
            "latest_pose_age_s": "" if latest_pose_age is None else latest_pose_age,
            "has_recent_valid_observation": bool(recent_ok),
        }

    def _body_error(self, state):
        if all(key in state for key in ("forward_m", "right_m", "up_m", "yaw_error_deg")):
            return state
        return camera_state_to_body_error(state, {"camera_to_body": self.camera_to_body})
