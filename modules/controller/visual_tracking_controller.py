import math

from modules.controller.motion_command import MotionCommand, camera_state_to_body_error


def _clamp(value, low, high):
    return max(low, min(high, value))


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
        if not state or state.get("status") in {"lost", "predicted"}:
            return False
        if state.get("has_valid_observation") is False:
            return False
        valid_count = int(state.get("valid_observation_count", self.min_pre_dock_valid_frames))
        if valid_count < self.min_pre_dock_valid_frames:
            return False

        state = self._body_error(state)
        return (
            abs(float(state["right_m"])) <= self.pre_dock_position_tolerance_m
            and abs(float(state.get("up_m", 0.0))) <= self.pre_dock_position_tolerance_m
            and abs(float(state["forward_m"]) - self.desired_z_m) <= self.pre_dock_distance_tolerance_m
            and abs(float(state.get("yaw_error_deg", 0.0))) <= self.pre_dock_yaw_tolerance_deg
        )

    def _body_error(self, state):
        if all(key in state for key in ("forward_m", "right_m", "up_m", "yaw_error_deg")):
            return state
        return camera_state_to_body_error(state, {"camera_to_body": self.camera_to_body})
