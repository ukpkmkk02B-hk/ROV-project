import math

from modules.controller.motion_command import MotionCommand, camera_state_to_body_error, motion_command_from_mapping
from modules.controller.pid_controller import MultiAxisPidController


def _clamp(value, low, high):
    return max(low, min(high, value))


def _apply_deadband(value, deadband):
    deadband = abs(float(deadband or 0.0))
    return 0.0 if abs(float(value)) <= deadband else float(value)


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
        control_mode="p",
        pid_config=None,
        control_deadband_m=0.0,
        yaw_deadband_deg=0.0,
        command_smoothing_alpha=1.0,
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
        self.control_mode = str(control_mode or "p").lower()
        self.control_deadband_m = abs(float(control_deadband_m or 0.0))
        self.yaw_deadband_deg = abs(float(yaw_deadband_deg or 0.0))
        self.command_smoothing_alpha = _clamp(float(command_smoothing_alpha), 0.0, 1.0)
        self.pid = MultiAxisPidController(self._build_pid_config(pid_config or {}))
        self._last_smoothed_command = MotionCommand.neutral()

    def compute_command(self, state):
        state = self._body_error(state)
        raw_distance_error = float(state["forward_m"]) - self.desired_z_m
        raw_lateral_error = float(state["right_m"])
        raw_vertical_error = float(state.get("up_m", 0.0))
        raw_yaw_error_deg = float(state.get("yaw_error_deg", 0.0))
        distance_error = _apply_deadband(raw_distance_error, self.control_deadband_m)
        lateral_error = _apply_deadband(raw_lateral_error, self.control_deadband_m)
        vertical_error = _apply_deadband(raw_vertical_error, self.control_deadband_m)
        yaw_error_deg = _apply_deadband(raw_yaw_error_deg, self.yaw_deadband_deg)
        error_diagnostics = {
            "raw_forward_error_m": raw_distance_error,
            "raw_right_error_m": raw_lateral_error,
            "raw_up_error_m": raw_vertical_error,
            "raw_yaw_error_deg": raw_yaw_error_deg,
            "deadbanded_forward_error_m": distance_error,
            "deadbanded_right_error_m": lateral_error,
            "deadbanded_up_error_m": vertical_error,
            "deadbanded_yaw_error_deg": yaw_error_deg,
        }

        if self.control_mode == "pid":
            command = self._compute_pid_command(
                distance_error=distance_error,
                lateral_error=lateral_error,
                vertical_error=vertical_error,
                yaw_error_deg=yaw_error_deg,
                timestamp=state.get("timestamp"),
            )
        else:
            command = MotionCommand(
                _clamp(self.kp_distance * distance_error, -self.max_v_m_s, self.max_v_m_s),
                _clamp(self.kp_lateral * lateral_error, -self.max_v_m_s, self.max_v_m_s),
                _clamp(self.kp_vertical * vertical_error, -self.max_v_m_s, self.max_v_m_s),
                _clamp(
                    -self.kp_yaw * math.radians(yaw_error_deg),
                    -self.max_yaw_rate_rad_s,
                    self.max_yaw_rate_rad_s,
                ),
            ).as_dict()

        command.update(error_diagnostics)
        return self._finalize_command(command)

    def neutral_command(self):
        return MotionCommand.neutral().as_dict()

    def reset(self):
        self.pid.reset()
        self._last_smoothed_command = MotionCommand.neutral()

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

    def _compute_pid_command(self, distance_error, lateral_error, vertical_error, yaw_error_deg, timestamp=None):
        errors = {
            "forward": distance_error,
            "right": lateral_error,
            "up": vertical_error,
            "yaw": -math.radians(yaw_error_deg),
        }
        outputs, diagnostics = self.pid.update(errors, timestamp=timestamp)
        command = MotionCommand(
            outputs.get("forward", 0.0),
            outputs.get("right", 0.0),
            outputs.get("up", 0.0),
            outputs.get("yaw", 0.0),
        ).as_dict()
        command.update(diagnostics)
        return command

    def _finalize_command(self, command):
        raw = motion_command_from_mapping(command)
        smoothed = self._smooth_command(raw)
        finalized = dict(command)
        finalized.update(smoothed.as_dict())
        finalized.update(
            {
                "raw_motion_forward_m_s": raw.forward_m_s,
                "raw_motion_right_m_s": raw.right_m_s,
                "raw_motion_up_m_s": raw.up_m_s,
                "raw_motion_yaw_rate_rad_s": raw.yaw_rate_rad_s,
                "control_deadband_m": self.control_deadband_m,
                "yaw_deadband_deg": self.yaw_deadband_deg,
                "command_smoothing_alpha": self.command_smoothing_alpha,
            }
        )
        return finalized

    def _smooth_command(self, raw):
        alpha = self.command_smoothing_alpha
        if alpha >= 1.0:
            self._last_smoothed_command = raw
            return raw
        previous = self._last_smoothed_command
        smoothed = MotionCommand(
            forward_m_s=previous.forward_m_s * (1.0 - alpha) + raw.forward_m_s * alpha,
            right_m_s=previous.right_m_s * (1.0 - alpha) + raw.right_m_s * alpha,
            up_m_s=previous.up_m_s * (1.0 - alpha) + raw.up_m_s * alpha,
            yaw_rate_rad_s=previous.yaw_rate_rad_s * (1.0 - alpha) + raw.yaw_rate_rad_s * alpha,
        )
        self._last_smoothed_command = smoothed
        return smoothed

    def _build_pid_config(self, pid_config):
        return {
            "forward": self._axis_pid_config(pid_config, "forward", self.kp_distance, self.max_v_m_s),
            "right": self._axis_pid_config(pid_config, "right", self.kp_lateral, self.max_v_m_s),
            "up": self._axis_pid_config(pid_config, "up", self.kp_vertical, self.max_v_m_s),
            "yaw": self._axis_pid_config(
                pid_config,
                "yaw",
                self.kp_yaw,
                self.max_yaw_rate_rad_s,
                output_limit_deg_key="output_limit_deg_s",
                integral_limit_deg_key="integral_limit_deg",
            ),
        }

    def _axis_pid_config(
        self,
        pid_config,
        axis,
        default_kp,
        default_output_limit,
        output_limit_deg_key=None,
        integral_limit_deg_key=None,
    ):
        axis_config = dict((pid_config or {}).get(axis, {}) or {})
        output_limit = axis_config.get("output_limit", default_output_limit)
        if output_limit_deg_key and output_limit_deg_key in axis_config:
            output_limit = math.radians(float(axis_config[output_limit_deg_key]))
        integral_limit = axis_config.get("integral_limit")
        if integral_limit_deg_key and integral_limit_deg_key in axis_config:
            integral_limit = math.radians(float(axis_config[integral_limit_deg_key]))
        return {
            "kp": axis_config.get("kp", default_kp),
            "ki": axis_config.get("ki", 0.0),
            "kd": axis_config.get("kd", 0.0),
            "integral_limit": integral_limit,
            "output_limit": output_limit,
            "derivative_min_dt_s": axis_config.get("derivative_min_dt_s", 0.0),
            "d_limit": axis_config.get("d_limit"),
        }
