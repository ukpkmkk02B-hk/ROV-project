import math
import time

from modules.controller.motion_command import MotionCommand, motion_command_from_mapping


def _clamp(value, low, high):
    return max(low, min(high, value))


class VisualAxisPolicy:
    """Runtime axis policy for visual tracking and pre-align stages."""

    TRACKING_VERTICAL_VISUAL_PID = "visual_pid"
    TRACKING_VERTICAL_HOLD_CH3 = "hold_captured_ch3"
    VALID_TRACKING_VERTICAL_MODES = {
        TRACKING_VERTICAL_VISUAL_PID,
        TRACKING_VERTICAL_HOLD_CH3,
    }

    PRE_ALIGN_FULL_CONTROL = "full_control"
    PRE_ALIGN_SMALL_CORRECTION = "small_correction"
    PRE_ALIGN_LOCK_HORIZONTAL = "lock_horizontal"
    VALID_PRE_ALIGN_AXIS_MODES = {
        PRE_ALIGN_FULL_CONTROL,
        PRE_ALIGN_SMALL_CORRECTION,
        PRE_ALIGN_LOCK_HORIZONTAL,
    }

    def __init__(self, config=None, up_channel="ch3"):
        config = config or {}
        self.up_channel = up_channel or "ch3"
        self.captured_hold_ch3_pwm = _optional_int(config.get("captured_hold_ch3_pwm"))
        self.captured_hold_ch3_time = None
        self.captured_hold_ch3_source = ""
        self.capture_rejected_reason = ""
        self.tracking_vertical_rejected_reason = ""
        self.pre_align_mode_rejected_reason = ""

        self.tracking_vertical_mode = self.TRACKING_VERTICAL_VISUAL_PID
        requested_vertical_mode = str(config.get("tracking_vertical_mode", self.TRACKING_VERTICAL_VISUAL_PID))
        if requested_vertical_mode in self.VALID_TRACKING_VERTICAL_MODES:
            if requested_vertical_mode == self.TRACKING_VERTICAL_HOLD_CH3 and self.captured_hold_ch3_pwm is None:
                self.tracking_vertical_rejected_reason = "ch3_not_captured"
            else:
                self.tracking_vertical_mode = requested_vertical_mode
        else:
            self.tracking_vertical_rejected_reason = "invalid_tracking_vertical_mode"

        requested_pre_align_mode = str(config.get("pre_align_axis_mode", self.PRE_ALIGN_SMALL_CORRECTION))
        self.pre_align_axis_mode = (
            requested_pre_align_mode
            if requested_pre_align_mode in self.VALID_PRE_ALIGN_AXIS_MODES
            else self.PRE_ALIGN_SMALL_CORRECTION
        )
        if requested_pre_align_mode not in self.VALID_PRE_ALIGN_AXIS_MODES:
            self.pre_align_mode_rejected_reason = "invalid_pre_align_axis_mode"

        self.pre_align_correction_scale = float(config.get("pre_align_correction_scale", 0.25))
        self.pre_align_max_v_m_s = abs(float(config.get("pre_align_max_v_m_s", 0.05)))
        self.pre_align_max_yaw_rate_rad_s = math.radians(
            abs(float(config.get("pre_align_max_yaw_rate_deg_s", 3.0)))
        )

    def set_tracking_vertical_mode(self, mode):
        mode = str(mode)
        if mode not in self.VALID_TRACKING_VERTICAL_MODES:
            self.tracking_vertical_rejected_reason = "invalid_tracking_vertical_mode"
            return self._result(False, self.tracking_vertical_rejected_reason, "tracking_vertical_mode", mode)
        if mode == self.TRACKING_VERTICAL_HOLD_CH3 and self.captured_hold_ch3_pwm is None:
            self.tracking_vertical_rejected_reason = "ch3_not_captured"
            return self._result(False, self.tracking_vertical_rejected_reason, "tracking_vertical_mode", mode)
        self.tracking_vertical_mode = mode
        self.tracking_vertical_rejected_reason = ""
        return self._result(True, "", "tracking_vertical_mode", mode)

    def set_pre_align_axis_mode(self, mode):
        mode = str(mode)
        if mode not in self.VALID_PRE_ALIGN_AXIS_MODES:
            self.pre_align_mode_rejected_reason = "invalid_pre_align_axis_mode"
            return self._result(False, self.pre_align_mode_rejected_reason, "pre_align_axis_mode", mode)
        self.pre_align_axis_mode = mode
        self.pre_align_mode_rejected_reason = ""
        return self._result(True, "", "pre_align_axis_mode", mode)

    def capture_ch3(self, pwm, timestamp=None, source="surface"):
        captured = _optional_int(pwm)
        if captured is None:
            self.capture_rejected_reason = "invalid_ch3_pwm"
            return self._capture_result(False, self.capture_rejected_reason, source)
        self.captured_hold_ch3_pwm = captured
        self.captured_hold_ch3_time = time.time() if timestamp is None else float(timestamp)
        self.captured_hold_ch3_source = source
        self.capture_rejected_reason = ""
        return self._capture_result(True, "", source)

    def apply(self, command, stage):
        motion = motion_command_from_mapping(command)
        forward = motion.forward_m_s
        right = motion.right_m_s
        up = motion.up_m_s
        yaw_rate = motion.yaw_rate_rad_s

        vertical_hold_active = False
        pre_align_horizontal_scaled = False
        pre_align_horizontal_locked = False

        if self.tracking_vertical_mode == self.TRACKING_VERTICAL_HOLD_CH3:
            up = 0.0
            vertical_hold_active = True

        if _is_pre_align_stage(stage):
            if self.pre_align_axis_mode == self.PRE_ALIGN_LOCK_HORIZONTAL:
                forward = 0.0
                right = 0.0
                yaw_rate = 0.0
                pre_align_horizontal_locked = True
            elif self.pre_align_axis_mode == self.PRE_ALIGN_SMALL_CORRECTION:
                forward = _clamp(
                    forward * self.pre_align_correction_scale,
                    -self.pre_align_max_v_m_s,
                    self.pre_align_max_v_m_s,
                )
                right = _clamp(
                    right * self.pre_align_correction_scale,
                    -self.pre_align_max_v_m_s,
                    self.pre_align_max_v_m_s,
                )
                yaw_rate = _clamp(
                    yaw_rate * self.pre_align_correction_scale,
                    -self.pre_align_max_yaw_rate_rad_s,
                    self.pre_align_max_yaw_rate_rad_s,
                )
                pre_align_horizontal_scaled = True

        adjusted = dict(command or {})
        adjusted.update(MotionCommand(forward, right, up, yaw_rate).as_dict())
        adjusted.update(self.status())
        adjusted.update(
            {
                "vertical_hold_active": vertical_hold_active,
                "pre_align_horizontal_scaled": pre_align_horizontal_scaled,
                "pre_align_horizontal_locked": pre_align_horizontal_locked,
            }
        )
        return adjusted

    def apply_rc_override(self, channels, stage=None):
        adjusted = dict(channels or {})
        if self.tracking_vertical_mode == self.TRACKING_VERTICAL_HOLD_CH3 and self.captured_hold_ch3_pwm is not None:
            adjusted[self.up_channel] = int(self.captured_hold_ch3_pwm)
        return adjusted

    def validate_for_motion(self):
        if self.tracking_vertical_mode == self.TRACKING_VERTICAL_HOLD_CH3 and self.captured_hold_ch3_pwm is None:
            raise ValueError("hold_captured_ch3 requires captured_hold_ch3_pwm before motion output")
        return True

    def status(self):
        return {
            "tracking_vertical_mode": self.tracking_vertical_mode,
            "captured_hold_ch3_available": self.captured_hold_ch3_pwm is not None,
            "captured_hold_ch3_pwm": self.captured_hold_ch3_pwm,
            "captured_hold_ch3_time": self.captured_hold_ch3_time,
            "captured_hold_ch3_source": self.captured_hold_ch3_source,
            "capture_ch3_rejected_reason": self.capture_rejected_reason,
            "tracking_vertical_rejected_reason": self.tracking_vertical_rejected_reason,
            "pre_align_axis_mode": self.pre_align_axis_mode,
            "pre_align_mode_rejected_reason": self.pre_align_mode_rejected_reason,
            "pre_align_correction_scale": self.pre_align_correction_scale,
            "pre_align_max_v_m_s": self.pre_align_max_v_m_s,
            "pre_align_max_yaw_rate_deg_s": math.degrees(self.pre_align_max_yaw_rate_rad_s),
        }

    def _result(self, accepted, reason, key, mode):
        result = {
            "accepted": bool(accepted),
            "reason": reason,
            key: mode,
        }
        result.update(self.status())
        return result

    def _capture_result(self, accepted, reason, source):
        result = {
            "accepted": bool(accepted),
            "reason": reason,
            "source": source,
        }
        result.update(self.status())
        return result


def _optional_int(value):
    if value in (None, ""):
        return None
    try:
        return int(round(float(value)))
    except (TypeError, ValueError):
        return None


def _is_pre_align_stage(stage):
    return str(stage or "").lower() in {"pre_align", "prealign", "align"}
