from modules.controller.motion_command import MotionCommand, motion_command_from_mapping


class RcOverrideMapper:
    """Optional dry-run mapper from body motion command to RC override channels."""

    VALID_CHANNELS = {f"ch{i}" for i in range(1, 9)}
    REQUIRED_AXES = ("forward", "right", "up", "yaw")

    def __init__(self, config=None):
        self.config = config or {}
        self.enabled = bool(self.config.get("enabled", False))
        self.channels = dict(self.config.get("channels") or {})
        self.neutral_pwm = int(self.config.get("neutral_pwm", 1500))
        self.min_pwm = int(self.config.get("min_pwm", 1400))
        self.max_pwm = int(self.config.get("max_pwm", 1600))
        self.pwm_per_m_s = float(self.config.get("pwm_per_m_s", 250))
        self.pwm_per_rad_s = float(self.config.get("pwm_per_rad_s", 200))
        self.min_active_pwm_offset = abs(float(self.config.get("min_active_pwm_offset", 0)))
        self.axis_signs = dict(self.config.get("axis_signs") or {})

    def validate_for_motion(self, require_enabled=False):
        if require_enabled and not self.enabled:
            raise ValueError("rc_override must be enabled before real motion output")
        if not self.enabled:
            return True
        missing = [axis for axis in self.REQUIRED_AXES if axis not in self.channels]
        if missing:
            raise ValueError(f"rc_override missing channel mapping for: {', '.join(missing)}")
        for axis, channel in self.channels.items():
            if channel not in self.VALID_CHANNELS:
                raise ValueError(f"rc_override invalid channel for {axis}: {channel}")
        return True

    def neutral_channels(self):
        return {f"ch{i}": self.neutral_pwm for i in range(1, 9)}

    def map_motion_command(self, command):
        if not self.enabled:
            return {}
        command = motion_command_from_mapping(command)
        channels = self.neutral_channels()
        values = {
            "forward": (command.forward_m_s, self.pwm_per_m_s),
            "right": (command.right_m_s, self.pwm_per_m_s),
            "up": (command.up_m_s, self.pwm_per_m_s),
            "yaw": (command.yaw_rate_rad_s, self.pwm_per_rad_s),
        }
        for axis, (value, gain) in values.items():
            channel = self.channels.get(axis)
            if not channel:
                continue
            sign = float(self.axis_signs.get(axis, 1.0))
            offset = self._apply_min_active_offset(value * gain * sign)
            channels[channel] = self._clamp_pwm(self.neutral_pwm + offset)
        return channels

    def _apply_min_active_offset(self, offset):
        if offset == 0 or self.min_active_pwm_offset <= 0:
            return offset
        if abs(offset) < 0.5:
            return 0.0
        if abs(offset) >= self.min_active_pwm_offset:
            return offset
        return self.min_active_pwm_offset if offset > 0 else -self.min_active_pwm_offset

    def _clamp_pwm(self, value):
        return int(round(max(self.min_pwm, min(self.max_pwm, value))))
