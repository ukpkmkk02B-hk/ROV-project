from dataclasses import dataclass


SAFE_CAMERA_TO_BODY_AXES = {
    "forward_axis": "y",
    "right_axis": "x",
    "up_axis": "z",
}


def _as_float(value, default=0.0):
    if value is None:
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def normalize_angle_deg(value):
    """Wrap an angle to [-180, 180) degrees."""
    angle = _as_float(value)
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def camera_to_body_axes_are_safe(config=None):
    config = config or {}
    mapping = config.get("camera_to_body", config) or {}
    return all(
        str(mapping.get(key, "")).lower() == expected
        for key, expected in SAFE_CAMERA_TO_BODY_AXES.items()
    )


@dataclass(frozen=True)
class MotionCommand:
    """ROV body-frame motion command used by visual tracking."""

    forward_m_s: float = 0.0
    right_m_s: float = 0.0
    up_m_s: float = 0.0
    yaw_rate_rad_s: float = 0.0

    @classmethod
    def neutral(cls):
        return cls()

    def as_dict(self):
        return {
            "forward_m_s": self.forward_m_s,
            "right_m_s": self.right_m_s,
            "up_m_s": self.up_m_s,
            "yaw_rate_rad_s": self.yaw_rate_rad_s,
            # Legacy aliases kept for existing status, logs, and callers.
            "vx": self.forward_m_s,
            "vy": self.right_m_s,
            "vz": self.up_m_s,
            "yaw_rate": self.yaw_rate_rad_s,
            "v_yaw": self.yaw_rate_rad_s,
        }

    def as_mavlink_body_ned(self):
        return {
            "vx": self.forward_m_s,
            "vy": self.right_m_s,
            "vz": -self.up_m_s,
            "yaw_rate": self.yaw_rate_rad_s,
        }


def motion_command_from_mapping(value):
    if isinstance(value, MotionCommand):
        return value
    value = value or {}
    if "forward_m_s" in value or "right_m_s" in value or "up_m_s" in value:
        return MotionCommand(
            forward_m_s=_as_float(value.get("forward_m_s")),
            right_m_s=_as_float(value.get("right_m_s")),
            up_m_s=_as_float(value.get("up_m_s")),
            yaw_rate_rad_s=_as_float(value.get("yaw_rate_rad_s", value.get("yaw_rate", value.get("v_yaw")))),
        )
    return MotionCommand(
        forward_m_s=_as_float(value.get("vx")),
        right_m_s=_as_float(value.get("vy")),
        up_m_s=_as_float(value.get("vz")),
        yaw_rate_rad_s=_as_float(value.get("yaw_rate", value.get("v_yaw"))),
    )


def camera_state_to_body_error(state, config=None):
    """Convert camera-frame ArUco state into ROV body-frame tracking errors."""
    state = state or {}
    config = config or {}
    mapping = config.get("camera_to_body", config) or {}
    axes = {
        "x": _as_float(state.get("x")),
        "y": _as_float(state.get("y")),
        "z": _as_float(state.get("z")),
    }

    def mapped_value(name, default_axis, default_sign):
        axis = str(mapping.get(f"{name}_axis", default_axis)).lower()
        sign = _as_float(mapping.get(f"{name}_sign", default_sign), default_sign)
        return axes.get(axis, 0.0) * sign

    body = dict(state)
    body["forward_m"] = mapped_value("forward", "z", 1.0)
    body["right_m"] = mapped_value("right", "x", 1.0)
    body["up_m"] = mapped_value("up", "y", -1.0)
    yaw_raw = _as_float(state.get("yaw"))
    yaw_sign = _as_float(mapping.get("yaw_sign", 1.0), 1.0)
    yaw_offset = _as_float(mapping.get("yaw_offset_deg", 0.0), 0.0)
    yaw_error = yaw_raw * yaw_sign + yaw_offset
    if bool(mapping.get("yaw_normalize", True)):
        yaw_error = normalize_angle_deg(yaw_error)
    body["yaw_raw_deg"] = yaw_raw
    body["yaw_error_deg"] = yaw_error
    return body
