from pathlib import Path


CONFIG_FIELDS = {
    "desired_z_m": {"type": float, "min": 0.05, "max": 5.0},
    "max_v_m_s": {"type": float, "min": 0.0, "max": 1.0},
    "max_yaw_rate_deg_s": {"type": float, "min": 0.0, "max": 25.0},
    "min_marker_pixel_size_px": {"type": float, "min": 0.0, "max": 5000.0},
    "max_reprojection_error_px": {"type": float, "min": 0.0, "max": 100.0},
    "camera_to_body.yaw_offset_deg": {"type": float, "min": -180.0, "max": 180.0},
    "control_deadband_m": {"type": float, "min": 0.0, "max": 0.5},
    "yaw_deadband_deg": {"type": float, "min": 0.0, "max": 30.0},
    "command_smoothing_alpha": {"type": float, "min": 0.0, "max": 1.0},
    "tracking_vertical_mode": {"type": str, "choices": {"visual_pid", "hold_captured_ch3"}},
    "pre_align_axis_mode": {"type": str, "choices": {"full_control", "small_correction", "lock_horizontal"}},
    "pre_align_correction_scale": {"type": float, "min": 0.0, "max": 1.0},
    "pre_align_max_v_m_s": {"type": float, "min": 0.0, "max": 1.0},
    "pre_align_max_yaw_rate_deg_s": {"type": float, "min": 0.0, "max": 25.0},
    "pid.forward.kp": {"type": float, "min": 0.0, "max": 5.0},
    "pid.right.kp": {"type": float, "min": 0.0, "max": 5.0},
    "pid.up.kp": {"type": float, "min": 0.0, "max": 5.0},
    "pid.yaw.kp": {"type": float, "min": 0.0, "max": 5.0},
    "pid.forward.output_limit": {"type": float, "min": 0.0, "max": 1.0},
    "pid.right.output_limit": {"type": float, "min": 0.0, "max": 1.0},
    "pid.up.output_limit": {"type": float, "min": 0.0, "max": 1.0},
    "rc_override.pwm_per_m_s": {"type": float, "min": 0.0, "max": 2000.0},
    "rc_override.pwm_per_rad_s": {"type": float, "min": 0.0, "max": 2000.0},
    "rc_override.min_active_pwm_offset": {"type": float, "min": 0.0, "max": 100.0},
    "enable_motion": {"type": bool},
    "min_pre_dock_valid_frames": {"type": int, "min": 1, "max": 100},
    "pre_dock_recent_observation_max_age_s": {"type": float, "min": 0.05, "max": 5.0},
    "pre_dock_position_tolerance_m": {"type": float, "min": 0.001, "max": 1.0},
    "pre_dock_distance_tolerance_m": {"type": float, "min": 0.001, "max": 1.0},
    "pre_dock_yaw_tolerance_deg": {"type": float, "min": 0.1, "max": 45.0},
}


def read_console_config(path):
    values = _read_vision_scalars(Path(path).read_text(encoding="utf-8"))
    return {field: values[field] for field in CONFIG_FIELDS if field in values}


def update_console_config(path, updates, confirm_motion=False):
    unknown = sorted(set(updates) - set(CONFIG_FIELDS))
    if unknown:
        raise ValueError(f"unsupported config fields: {', '.join(unknown)}")

    normalized = {}
    for field, value in updates.items():
        normalized[field] = _coerce_and_validate(field, value)

    if normalized.get("enable_motion") is True and not confirm_motion:
        raise PermissionError("enable_motion=true requires confirm_motion=true")

    path = Path(path)
    text = path.read_text(encoding="utf-8")
    text = _update_vision_scalars(text, normalized)
    path.write_text(text, encoding="utf-8")
    return read_console_config(path)


def _read_vision_scalars(text):
    lines = text.splitlines()
    start, end = _vision_section_bounds(lines)
    values = {}
    for idx in range(start + 1, end):
        line = lines[idx]
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        if not line.startswith("  "):
            continue
        key, raw_value = stripped.split(":", 1)
        if raw_value.strip() == "":
            continue
        path = _field_path_for_line(lines, start, idx, key)
        if path in CONFIG_FIELDS:
            values[path] = _parse_scalar(raw_value.strip())
    return values


def _update_vision_scalars(text, updates):
    if not updates:
        return text

    lines = text.splitlines()
    start, end = _vision_section_bounds(lines)
    remaining = dict(updates)
    for idx in range(start + 1, end):
        line = lines[idx]
        stripped = line.strip()
        if not line.startswith("  ") or ":" not in stripped:
            continue
        key = stripped.split(":", 1)[0]
        path = _field_path_for_line(lines, start, idx, key)
        if path in remaining:
            prefix = line[: len(line) - len(line.lstrip())]
            lines[idx] = f"{prefix}{key}: {_format_scalar(remaining.pop(path))}"

    if remaining:
        missing = ", ".join(sorted(remaining))
        raise ValueError(f"missing vision_tracking config fields: {missing}")

    trailing_newline = "\n" if text.endswith("\n") else ""
    return "\n".join(lines) + trailing_newline


def _vision_section_bounds(lines):
    start = None
    for idx, line in enumerate(lines):
        if line.strip() == "vision_tracking:" and not line.startswith(" "):
            start = idx
            break
    if start is None:
        raise ValueError("vision_tracking section not found")

    end = len(lines)
    for idx in range(start + 1, len(lines)):
        line = lines[idx]
        if line.strip() and not line.startswith(" "):
            end = idx
            break
    return start, end


def _field_path_for_line(lines, section_start, line_index, key):
    line = lines[line_index]
    indent = len(line) - len(line.lstrip(" "))
    parents = []
    current_indent = indent
    for prev in reversed(lines[section_start + 1 : line_index]):
        prev_stripped = prev.strip()
        if not prev_stripped or prev_stripped.startswith("#") or ":" not in prev_stripped:
            continue
        prev_indent = len(prev) - len(prev.lstrip(" "))
        if prev_indent < 2 or prev_indent >= current_indent:
            continue
        prev_key, prev_value = prev_stripped.split(":", 1)
        if prev_value.strip() == "":
            parents.append((prev_indent, prev_key))
            current_indent = prev_indent
    return ".".join([name for _indent, name in reversed(parents)] + [key])


def _coerce_and_validate(field, value):
    spec = CONFIG_FIELDS[field]
    value_type = spec["type"]
    if value_type is bool:
        if isinstance(value, bool):
            coerced = value
        elif isinstance(value, str) and value.lower() in {"true", "false"}:
            coerced = value.lower() == "true"
        else:
            raise ValueError(f"{field} must be a boolean")
    elif value_type is int:
        if isinstance(value, bool):
            raise ValueError(f"{field} must be an integer")
        coerced = int(value)
    elif value_type is str:
        coerced = str(value)
        choices = spec.get("choices")
        if choices and coerced not in choices:
            raise ValueError(f"{field} must be one of: {', '.join(sorted(choices))}")
    else:
        if isinstance(value, bool):
            raise ValueError(f"{field} must be a number")
        coerced = float(value)

    if "min" in spec and coerced < spec["min"]:
        raise ValueError(f"{field} must be >= {spec['min']}")
    if "max" in spec and coerced > spec["max"]:
        raise ValueError(f"{field} must be <= {spec['max']}")
    return coerced


def _parse_scalar(value):
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        if "." not in value and "e" not in value.lower():
            return int(value)
        return float(value)
    except ValueError:
        return value.strip('"').strip("'")


def _format_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)
