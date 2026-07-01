from pathlib import Path


CONFIG_FIELDS = {
    "desired_z_m": {"type": float, "min": 0.05, "max": 5.0},
    "max_v_m_s": {"type": float, "min": 0.0, "max": 0.4},
    "max_yaw_rate_deg_s": {"type": float, "min": 0.0, "max": 25.0},
    "control_deadband_m": {"type": float, "min": 0.0, "max": 0.5},
    "yaw_deadband_deg": {"type": float, "min": 0.0, "max": 30.0},
    "command_smoothing_alpha": {"type": float, "min": 0.0, "max": 1.0},
    "tracking_vertical_mode": {"type": str, "choices": {"visual_pid", "hold_captured_ch3"}},
    "pre_align_axis_mode": {"type": str, "choices": {"full_control", "small_correction", "lock_horizontal"}},
    "pre_align_correction_scale": {"type": float, "min": 0.0, "max": 1.0},
    "pre_align_max_v_m_s": {"type": float, "min": 0.0, "max": 0.4},
    "pre_align_max_yaw_rate_deg_s": {"type": float, "min": 0.0, "max": 25.0},
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
    for line in lines[start + 1 : end]:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        if not line.startswith("  ") or line.startswith("    "):
            continue
        key, raw_value = stripped.split(":", 1)
        if key in CONFIG_FIELDS:
            values[key] = _parse_scalar(raw_value.strip())
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
        if not line.startswith("  ") or line.startswith("    ") or ":" not in stripped:
            continue
        key = stripped.split(":", 1)[0]
        if key in remaining:
            prefix = line[: len(line) - len(line.lstrip())]
            lines[idx] = f"{prefix}{key}: {_format_scalar(remaining.pop(key))}"

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
