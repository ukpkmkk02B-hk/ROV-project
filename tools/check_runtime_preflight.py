import argparse
import importlib
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from modules.controller.motion_command import camera_to_body_axes_are_safe


REQUIRED_MODULES_BY_MARKER = {
    "aruco": ["cv2.aruco", "numpy", "yaml", "serial", "pymavlink.mavutil"],
    "apriltag": ["cv2", "numpy", "yaml", "serial", "pymavlink.mavutil", "pupil_apriltags"],
}


def default_module_checker(module_name):
    if module_name == "cv2.aruco":
        cv2 = importlib.import_module("cv2")
        return hasattr(cv2, "aruco")
    importlib.import_module(module_name)
    return True


def default_path_exists(path):
    return Path(path).exists()


def load_settings(config_path):
    import yaml

    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_project_path(path, project_root):
    path = Path(path)
    if path.is_absolute():
        return path
    return Path(project_root) / path


def run_preflight(config_path="config/settings.yaml", project_root=None, module_checker=None, path_exists=None):
    config_path = Path(config_path)
    project_root = Path(project_root) if project_root is not None else config_path.resolve().parents[1]
    module_checker = default_module_checker if module_checker is None else module_checker
    path_exists = default_path_exists if path_exists is None else path_exists

    errors = []
    warnings = []
    summary = {}

    try:
        settings = load_settings(config_path)
    except Exception as exc:
        return {
            "ok": False,
            "errors": [f"settings parse failed: {exc}"],
            "warnings": warnings,
            "summary": summary,
        }

    vision = settings.get("vision_tracking") or {}
    pixhawk = settings.get("pixhawk_comm") or {}
    marker_type = str(vision.get("marker_type", "")).lower()
    if not camera_to_body_axes_are_safe(vision):
        errors.append("unsafe_camera_to_body_axis_mapping")
    required_modules = REQUIRED_MODULES_BY_MARKER.get(marker_type, ["yaml"])
    for module_name in required_modules:
        try:
            available = bool(module_checker(module_name))
        except Exception:
            available = False
        if not available:
            errors.append(f"missing module: {module_name}")

    calibration_file = vision.get("calibration_file")
    calibration_path = ""
    if calibration_file:
        calibration_path = resolve_project_path(calibration_file, project_root)
        if not path_exists(calibration_path):
            errors.append(f"missing calibration file: {calibration_file}")
    else:
        errors.append("missing calibration file setting: vision_tracking.calibration_file")

    camera_device = vision.get("device")
    if camera_device and not path_exists(camera_device):
        errors.append(f"missing camera device: {camera_device}")
    elif not camera_device:
        warnings.append("vision_tracking.device is empty")

    pixhawk_device = pixhawk.get("device")
    if pixhawk_device and not path_exists(pixhawk_device):
        errors.append(f"missing pixhawk device: {pixhawk_device}")
    elif not pixhawk_device:
        warnings.append("pixhawk_comm.device is empty")

    rc_override = vision.get("rc_override") or {}
    summary.update(
        {
            "marker_type": marker_type,
            "camera_device": camera_device,
            "pixhawk_device": pixhawk_device,
            "calibration_file": str(calibration_path or calibration_file or ""),
            "enable_motion": bool(vision.get("enable_motion", False)),
            "output_backend": vision.get("output_backend"),
            "required_mode": vision.get("required_mode"),
            "rc_channels": dict(rc_override.get("channels") or {}),
        }
    )

    return {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "summary": summary,
    }


def format_report(report):
    lines = []
    lines.append(f"preflight_ok: {str(report['ok']).lower()}")
    for key, value in report.get("summary", {}).items():
        if isinstance(value, dict):
            value = json.dumps(value, ensure_ascii=False, sort_keys=True)
        lines.append(f"{key}: {value}")
    for warning in report.get("warnings", []):
        lines.append(f"WARNING: {warning}")
    for error in report.get("errors", []):
        lines.append(f"ERROR: {error}")
    return "\n".join(lines)


def parse_args():
    parser = argparse.ArgumentParser(description="Read-only runtime preflight check for ROV visual tracking.")
    parser.add_argument("--config", default="config/settings.yaml")
    parser.add_argument("--project-root", default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    report = run_preflight(config_path=args.config, project_root=args.project_root)
    print(format_report(report))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
