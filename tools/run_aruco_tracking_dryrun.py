import argparse
import time
from pathlib import Path

from modules.controller.motion_command import camera_state_to_body_error, motion_command_from_mapping
from modules.controller.rc_override_mapper import RcOverrideMapper
from modules.controller.visual_tracking_controller import VisualTrackingController
from modules.perception.marker_tracker import ArucoMarkerTracker, validate_pose_quality
from modules.perception.tracking_dryrun_logger import TrackingDryRunLogger
from modules.state.state_estimator import ConstantVelocityEKF


def _latest_pose_age_s(diagnostics, now):
    timestamp = diagnostics.get("tracker_last_valid_pose_timestamp", "")
    if timestamp in (None, ""):
        return ""
    try:
        return max(0.0, now - float(timestamp))
    except (TypeError, ValueError):
        return ""


def _as_int(value):
    if value in (None, ""):
        return 0
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def _as_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def update_pre_dock_observation_state(
    current_count,
    last_tracker_valid_pose_frames,
    diagnostics,
    latest_pose_age_s,
    config,
    has_valid_observation,
):
    max_age_s = float(config.get("pre_dock_recent_observation_max_age_s", 0.5))
    latest_age = _as_float(latest_pose_age_s)
    has_recent = bool(has_valid_observation) or (latest_age is not None and latest_age <= max_age_s)

    tracker_counter_available = diagnostics.get("tracker_valid_pose_frames") not in (None, "")
    tracker_valid_pose_frames = _as_int(diagnostics.get("tracker_valid_pose_frames"))
    if tracker_counter_available:
        delta = max(0, tracker_valid_pose_frames - int(last_tracker_valid_pose_frames))
        current_count += delta
        last_tracker_valid_pose_frames = tracker_valid_pose_frames
    elif has_valid_observation:
        current_count += 1

    if not has_recent:
        current_count = 0

    return current_count, last_tracker_valid_pose_frames, has_recent


def format_control_direction(state, command, pose=None, new_pose=False, latest_pose_age_s=""):
    pose = pose or {}
    latest_age = "" if latest_pose_age_s in (None, "") else f"{float(latest_pose_age_s):.3f}s"
    return (
        f"status={state.get('status', 'unknown')}, lost={state.get('lost_frames', '')}, "
        f"new_pose={bool(new_pose)}, pose_valid={pose.get('pose_valid', False)}, "
        f"reject={pose.get('reject_reason', '')}, latest_pose_age={latest_age} | "
        f"state x={state.get('x', 0.0):.3f}, y={state.get('y', 0.0):.3f}, "
        f"z={state.get('z', 0.0):.3f}, yaw={state.get('yaw', 0.0):.2f} deg | "
        f"motion forward={command.get('forward_m_s', command.get('vx', 0.0)):+.3f}, "
        f"right={command.get('right_m_s', command.get('vy', 0.0)):+.3f}, "
        f"up={command.get('up_m_s', command.get('vz', 0.0)):+.3f}, "
        f"cmd vx={command.get('vx', 0.0):+.3f}, vy={command.get('vy', 0.0):+.3f}, "
        f"vz={command.get('vz', 0.0):+.3f}, yaw_rate={command.get('yaw_rate', 0.0):+.3f} rad/s"
    )


def load_settings(path):
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_log_path(vision_config, cli_log_path):
    return Path(cli_log_path or vision_config.get("dryrun_log_path", "logs/aruco_tracking_dryrun.csv"))


def is_plausible_pose(pose, config):
    return validate_pose_quality(pose, config)


def build_controller(config):
    return VisualTrackingController(
        max_v_m_s=config.get("max_v_m_s", 0.4),
        max_yaw_rate_deg_s=config.get("max_yaw_rate_deg_s", 25.0),
        kp_lateral=config.get("kp_lateral", 0.4),
        kp_vertical=config.get("kp_vertical", 0.3),
        kp_distance=config.get("kp_distance", 0.4),
        kp_yaw=config.get("kp_yaw", 1.0),
        pre_dock_position_tolerance_m=config.get("pre_dock_position_tolerance_m", 0.05),
        pre_dock_distance_tolerance_m=config.get("pre_dock_distance_tolerance_m", 0.05),
        pre_dock_yaw_tolerance_deg=config.get("pre_dock_yaw_tolerance_deg", 5.0),
        camera_to_body=config.get("camera_to_body", {}),
        min_pre_dock_valid_frames=config.get("min_pre_dock_valid_frames", 3),
        pre_dock_recent_observation_max_age_s=config.get("pre_dock_recent_observation_max_age_s", 0.5),
        control_mode=config.get("control_mode", "p"),
        pid_config=config.get("pid", {}),
        control_deadband_m=config.get("control_deadband_m", 0.0),
        yaw_deadband_deg=config.get("yaw_deadband_deg", 0.0),
        command_smoothing_alpha=config.get("command_smoothing_alpha", 1.0),
    )


def compute_dryrun_command(controller, state):
    if state.get("status") == "lost":
        controller.reset()
        return controller.neutral_command()
    return controller.compute_command(state)


DEFAULT_DRYRUN_RC_CHANNELS = {
    "forward": "ch5",
    "right": "ch6",
    "up": "ch3",
    "yaw": "ch4",
}


def build_rc_dryrun_mapper(config):
    """Build a dry-run RC mapper without enabling real RC motion output."""
    rc_config = dict((config or {}).get("rc_override", {}) or {})
    rc_config["enabled"] = True
    if not rc_config.get("channels"):
        rc_config["channels"] = dict(DEFAULT_DRYRUN_RC_CHANNELS)
    return RcOverrideMapper(rc_config)


def draw_preview_overlay(frame, state, rc_override, pre_dock_ready, cv2_module):
    lines = [
        f"status={state.get('status', 'unknown')} lost={state.get('lost_frames', '')} "
        f"pre_dock={bool(pre_dock_ready)}",
        "rc "
        f"ch3={rc_override.get('ch3', '')} ch4={rc_override.get('ch4', '')} "
        f"ch5={rc_override.get('ch5', '')} ch6={rc_override.get('ch6', '')}",
        "press q to quit",
    ]
    height = frame.shape[0]
    for idx, line in enumerate(lines):
        y = max(24, height - 72 + idx * 24)
        cv2_module.putText(
            frame,
            line,
            (16, y),
            cv2_module.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2_module.LINE_AA,
        )
    return frame


def show_preview_frame(tracker, state, rc_override, pre_dock_ready, preview_scale=1.0, cv2_module=None):
    if cv2_module is None:
        import cv2

        cv2_module = cv2
    frame = tracker.get_annotated_frame()
    if frame is None:
        return False

    frame = draw_preview_overlay(frame, state, rc_override, pre_dock_ready, cv2_module)
    scale = float(preview_scale)
    if scale <= 0.0:
        scale = 1.0
    if scale != 1.0:
        frame = cv2_module.resize(frame, None, fx=scale, fy=scale, interpolation=cv2_module.INTER_AREA)

    cv2_module.imshow("ArUco tracking dry-run", frame)
    key = cv2_module.waitKey(1) & 0xFF
    return key == ord("q")


def run_dryrun(
    config_path,
    log_path,
    duration_s=None,
    print_interval_s=0.5,
    device_override=None,
    yaw_offset_override=None,
    preview=False,
    preview_scale=1.0,
    preview_fps=10.0,
    detection_scale=1.0,
):
    settings = load_settings(config_path)
    vision_config = dict(settings["vision_tracking"])
    if vision_config.get("marker_type", "aruco").lower() != "aruco":
        raise ValueError("This dry-run tool only supports vision_tracking.marker_type: aruco")
    if device_override:
        vision_config["device"] = device_override
    if yaw_offset_override is not None:
        camera_to_body = dict(vision_config.get("camera_to_body", {}))
        camera_to_body["yaw_offset_deg"] = float(yaw_offset_override)
        vision_config["camera_to_body"] = camera_to_body
    vision_config["detection_scale"] = float(detection_scale)
    vision_config["enable_preview_annotations"] = bool(preview)

    log_path = resolve_log_path(vision_config, log_path)
    tracker = ArucoMarkerTracker(vision_config)
    estimator = ConstantVelocityEKF(max_lost_frames=vision_config.get("max_lost_frames", 10))
    controller = build_controller(vision_config)
    rc_mapper = build_rc_dryrun_mapper(vision_config)
    output_backend = vision_config.get("output_backend", "mavlink_velocity")
    valid_observation_count = 0
    pre_dock_valid_frame_count = 0
    last_tracker_valid_pose_frames = 0

    start_time = time.time()
    last_print = 0.0
    last_preview = 0.0
    preview_interval_s = 1.0 / float(preview_fps) if float(preview_fps) > 0.0 else 0.0
    tracker.start()
    try:
        with TrackingDryRunLogger(log_path) as logger:
            while True:
                now = time.time()
                if duration_s is not None and now - start_time >= duration_s:
                    break

                pose = tracker.get_pose()
                diagnostics = tracker.get_diagnostics()
                new_pose = pose is not None
                latest_pose_age = _latest_pose_age_s(diagnostics, now)
                if pose is not None and validate_pose_quality(pose, vision_config):
                    valid_observation_count += 1
                    state = estimator.update(pose, pose.get("timestamp", now))
                    has_valid_observation = True
                else:
                    state = estimator.predict(now)
                    has_valid_observation = False
                pre_dock_valid_frame_count, last_tracker_valid_pose_frames, has_recent_valid_observation = (
                    update_pre_dock_observation_state(
                        current_count=pre_dock_valid_frame_count,
                        last_tracker_valid_pose_frames=last_tracker_valid_pose_frames,
                        diagnostics=diagnostics,
                        latest_pose_age_s=latest_pose_age,
                        config=vision_config,
                        has_valid_observation=has_valid_observation,
                    )
                )
                if not has_recent_valid_observation:
                    valid_observation_count = 0
                state["has_valid_observation"] = has_valid_observation
                state["valid_observation_count"] = valid_observation_count
                state["has_recent_valid_observation"] = has_recent_valid_observation
                state["latest_pose_age_s"] = latest_pose_age
                state["pre_dock_valid_frame_count"] = pre_dock_valid_frame_count
                state["pre_dock_recent_observation_max_age_s"] = float(
                    vision_config.get("pre_dock_recent_observation_max_age_s", 0.5)
                )
                state.update(camera_state_to_body_error(state, vision_config))

                command = compute_dryrun_command(controller, state)
                state.update(controller.pre_dock_diagnostics(state))
                pre_dock_ready = controller.is_pre_dock_ready(state)
                motion = motion_command_from_mapping(command)
                mavlink_command = motion.as_mavlink_body_ned()
                rc_override = rc_mapper.map_motion_command(motion)
                logger.log_sample(
                    pose=pose,
                    filtered_state=state,
                    control_cmd=command,
                    pre_dock_ready=pre_dock_ready,
                    diagnostics=diagnostics,
                    output_backend=output_backend,
                    mavlink_command=mavlink_command,
                    rc_override=rc_override,
                    new_pose=new_pose,
                    latest_pose_age_s=latest_pose_age,
                    timestamp=now,
                )

                if now - last_print >= print_interval_s:
                    print(
                        format_control_direction(
                            state,
                            command,
                            pose=pose,
                            new_pose=new_pose,
                            latest_pose_age_s=latest_pose_age,
                        )
                        + f" | pre_dock_ready={pre_dock_ready}"
                    )
                    last_print = now

                if preview and now - last_preview >= preview_interval_s:
                    if show_preview_frame(
                        tracker,
                        state,
                        rc_override,
                        pre_dock_ready,
                        preview_scale=preview_scale,
                    ):
                        break
                    last_preview = now

                time.sleep(0.02)
    finally:
        tracker.stop()
        if preview:
            import cv2

            try:
                cv2.destroyWindow("ArUco tracking dry-run")
            except cv2.error:
                pass


def parse_args():
    parser = argparse.ArgumentParser(description="Run ArUco visual tracking dry-run without Pixhawk motion.")
    parser.add_argument("--config", default="config/settings.yaml", help="Path to settings.yaml")
    parser.add_argument("--log", default=None, help="CSV log output path")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds")
    parser.add_argument("--print-interval", type=float, default=0.5, help="Console print interval in seconds")
    parser.add_argument("--device", default=None, help="Optional camera device override, for example /dev/video0")
    parser.add_argument("--yaw-offset", type=float, default=None, help="Temporary yaw offset in degrees for dry-run")
    parser.add_argument("--preview", action="store_true", help="Show annotated ArUco camera preview window")
    parser.add_argument("--preview-scale", type=float, default=1.0, help="Scale factor for preview window, for example 0.5")
    parser.add_argument("--preview-fps", type=float, default=10.0, help="Maximum preview refresh rate")
    parser.add_argument(
        "--detection-scale",
        type=float,
        default=1.0,
        help="Scale input frames before ArUco detection, for example 0.5",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    run_dryrun(
        config_path=Path(args.config),
        log_path=args.log,
        duration_s=args.duration,
        print_interval_s=args.print_interval,
        device_override=args.device,
        yaw_offset_override=args.yaw_offset,
        preview=args.preview,
        preview_scale=args.preview_scale,
        preview_fps=args.preview_fps,
        detection_scale=args.detection_scale,
    )


if __name__ == "__main__":
    main()
