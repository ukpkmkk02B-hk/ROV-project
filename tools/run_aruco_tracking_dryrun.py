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
        desired_z_m=config.get("desired_z_m", 0.8),
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
    )


def run_dryrun(
    config_path,
    log_path,
    duration_s=None,
    print_interval_s=0.5,
    device_override=None,
    desired_z_override=None,
    yaw_offset_override=None,
):
    settings = load_settings(config_path)
    vision_config = dict(settings["vision_tracking"])
    if vision_config.get("marker_type", "aruco").lower() != "aruco":
        raise ValueError("This dry-run tool only supports vision_tracking.marker_type: aruco")
    if device_override:
        vision_config["device"] = device_override
    if desired_z_override is not None:
        vision_config["desired_z_m"] = float(desired_z_override)
    if yaw_offset_override is not None:
        camera_to_body = dict(vision_config.get("camera_to_body", {}))
        camera_to_body["yaw_offset_deg"] = float(yaw_offset_override)
        vision_config["camera_to_body"] = camera_to_body

    log_path = resolve_log_path(vision_config, log_path)
    tracker = ArucoMarkerTracker(vision_config)
    estimator = ConstantVelocityEKF(max_lost_frames=vision_config.get("max_lost_frames", 10))
    controller = build_controller(vision_config)
    rc_mapper = RcOverrideMapper(vision_config.get("rc_override", {}))
    output_backend = vision_config.get("output_backend", "mavlink_velocity")
    valid_observation_count = 0

    start_time = time.time()
    last_print = 0.0
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
                    valid_observation_count = 0
                    state = estimator.predict(now)
                    has_valid_observation = False
                state["has_valid_observation"] = has_valid_observation
                state["valid_observation_count"] = valid_observation_count
                state.update(camera_state_to_body_error(state, vision_config))

                command = controller.compute_command(state) if state.get("status") != "lost" else controller.neutral_command()
                pre_dock_ready = controller.is_pre_dock_ready(state) if state.get("status") != "lost" else False
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

                time.sleep(0.02)
    finally:
        tracker.stop()


def parse_args():
    parser = argparse.ArgumentParser(description="Run ArUco visual tracking dry-run without Pixhawk motion.")
    parser.add_argument("--config", default="config/settings.yaml", help="Path to settings.yaml")
    parser.add_argument("--log", default=None, help="CSV log output path")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds")
    parser.add_argument("--print-interval", type=float, default=0.5, help="Console print interval in seconds")
    parser.add_argument("--device", default=None, help="Optional camera device override, for example /dev/video0")
    parser.add_argument("--desired-z", type=float, default=None, help="Temporary desired distance in meters for dry-run")
    parser.add_argument("--yaw-offset", type=float, default=None, help="Temporary yaw offset in degrees for dry-run")
    return parser.parse_args()


def main():
    args = parse_args()
    run_dryrun(
        config_path=Path(args.config),
        log_path=args.log,
        duration_s=args.duration,
        print_interval_s=args.print_interval,
        device_override=args.device,
        desired_z_override=args.desired_z,
        yaw_offset_override=args.yaw_offset,
    )


if __name__ == "__main__":
    main()
