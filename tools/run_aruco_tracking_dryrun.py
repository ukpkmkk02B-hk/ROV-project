import argparse
import time
from pathlib import Path

from modules.controller.visual_tracking_controller import VisualTrackingController
from modules.perception.marker_tracker import ArucoMarkerTracker
from modules.perception.tracking_dryrun_logger import TrackingDryRunLogger
from modules.state.state_estimator import ConstantVelocityEKF


def format_control_direction(state, command):
    return (
        f"state x={state.get('x', 0.0):.3f}, y={state.get('y', 0.0):.3f}, "
        f"z={state.get('z', 0.0):.3f}, yaw={state.get('yaw', 0.0):.2f} deg | "
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
    if pose is None:
        return False

    max_abs_position_m = float(config.get("max_abs_position_m", 5.0))
    max_abs_yaw_deg = float(config.get("max_abs_yaw_deg", 180.0))
    try:
        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        yaw = float(pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return False

    return (
        abs(x) <= max_abs_position_m
        and abs(y) <= max_abs_position_m
        and 0.0 < z <= max_abs_position_m
        and abs(yaw) <= max_abs_yaw_deg
    )


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
    )


def run_dryrun(config_path, log_path, duration_s=None, print_interval_s=0.5):
    settings = load_settings(config_path)
    vision_config = settings["vision_tracking"]
    if vision_config.get("marker_type", "aruco").lower() != "aruco":
        raise ValueError("This dry-run tool only supports vision_tracking.marker_type: aruco")

    log_path = resolve_log_path(vision_config, log_path)
    tracker = ArucoMarkerTracker(vision_config)
    estimator = ConstantVelocityEKF(max_lost_frames=vision_config.get("max_lost_frames", 10))
    controller = build_controller(vision_config)

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
                if pose is not None and is_plausible_pose(pose, vision_config):
                    state = estimator.update(pose, pose.get("timestamp", now))
                else:
                    pose = None
                    state = estimator.predict(now)

                command = controller.compute_command(state) if state.get("status") != "lost" else controller.neutral_command()
                pre_dock_ready = controller.is_pre_dock_ready(state) if state.get("status") != "lost" else False
                logger.log_sample(
                    pose=pose,
                    filtered_state=state,
                    control_cmd=command,
                    pre_dock_ready=pre_dock_ready,
                    timestamp=now,
                )

                if now - last_print >= print_interval_s:
                    print(format_control_direction(state, command) + f" | pre_dock_ready={pre_dock_ready}")
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
    return parser.parse_args()


def main():
    args = parse_args()
    run_dryrun(
        config_path=Path(args.config),
        log_path=args.log,
        duration_s=args.duration,
        print_interval_s=args.print_interval,
    )


if __name__ == "__main__":
    main()
