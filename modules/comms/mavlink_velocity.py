from modules.controller.motion_command import motion_command_from_mapping

try:
    from pymavlink import mavutil as _default_mavutil
except ImportError:  # pragma: no cover - Linux target should provide pymavlink.
    _default_mavutil = None


def _require_mavutil(mavutil_module=None):
    mavutil_module = mavutil_module or _default_mavutil
    if mavutil_module is None:
        raise RuntimeError("pymavlink is required to send MAVLink velocity commands")
    return mavutil_module


def build_velocity_type_mask(mavlink):
    """Ignore position, acceleration, and yaw; keep velocity and yaw_rate."""
    return (
        mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | getattr(mavlink, "POSITION_TARGET_TYPEMASK_YAW_IGNORE", 0)
    )


def send_body_velocity_command(
    master,
    target_system,
    target_component,
    command,
    mavutil_module=None,
    time_boot_ms=0,
):
    """Send a body-frame velocity command to ArduSub/Pixhawk.

    MotionCommand uses body axes: forward/right/up. MAVLink BODY_NED uses
    forward/right/down, so up is inverted into vz.
    """
    mavutil_module = _require_mavutil(mavutil_module)
    mavlink = mavutil_module.mavlink
    motion = motion_command_from_mapping(command)
    mav_velocity = motion.as_mavlink_body_ned()
    frame = getattr(mavlink, "MAV_FRAME_BODY_NED", mavlink.MAV_FRAME_LOCAL_NED)

    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        target_system,
        target_component,
        frame,
        build_velocity_type_mask(mavlink),
        0,
        0,
        0,
        mav_velocity["vx"],
        mav_velocity["vy"],
        mav_velocity["vz"],
        0,
        0,
        0,
        0,
        mav_velocity["yaw_rate"],
    )
    return mav_velocity
