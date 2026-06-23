from __future__ import annotations

from control.rc_mapper import rc_command


def compute_errors(measured_u, measured_v, measured_z_m, measured_yaw_deg, cfg: dict) -> dict:
    target = cfg["visual_servo"]
    return {
        "u_px": float(measured_u - target["desired_u"]),
        "v_px": float(measured_v - target["desired_v"]),
        "z_m": float(measured_z_m - target["desired_z_m"]),
        "yaw_deg": float(measured_yaw_deg - target["desired_yaw_deg"]),
    }


def p_control(errors: dict, cfg: dict, safe: bool = False) -> dict:
    gains = cfg["gains"]
    rc = cfg["rc"]
    max_delta = int(rc["max_delta_safe"] if safe else rc["max_delta_initial"])
    neutral = int(rc["neutral"])
    return {
        "lateral": rc_command(neutral, -gains["kx"] * errors["u_px"], max_delta),
        "vertical": rc_command(neutral, -gains["ky"] * errors["v_px"], max_delta),
        "forward": rc_command(neutral, -gains["kz"] * errors["z_m"], max_delta),
        "yaw": rc_command(neutral, -gains["kyaw"] * errors["yaw_deg"], max_delta),
    }
