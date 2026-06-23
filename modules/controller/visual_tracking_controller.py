import math


def _clamp(value, low, high):
    return max(low, min(high, value))


class VisualTrackingController:
    """PD-style visual tracking controller with safe velocity limits."""

    def __init__(
        self,
        desired_z_m=0.8,
        max_v_m_s=0.4,
        max_yaw_rate_deg_s=25.0,
        kp_lateral=0.4,
        kp_vertical=0.3,
        kp_distance=0.4,
        kp_yaw=1.0,
        pre_dock_position_tolerance_m=0.05,
        pre_dock_distance_tolerance_m=0.05,
        pre_dock_yaw_tolerance_deg=5.0,
    ):
        self.desired_z_m = float(desired_z_m)
        self.max_v_m_s = float(max_v_m_s)
        self.max_yaw_rate_rad_s = math.radians(float(max_yaw_rate_deg_s))
        self.kp_lateral = float(kp_lateral)
        self.kp_vertical = float(kp_vertical)
        self.kp_distance = float(kp_distance)
        self.kp_yaw = float(kp_yaw)
        self.pre_dock_position_tolerance_m = float(pre_dock_position_tolerance_m)
        self.pre_dock_distance_tolerance_m = float(pre_dock_distance_tolerance_m)
        self.pre_dock_yaw_tolerance_deg = float(pre_dock_yaw_tolerance_deg)

    def compute_command(self, state):
        distance_error = float(state["z"]) - self.desired_z_m
        lateral_error = float(state["x"])
        vertical_error = float(state.get("y", 0.0))
        yaw_error_deg = float(state.get("yaw", 0.0))

        vx = _clamp(self.kp_distance * distance_error, -self.max_v_m_s, self.max_v_m_s)
        vy = _clamp(-self.kp_lateral * lateral_error, -self.max_v_m_s, self.max_v_m_s)
        vz = _clamp(-self.kp_vertical * vertical_error, -self.max_v_m_s, self.max_v_m_s)
        yaw_rate = _clamp(
            -self.kp_yaw * math.radians(yaw_error_deg),
            -self.max_yaw_rate_rad_s,
            self.max_yaw_rate_rad_s,
        )

        return {
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "yaw_rate": yaw_rate,
            "v_yaw": yaw_rate,
        }

    def neutral_command(self):
        return {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0, "v_yaw": 0.0}

    def is_pre_dock_ready(self, state):
        return (
            abs(float(state["x"])) <= self.pre_dock_position_tolerance_m
            and abs(float(state.get("y", 0.0))) <= self.pre_dock_position_tolerance_m
            and abs(float(state["z"]) - self.desired_z_m) <= self.pre_dock_distance_tolerance_m
            and abs(float(state.get("yaw", 0.0))) <= self.pre_dock_yaw_tolerance_deg
        )
