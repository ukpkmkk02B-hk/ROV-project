import math


class DockingVerticalController:
    """Docking-only buoyancy compensation and ArUco approach-speed control."""

    NEUTRAL_PWM = 1500
    MIN_PWM = 1500
    MAX_PWM = 1800
    SPEED_FILTER_ALPHA = 0.2
    PWM_SLEW_RATE_PER_S = 100.0
    DEFAULT_DT_S = 0.05

    def __init__(self, config=None):
        config = config or {}
        self.buoyancy_hold_pwm = self._integer_config(
            config,
            "pre_align_buoyancy_hold_pwm",
            1600,
            self.MIN_PWM,
            self.MAX_PWM,
        )
        self.down_pwm_max = self._integer_config(
            config,
            "pre_align_down_pwm_max",
            1700,
            self.MIN_PWM,
            self.MAX_PWM,
        )
        if self.buoyancy_hold_pwm > self.down_pwm_max:
            raise ValueError("pre_align_buoyancy_hold_pwm must be <= pre_align_down_pwm_max")

        self.target_approach_speed_m_s = self._float_config(
            config,
            "pre_align_target_approach_speed_m_s",
            0.03,
            0.0,
            0.2,
        )
        self.approach_speed_kp = self._float_config(
            config,
            "pre_align_approach_speed_kp",
            2000.0,
            0.0,
            5000.0,
        )
        self.approach_speed_tolerance_m_s = self._float_config(
            config,
            "pre_dock_approach_speed_tolerance_m_s",
            0.01,
            0.0,
            0.1,
        )

        self._filtered_approach_speed_m_s = None
        self._last_pwm = self.NEUTRAL_PWM
        self._last_timestamp = None
        self._last_status = self._neutral_status()

    def reset(self, timestamp=None, initial_pwm=None):
        self._filtered_approach_speed_m_s = None
        self._last_pwm = self.NEUTRAL_PWM if initial_pwm is None else int(initial_pwm)
        self._last_timestamp = None if timestamp is None else float(timestamp)
        self._last_status = self._neutral_status()

    def update(self, camera_vz_m_s, desired_up_m_s, vertical_allowed, timestamp):
        try:
            timestamp = float(timestamp)
            camera_vz_m_s = float(camera_vz_m_s or 0.0)
            desired_up_m_s = float(desired_up_m_s or 0.0)
        except (TypeError, ValueError):
            return self._invalid_input_status("invalid_numeric_input")
        if not all(math.isfinite(value) for value in (timestamp, camera_vz_m_s, desired_up_m_s)):
            return self._invalid_input_status("non_finite_input")

        raw_approach_speed = -camera_vz_m_s
        if self._filtered_approach_speed_m_s is None:
            filtered_approach_speed = raw_approach_speed
        else:
            alpha = self.SPEED_FILTER_ALPHA
            filtered_approach_speed = (
                self._filtered_approach_speed_m_s * (1.0 - alpha)
                + raw_approach_speed * alpha
            )
        self._filtered_approach_speed_m_s = filtered_approach_speed

        if vertical_allowed:
            target_approach_speed = self._clamp(
                -desired_up_m_s,
                -self.target_approach_speed_m_s,
                self.target_approach_speed_m_s,
            )
        else:
            target_approach_speed = 0.0

        speed_error = target_approach_speed - filtered_approach_speed
        raw_pwm = self.buoyancy_hold_pwm + self.approach_speed_kp * speed_error
        saturated_low = raw_pwm < self.MIN_PWM
        saturated_high = raw_pwm > self.down_pwm_max
        limited_pwm = self._clamp(raw_pwm, self.MIN_PWM, self.down_pwm_max)

        if self._last_timestamp is None:
            dt = self.DEFAULT_DT_S
        else:
            dt = max(0.0, timestamp - self._last_timestamp)
        max_delta = self.PWM_SLEW_RATE_PER_S * dt
        slewed_pwm = self._move_toward(self._last_pwm, limited_pwm, max_delta)
        final_pwm = int(round(slewed_pwm))
        self._last_pwm = final_pwm
        self._last_timestamp = timestamp

        speed_ready = (
            abs(filtered_approach_speed)
            <= self.approach_speed_tolerance_m_s + 1e-12
        )
        self._last_status = {
            "pre_align_raw_approach_speed_m_s": raw_approach_speed,
            "pre_align_filtered_approach_speed_m_s": filtered_approach_speed,
            "pre_align_target_approach_speed_m_s": target_approach_speed,
            "pre_align_approach_speed_error_m_s": speed_error,
            "pre_align_vertical_pwm_raw": raw_pwm,
            "pre_align_vertical_pwm": final_pwm,
            "pre_align_buoyancy_hold_active": abs(target_approach_speed) <= 1e-12,
            "pre_align_pwm_saturated_low": saturated_low,
            "pre_align_pwm_saturated_high": saturated_high,
            "pre_align_vertical_allowed": bool(vertical_allowed),
            "pre_dock_approach_speed_tolerance_m_s": self.approach_speed_tolerance_m_s,
            "pre_dock_approach_speed_ok": speed_ready,
            "pre_align_input_valid": True,
            "pre_align_invalid_reason": "",
        }
        return dict(self._last_status)

    def status(self):
        return dict(self._last_status)

    def activate_buoyancy_hold(self):
        """Switch diagnostics and output state to fixed post-confirm buoyancy hold."""
        self._last_pwm = self.buoyancy_hold_pwm
        self._last_timestamp = None
        self._last_status.update(
            {
                "pre_align_target_approach_speed_m_s": 0.0,
                "pre_align_approach_speed_error_m_s": 0.0,
                "pre_align_vertical_pwm_raw": float(self.buoyancy_hold_pwm),
                "pre_align_vertical_pwm": self.buoyancy_hold_pwm,
                "pre_align_buoyancy_hold_active": True,
                "pre_align_pwm_saturated_low": False,
                "pre_align_pwm_saturated_high": False,
                "pre_align_vertical_allowed": False,
            }
        )
        return dict(self._last_status)

    def _neutral_status(self):
        return {
            "pre_align_raw_approach_speed_m_s": 0.0,
            "pre_align_filtered_approach_speed_m_s": 0.0,
            "pre_align_target_approach_speed_m_s": 0.0,
            "pre_align_approach_speed_error_m_s": 0.0,
            "pre_align_vertical_pwm_raw": float(self.buoyancy_hold_pwm),
            "pre_align_vertical_pwm": self.NEUTRAL_PWM,
            "pre_align_buoyancy_hold_active": False,
            "pre_align_pwm_saturated_low": False,
            "pre_align_pwm_saturated_high": False,
            "pre_align_vertical_allowed": False,
            "pre_dock_approach_speed_tolerance_m_s": self.approach_speed_tolerance_m_s,
            "pre_dock_approach_speed_ok": True,
            "pre_align_input_valid": True,
            "pre_align_invalid_reason": "",
        }

    def _invalid_input_status(self, reason):
        self._filtered_approach_speed_m_s = None
        self._last_pwm = self.NEUTRAL_PWM
        self._last_timestamp = None
        self._last_status = self._neutral_status()
        self._last_status.update(
            {
                "pre_dock_approach_speed_ok": False,
                "pre_align_input_valid": False,
                "pre_align_invalid_reason": str(reason),
            }
        )
        return dict(self._last_status)

    @staticmethod
    def _move_toward(current, target, max_delta):
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def _integer_config(config, key, default, minimum, maximum):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
        if not minimum <= value <= maximum:
            raise ValueError(f"{key} must be between {minimum} and {maximum}")
        return value

    @staticmethod
    def _float_config(config, key, default, minimum, maximum):
        value = config.get(key, default)
        if isinstance(value, bool):
            raise ValueError(f"{key} must be a number")
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{key} must be a number") from None
        if not minimum <= value <= maximum:
            raise ValueError(f"{key} must be between {minimum} and {maximum}")
        return value
