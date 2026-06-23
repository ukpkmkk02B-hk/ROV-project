import math
import unittest

from modules.controller.motion_command import (
    MotionCommand,
    camera_state_to_body_error,
    motion_command_from_mapping,
    normalize_angle_deg,
)
from modules.controller.rc_override_mapper import RcOverrideMapper


class MotionCommandTests(unittest.TestCase):
    def test_camera_state_to_body_error_uses_default_mapping(self):
        body = camera_state_to_body_error(
            {"x": 0.2, "y": -0.1, "z": 1.2, "yaw": 15.0, "status": "tracking"},
            {},
        )

        self.assertEqual(body["forward_m"], 1.2)
        self.assertEqual(body["right_m"], 0.2)
        self.assertEqual(body["up_m"], 0.1)
        self.assertEqual(body["yaw_error_deg"], 15.0)
        self.assertEqual(body["status"], "tracking")

    def test_motion_command_serializes_body_and_legacy_velocity_fields(self):
        command = MotionCommand(forward_m_s=0.1, right_m_s=-0.2, up_m_s=0.3, yaw_rate_rad_s=-0.4)

        data = command.as_dict()

        self.assertEqual(data["forward_m_s"], 0.1)
        self.assertEqual(data["right_m_s"], -0.2)
        self.assertEqual(data["up_m_s"], 0.3)
        self.assertEqual(data["yaw_rate_rad_s"], -0.4)
        self.assertEqual(data["vx"], 0.1)
        self.assertEqual(data["vy"], -0.2)
        self.assertEqual(data["vz"], 0.3)
        self.assertEqual(data["yaw_rate"], -0.4)

    def test_motion_command_from_legacy_mapping_treats_vz_as_up(self):
        command = motion_command_from_mapping({"vx": 0.1, "vy": 0.2, "vz": -0.3, "v_yaw": 0.4})

        self.assertEqual(command.forward_m_s, 0.1)
        self.assertEqual(command.right_m_s, 0.2)
        self.assertEqual(command.up_m_s, -0.3)
        self.assertEqual(command.yaw_rate_rad_s, 0.4)

    def test_camera_state_to_body_error_applies_yaw_offset_and_normalization(self):
        body = camera_state_to_body_error(
            {"x": 0.0, "y": 0.0, "z": 0.2, "yaw": 170.0},
            {"camera_to_body": {"yaw_offset_deg": -180.0, "yaw_normalize": True}},
        )

        self.assertEqual(body["yaw_raw_deg"], 170.0)
        self.assertEqual(body["yaw_error_deg"], -10.0)

    def test_normalize_angle_deg_wraps_to_signed_range(self):
        self.assertEqual(normalize_angle_deg(190.0), -170.0)
        self.assertEqual(normalize_angle_deg(-190.0), 170.0)


class RcOverrideMapperTests(unittest.TestCase):
    def test_disabled_mapper_returns_blank_dryrun_channels(self):
        mapper = RcOverrideMapper({"enabled": False})

        self.assertEqual(mapper.map_motion_command(MotionCommand.neutral()), {})

    def test_mapper_requires_channel_mapping_for_motion_output(self):
        mapper = RcOverrideMapper({"enabled": True})

        with self.assertRaises(ValueError):
            mapper.validate_for_motion()

    def test_mapper_converts_motion_to_configured_rc_channels(self):
        mapper = RcOverrideMapper(
            {
                "enabled": True,
                "channels": {
                    "forward": "ch5",
                    "right": "ch6",
                    "up": "ch3",
                    "yaw": "ch4",
                },
                "neutral_pwm": 1500,
                "min_pwm": 1400,
                "max_pwm": 1600,
                "pwm_per_m_s": 200,
                "pwm_per_rad_s": 100,
            }
        )

        channels = mapper.map_motion_command(
            MotionCommand(forward_m_s=0.25, right_m_s=-0.25, up_m_s=1.0, yaw_rate_rad_s=math.pi)
        )

        self.assertEqual(channels["ch5"], 1550)
        self.assertEqual(channels["ch6"], 1450)
        self.assertEqual(channels["ch3"], 1600)
        self.assertEqual(channels["ch4"], 1600)
        self.assertEqual(channels["ch1"], 1500)


if __name__ == "__main__":
    unittest.main()
