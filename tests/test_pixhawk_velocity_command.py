import importlib
import sys
import types
import unittest


class FakeMavlinkConstants:
    MAV_FRAME_LOCAL_NED = 1
    MAV_FRAME_BODY_NED = 8
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1 << 0
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 1 << 1
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 1 << 2
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 1 << 3
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 1 << 4
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 1 << 5
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1 << 10
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 1 << 11
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_DATA_STREAM_EXTRA1 = 10
    MAV_DATA_STREAM_POSITION = 11


class FakeMav:
    def __init__(self):
        self.position_target = None
        self.rc_override = None

    def set_position_target_local_ned_send(self, *args):
        self.position_target = args

    def command_long_send(self, *args):
        self.command_long = args

    def rc_channels_override_send(self, *args):
        self.rc_override = args


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()

    def close(self):
        self.closed = True


def import_pixhawk_with_stubs():
    sys.modules.setdefault("yaml", types.SimpleNamespace(safe_load=lambda f: {}))
    pymavlink = types.ModuleType("pymavlink")
    mavutil = types.SimpleNamespace(mavlink=FakeMavlinkConstants)
    pymavlink.mavutil = mavutil
    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = mavutil
    module_name = "modules.comms.pixhawk_comm"
    sys.modules.pop(module_name, None)
    return importlib.import_module(module_name)


class PixhawkVelocityCommandTests(unittest.TestCase):
    def test_send_velocity_command_accepts_body_motion_fields(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = FakeMaster()

        comm.send_velocity_command(
            {"forward_m_s": 0.1, "right_m_s": 0.2, "up_m_s": 0.3, "yaw_rate_rad_s": 0.4}
        )

        args = comm.master.mav.position_target
        self.assertIsNotNone(args)
        self.assertEqual(args[3], FakeMavlinkConstants.MAV_FRAME_BODY_NED)
        self.assertEqual(args[8], 0.1)
        self.assertEqual(args[9], 0.2)
        self.assertEqual(args[10], -0.3)
        self.assertEqual(args[15], 0.4)

        ignore_mask = args[4]
        self.assertTrue(ignore_mask & FakeMavlinkConstants.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
        self.assertFalse(ignore_mask & FakeMavlinkConstants.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

    def test_send_velocity_command_keeps_legacy_v_yaw_field(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = FakeMaster()

        comm.send_velocity_command({"vx": 0.1, "vy": 0.2, "vz": -0.3, "v_yaw": -0.2})

        self.assertEqual(comm.master.mav.position_target[8], 0.1)
        self.assertEqual(comm.master.mav.position_target[9], 0.2)
        self.assertEqual(comm.master.mav.position_target[10], 0.3)
        self.assertEqual(comm.master.mav.position_target[15], -0.2)

    def test_send_rc_override_requires_active_master(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = None

        with self.assertRaisesRegex(RuntimeError, "Pixhawk is not connected"):
            comm.send_rc_override({"ch3": 1500})

    def test_send_rc_override_sends_all_channels_with_defaults(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = FakeMaster()

        comm.send_rc_override({"ch5": 1600})

        self.assertEqual(
            comm.master.mav.rc_override,
            (1, 1, 1500, 1500, 1500, 1500, 1600, 1500, 1500, 1500),
        )

    def test_stop_with_no_master_does_not_raise(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = None

        comm.stop()


if __name__ == "__main__":
    unittest.main()
