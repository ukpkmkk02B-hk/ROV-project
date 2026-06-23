import importlib
import sys
import types
import unittest


class FakeMavlinkConstants:
    MAV_FRAME_LOCAL_NED = 1
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1 << 0
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 1 << 1
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 1 << 2
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 1 << 3
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 1 << 4
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 1 << 5
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_DATA_STREAM_EXTRA1 = 10
    MAV_DATA_STREAM_POSITION = 11


class FakeMav:
    def __init__(self):
        self.position_target = None

    def set_position_target_local_ned_send(self, *args):
        self.position_target = args

    def command_long_send(self, *args):
        self.command_long = args


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
    def test_send_velocity_command_accepts_yaw_rate_field(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = FakeMaster()

        comm.send_velocity_command({"vx": 0.1, "vy": 0.2, "vz": -0.1, "yaw_rate": 0.3})

        args = comm.master.mav.position_target
        self.assertIsNotNone(args)
        self.assertEqual(args[8], 0.1)
        self.assertEqual(args[9], 0.2)
        self.assertEqual(args[10], -0.1)
        self.assertEqual(args[15], 0.3)

    def test_send_velocity_command_keeps_legacy_v_yaw_field(self):
        module = import_pixhawk_with_stubs()
        comm = module.PixhawkComm({"device": "fake", "baud": 115200})
        comm.master = FakeMaster()

        comm.send_velocity_command({"v_yaw": -0.2})

        self.assertEqual(comm.master.mav.position_target[15], -0.2)


if __name__ == "__main__":
    unittest.main()
