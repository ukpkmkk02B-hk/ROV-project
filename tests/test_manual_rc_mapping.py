import unittest

from modules.controller.manual_rc import apply_manual_rov_command


class ManualRcMappingTests(unittest.TestCase):
    def test_left_and_right_match_current_vehicle_direction(self):
        rc_state = {f"ch{i}": 1500 for i in range(1, 9)}

        self.assertTrue(apply_manual_rov_command(rc_state, "left"))
        self.assertEqual(rc_state["ch3"], 1500)
        self.assertEqual(rc_state["ch4"], 1500)
        self.assertEqual(rc_state["ch5"], 1500)
        self.assertEqual(rc_state["ch6"], 1600)

        self.assertTrue(apply_manual_rov_command(rc_state, "right"))
        self.assertEqual(rc_state["ch3"], 1500)
        self.assertEqual(rc_state["ch4"], 1500)
        self.assertEqual(rc_state["ch5"], 1500)
        self.assertEqual(rc_state["ch6"], 1400)

    def test_up_and_down_match_current_vehicle_direction(self):
        rc_state = {f"ch{i}": 1500 for i in range(1, 9)}

        self.assertTrue(apply_manual_rov_command(rc_state, "up"))
        self.assertEqual(rc_state["ch3"], 1400)
        self.assertEqual(rc_state["ch4"], 1500)
        self.assertEqual(rc_state["ch5"], 1500)
        self.assertEqual(rc_state["ch6"], 1500)

        self.assertTrue(apply_manual_rov_command(rc_state, "down"))
        self.assertEqual(rc_state["ch3"], 1700)
        self.assertEqual(rc_state["ch4"], 1500)
        self.assertEqual(rc_state["ch5"], 1500)
        self.assertEqual(rc_state["ch6"], 1500)

    def test_unhandled_command_does_not_change_rc_state(self):
        rc_state = {"ch3": 1400, "ch4": 1520, "ch5": 1600, "ch6": 1490}

        self.assertFalse(apply_manual_rov_command(rc_state, "tracking start"))
        self.assertEqual(rc_state, {"ch3": 1400, "ch4": 1520, "ch5": 1600, "ch6": 1490})


if __name__ == "__main__":
    unittest.main()
