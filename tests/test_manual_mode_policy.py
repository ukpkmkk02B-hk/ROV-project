import unittest

from modules.controller.manual_modes import mode_command_result, normalize_supported_mode


class ManualModePolicyTests(unittest.TestCase):
    def test_manual_is_the_only_supported_flight_mode_command(self):
        self.assertEqual(
            mode_command_result("MANUAL"),
            {"is_mode_command": True, "accepted": True, "mode": "MANUAL", "reason": ""},
        )

        self.assertEqual(
            mode_command_result("STABILIZE"),
            {
                "is_mode_command": True,
                "accepted": False,
                "mode": "STABILIZE",
                "reason": "manual_mode_only",
            },
        )
        self.assertEqual(
            mode_command_result("ALT_HOLD"),
            {
                "is_mode_command": True,
                "accepted": False,
                "mode": "ALT_HOLD",
                "reason": "manual_mode_only",
            },
        )

    def test_non_mode_command_is_left_for_other_handlers(self):
        self.assertEqual(
            mode_command_result("tracking start"),
            {"is_mode_command": False, "accepted": False, "mode": "tracking start", "reason": ""},
        )

    def test_mode_normalization_rejects_non_manual_for_safety_tools(self):
        self.assertEqual(normalize_supported_mode("manual"), "MANUAL")
        with self.assertRaisesRegex(ValueError, "only MANUAL mode is supported"):
            normalize_supported_mode("GUIDED")


if __name__ == "__main__":
    unittest.main()
