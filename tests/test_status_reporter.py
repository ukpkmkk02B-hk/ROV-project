import unittest

from modules.telemetry.status_reporter import StatusReporter


class FakeScheduler:
    def get_system_status(self):
        return {
            "system_state": "task_running",
            "current_task": {
                "name": "docking",
                "pre_dock_ready": True,
            },
        }


class FakePixhawk:
    current_mode = "STABILIZE"

    def get_attitude(self):
        return {"roll": 1.0, "pitch": 2.0, "yaw": 3.0}

    def get_velocity(self):
        return {"vx": 0.1, "vy": 0.2, "vz": -0.1}

    def get_servo_outputs(self):
        return {"ch3": 1500, "ch5": 1520}

    def is_armed(self):
        return False


class StatusReporterTests(unittest.TestCase):
    def test_collect_status_includes_scheduler_task_status(self):
        reporter = StatusReporter(surface=None, scheduler=FakeScheduler())

        status = reporter._collect_status()

        self.assertTrue(status["task_status"]["current_task"]["pre_dock_ready"])

    def test_collect_status_includes_pixhawk_inner_loop_diagnostics(self):
        reporter = StatusReporter(surface=None, pixhawk=FakePixhawk())

        status = reporter._collect_status()

        self.assertEqual(status["rov_telemetry"]["flight_mode"], "STABILIZE")
        self.assertFalse(status["rov_telemetry"]["armed"])
        self.assertEqual(status["rov_telemetry"]["attitude"]["yaw"], 3.0)
        self.assertEqual(status["rov_telemetry"]["local_velocity"]["vx"], 0.1)
        self.assertEqual(status["rov_telemetry"]["servo_outputs"]["ch5"], 1520)
        self.assertEqual(status["rov_attitude"]["roll"], 1.0)


if __name__ == "__main__":
    unittest.main()
