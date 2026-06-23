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


class StatusReporterTests(unittest.TestCase):
    def test_collect_status_includes_scheduler_task_status(self):
        reporter = StatusReporter(surface=None, scheduler=FakeScheduler())

        status = reporter._collect_status()

        self.assertTrue(status["task_status"]["current_task"]["pre_dock_ready"])


if __name__ == "__main__":
    unittest.main()
