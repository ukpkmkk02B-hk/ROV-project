import ast
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class MainRuntimeSafetyTests(unittest.TestCase):
    def test_main_loop_does_not_consume_camera_pose_queue(self):
        tree = ast.parse(Path("main.py").read_text(encoding="utf-8"))
        calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "get_pose"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "camera"
        ]

        self.assertEqual(calls, [])

    def test_main_uses_lazy_imports_for_optional_camera_and_depth_dependencies(self):
        tree = ast.parse(Path("main.py").read_text(encoding="utf-8"))
        top_level_imports = []
        for node in tree.body:
            if isinstance(node, ast.ImportFrom):
                top_level_imports.append(node.module)

        self.assertNotIn("modules.comms.depth_forwarder", top_level_imports)
        self.assertNotIn("modules.perception.camera", top_level_imports)
        self.assertNotIn("modules.perception.marker_tracker", top_level_imports)

    def test_docking_task_does_not_import_optional_hardware_interfaces(self):
        tree = ast.parse(Path("modules/tasks/docking_task.py").read_text(encoding="utf-8"))
        top_level_imports = []
        for node in tree.body:
            if isinstance(node, ast.ImportFrom):
                top_level_imports.append(node.module)

        self.assertNotIn("modules.perception.camera", top_level_imports)
        self.assertNotIn("modules.comms.pixhawk_comm", top_level_imports)
        self.assertNotIn("modules.comms.comm_base", top_level_imports)


class RuntimePreflightTests(unittest.TestCase):
    def _write_settings(self, directory, camera_device="/dev/camera_main", pixhawk_device="/dev/ttl_pixhawk"):
        calib = directory / "calib.yaml"
        calib.write_text("camera_matrix: {}\n", encoding="utf-8")
        settings = directory / "settings.yaml"
        settings.write_text(
            f"""
pixhawk_comm:
  device: "{pixhawk_device}"
  baud: 115200
vision_tracking:
  marker_type: "aruco"
  device: "{camera_device}"
  dictionary: "DICT_4X4_50"
  marker_id: 20
  marker_size_m: 0.04
  calibration_file: "{calib.name}"
  desired_z_m: 0.5
  enable_motion: false
  output_backend: "rc_override"
  required_mode: "STABILIZE"
  rc_override:
    channels:
      forward: "ch5"
      right: "ch6"
      up: "ch3"
      yaw: "ch4"
""".strip()
            + "\n",
            encoding="utf-8",
        )
        return settings, calib

    def _settings_dict(self, calib_name="calib.yaml", camera_device="/dev/camera_main", pixhawk_device="/dev/ttl_pixhawk"):
        return {
            "pixhawk_comm": {"device": pixhawk_device, "baud": 115200},
            "vision_tracking": {
                "marker_type": "aruco",
                "device": camera_device,
                "calibration_file": calib_name,
                "desired_z_m": 0.5,
                "enable_motion": False,
                "output_backend": "rc_override",
                "required_mode": "STABILIZE",
                "rc_override": {
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    }
                },
            },
        }

    def test_preflight_reports_safe_summary_for_valid_aruco_runtime(self):
        from tools.check_runtime_preflight import run_preflight

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            settings, calib = self._write_settings(root)

            with patch("tools.check_runtime_preflight.load_settings", return_value=self._settings_dict()):
                report = run_preflight(
                    config_path=settings,
                    project_root=root,
                    module_checker=lambda name: True,
                    path_exists=lambda path: Path(path) == calib or str(path) in {"/dev/camera_main", "/dev/ttl_pixhawk"},
                )

        self.assertTrue(report["ok"])
        self.assertEqual(report["summary"]["marker_type"], "aruco")
        self.assertFalse(report["summary"]["enable_motion"])
        self.assertEqual(report["summary"]["output_backend"], "rc_override")
        self.assertEqual(report["summary"]["rc_channels"]["forward"], "ch5")

    def test_preflight_reports_missing_device_and_dependency_errors(self):
        from tools.check_runtime_preflight import run_preflight

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            settings, calib = self._write_settings(root)

            with patch("tools.check_runtime_preflight.load_settings", return_value=self._settings_dict()):
                report = run_preflight(
                    config_path=settings,
                    project_root=root,
                    module_checker=lambda name: name != "cv2.aruco",
                    path_exists=lambda path: Path(path) == calib,
                )

        self.assertFalse(report["ok"])
        self.assertIn("missing module: cv2.aruco", report["errors"])
        self.assertIn("missing camera device: /dev/camera_main", report["errors"])
        self.assertIn("missing pixhawk device: /dev/ttl_pixhawk", report["errors"])


if __name__ == "__main__":
    unittest.main()
