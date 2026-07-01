import json
import unittest

from tools.surface_console.rov_client import build_command_payload, parse_prefixed_json_messages


class SurfaceConsoleClientTests(unittest.TestCase):
    def test_build_command_payload_wraps_rov_command_for_surface_comm(self):
        payload = build_command_payload(rov="docking start")

        self.assertEqual(payload, {"type": "command", "data": {"rov": "docking start"}})

    def test_build_command_payload_wraps_runtime_tracking_command(self):
        payload = build_command_payload(rov="tracking mode hold_captured_ch3")

        self.assertEqual(payload, {"type": "command", "data": {"rov": "tracking mode hold_captured_ch3"}})

    def test_build_command_payload_can_include_fish_command(self):
        payload = build_command_payload(fish="float")

        self.assertEqual(payload, {"type": "command", "data": {"fish": "float"}})

    def test_parse_prefixed_json_messages_accepts_surface_status_prefix(self):
        raw = b'\x02{"type":"status","data":{"task_status":{"system_state":"system_idle"}}}\n'

        messages, leftover = parse_prefixed_json_messages(raw)

        self.assertEqual(leftover, b"")
        self.assertEqual(messages[0]["type"], "status")
        self.assertEqual(messages[0]["data"]["task_status"]["system_state"], "system_idle")

    def test_parse_prefixed_json_messages_ignores_non_json_chunks(self):
        status = {"type": "status", "data": {"ok": True}}
        raw = b"\x01binary-chunk\n\x02" + json.dumps(status).encode("utf-8") + b"\npartial"

        messages, leftover = parse_prefixed_json_messages(raw)

        self.assertEqual(messages, [status])
        self.assertEqual(leftover, b"partial")

    def test_parse_prefixed_json_messages_skips_binary_video_frames(self):
        fake_status_in_video_payload = b'{"type":"status","data":{"fake":true}}\n'
        video_frame = (
            b"\x01\x00"
            + len(fake_status_in_video_payload).to_bytes(4, "big")
            + fake_status_in_video_payload
        )
        real_status = {"type": "status", "data": {"task_status": {"system_state": "system_idle"}}}
        raw = video_frame + b"\x02" + json.dumps(real_status).encode("utf-8") + b"\n"

        messages, leftover = parse_prefixed_json_messages(raw)

        self.assertEqual(leftover, b"")
        self.assertEqual(messages, [real_status])

    def test_parse_prefixed_json_messages_keeps_partial_video_frame(self):
        payload = b"abcdef"
        partial_video_frame = b"\x01\x00" + len(payload).to_bytes(4, "big") + payload[:2]

        messages, leftover = parse_prefixed_json_messages(partial_video_frame)

        self.assertEqual(messages, [])
        self.assertEqual(leftover, partial_video_frame)


if __name__ == "__main__":
    unittest.main()
