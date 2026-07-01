import tempfile
import threading
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path

from tools.surface_console.app import make_handler


class FakeClient:
    def __init__(self, frame=None):
        self.frame = frame

    def snapshot(self):
        return {"connected": False, "video": {"has_frame": self.frame is not None}}

    def latest_frame(self):
        return self.frame


class SurfaceConsoleAppTests(unittest.TestCase):
    def test_latest_frame_endpoint_returns_cached_jpeg(self):
        response, body = self._request_latest_frame(FakeClient(frame=b"\xff\xd8jpeg\xff\xd9"))

        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Content-Type"), "image/jpeg")
        self.assertEqual(body, b"\xff\xd8jpeg\xff\xd9")

    def test_latest_frame_endpoint_returns_404_without_frame(self):
        response, body = self._request_latest_frame(FakeClient(frame=None))

        self.assertEqual(response.status, 404)
        self.assertIn(b"no video frame", body)

    def _request_latest_frame(self, client):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "settings.yaml"
            config_path.write_text("vision_tracking:\n  desired_z_m: 0.5\n", encoding="utf-8")
            handler = make_handler(client, config_path, "127.0.0.1", 9002)
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                conn = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                conn.request("GET", "/api/latest-frame.jpg")
                response = conn.getresponse()
                body = response.read()
                conn.close()
                return response, body
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
