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

    def latest_frame_snapshot(self):
        return {
            "frame": self.frame,
            "latest_frame_time": 123.0 if self.frame is not None else None,
            "latest_frame_size": len(self.frame) if self.frame is not None else 0,
        }


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

    def test_mjpeg_endpoint_streams_cached_jpeg_frame(self):
        response, body = self._request_latest_frame(
            FakeClient(frame=b"\xff\xd8jpeg\xff\xd9"),
            path="/api/video.mjpg?once=1&fps=25",
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            response.getheader("Content-Type"),
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.assertIn(b"--frame", body)
        self.assertIn(b"Content-Type: image/jpeg", body)
        self.assertIn(b"\xff\xd8jpeg\xff\xd9", body)

    def _request_latest_frame(self, client, path="/api/latest-frame.jpg"):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = Path(tmpdir) / "settings.yaml"
            config_path.write_text("vision_tracking:\n  max_v_m_s: 0.4\n", encoding="utf-8")
            handler = make_handler(client, config_path, "127.0.0.1", 9002)
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                conn = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                conn.request("GET", path)
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
