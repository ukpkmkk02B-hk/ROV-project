import argparse
import json
import mimetypes
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tools.surface_console.config_store import read_console_config, update_console_config
from tools.surface_console.rov_client import RovTcpClient, build_command_payload

STATIC_DIR = Path(__file__).with_name("static")


def make_handler(client, config_path, default_rov_host, default_rov_port):
    class SurfaceConsoleHandler(BaseHTTPRequestHandler):
        server_version = "SurfaceConsole/1.0"

        def log_message(self, fmt, *args):
            print(f"[SurfaceConsole] {self.address_string()} - {fmt % args}")

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/status":
                self._json_response(
                    {
                        "rov": client.snapshot(),
                        "config": _safe_read_config(),
                        "defaults": {"rov_host": default_rov_host, "rov_port": default_rov_port},
                    }
                )
                return
            if path == "/api/config":
                self._json_response({"config": _safe_read_config()})
                return
            if path == "/api/latest-frame.jpg":
                self._serve_latest_frame()
                return
            self._serve_static(path)

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                body = self._read_json_body()
                if path == "/api/connect":
                    host = body.get("host") or default_rov_host
                    port = int(body.get("port") or default_rov_port)
                    client.connect(host, port)
                    self._json_response({"ok": True, "rov": client.snapshot()})
                elif path == "/api/disconnect":
                    client.disconnect()
                    self._json_response({"ok": True, "rov": client.snapshot()})
                elif path == "/api/command":
                    payload = build_command_payload(rov=body.get("rov"), fish=body.get("fish"))
                    client.send_command(payload)
                    self._json_response({"ok": True, "sent": payload})
                elif path == "/api/config":
                    updates = body.get("updates") or {}
                    confirm_motion = bool(body.get("confirm_motion", False))
                    updated = update_console_config(config_path, updates, confirm_motion=confirm_motion)
                    self._json_response({"ok": True, "config": updated, "restart_required": True})
                else:
                    self._json_response({"ok": False, "error": "not found"}, status=404)
            except Exception as exc:
                self._json_response({"ok": False, "error": str(exc)}, status=400)

        def _read_json_body(self):
            length = int(self.headers.get("Content-Length", "0") or "0")
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            return json.loads(raw.decode("utf-8"))

        def _serve_static(self, path):
            if path in ("", "/"):
                path = "/index.html"
            relative = path.lstrip("/")
            if "/" in relative and not relative.startswith("assets/"):
                self._json_response({"ok": False, "error": "not found"}, status=404)
                return
            static_path = (STATIC_DIR / relative).resolve()
            if not str(static_path).startswith(str(STATIC_DIR.resolve())) or not static_path.exists():
                self._json_response({"ok": False, "error": "not found"}, status=404)
                return
            content_type = mimetypes.guess_type(static_path.name)[0] or "application/octet-stream"
            data = static_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _serve_latest_frame(self):
            frame = client.latest_frame()
            if frame is None:
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                raw = b"no video frame"
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        def _json_response(self, data, status=200):
            raw = json.dumps(data).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

    def _safe_read_config():
        try:
            return read_console_config(config_path)
        except Exception as exc:
            return {"error": str(exc)}

    return SurfaceConsoleHandler


def parse_args():
    parser = argparse.ArgumentParser(description="Run the browser-based ROV surface console.")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8080, help="HTTP bind port")
    parser.add_argument("--rov-host", default="127.0.0.1", help="ROV SurfaceComm host")
    parser.add_argument("--rov-port", type=int, default=9002, help="ROV SurfaceComm TCP port")
    parser.add_argument("--config", default="config/settings.yaml", help="Path to settings.yaml")
    return parser.parse_args()


def main():
    args = parse_args()
    client = RovTcpClient()
    config_path = Path(args.config)
    handler = make_handler(client, config_path, args.rov_host, args.rov_port)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"[SurfaceConsole] Open http://{args.host}:{args.port}")
    print(f"[SurfaceConsole] Default ROV target {args.rov_host}:{args.rov_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
