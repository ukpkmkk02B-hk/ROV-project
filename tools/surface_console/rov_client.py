import json
import socket
import threading
import time


MAX_BINARY_FRAME_BYTES = 64 * 1024 * 1024
VIDEO_MAIN_TYPE = 0x01
VIDEO_SUB_TYPE = 0x00


def build_command_payload(rov=None, fish=None):
    data = {}
    if rov:
        data["rov"] = rov
    if fish:
        data["fish"] = fish
    if not data:
        raise ValueError("rov or fish command is required")
    return {"type": "command", "data": data}


def parse_surface_stream(buffer):
    messages = []
    video_frames = []
    cursor = 0
    total = len(buffer)
    while cursor < total:
        if buffer[cursor : cursor + 2] == b"\x02{":
            newline = buffer.find(b"\n", cursor + 1)
            if newline < 0:
                break
            try:
                decoded = buffer[cursor + 1 : newline].decode("utf-8")
                messages.append(json.loads(decoded))
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            cursor = newline + 1
            continue

        if cursor + 6 <= total and buffer[cursor] != 0x02:
            main_type = buffer[cursor]
            sub_type = buffer[cursor + 1]
            frame_size = int.from_bytes(buffer[cursor + 2 : cursor + 6], "big")
            frame_end = cursor + 6 + frame_size
            if 0 <= frame_size <= MAX_BINARY_FRAME_BYTES:
                if frame_end > total:
                    break
                if main_type == VIDEO_MAIN_TYPE and sub_type == VIDEO_SUB_TYPE:
                    video_frames.append(buffer[cursor + 6 : frame_end])
                cursor = frame_end
                continue

        newline = buffer.find(b"\n", cursor)
        if newline < 0:
            break
        line = buffer[cursor:newline]
        json_start = line.find(b"{")
        if json_start >= 0:
            try:
                decoded = line[json_start:].decode("utf-8")
                messages.append(json.loads(decoded))
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
        cursor = newline + 1

    leftover = buffer[cursor:]
    if len(leftover) > 1024 * 1024:
        leftover = leftover[-4096:]
    return messages, video_frames, leftover


def parse_prefixed_json_messages(buffer):
    messages, _video_frames, leftover = parse_surface_stream(buffer)
    return messages, leftover


class RovTcpClient:
    def __init__(self):
        self._socket = None
        self._reader = None
        self._lock = threading.RLock()
        self._buffer = b""
        self._connected = False
        self._host = ""
        self._port = 0
        self._last_status = {}
        self._last_message_time = None
        self._last_error = ""
        self._latest_frame = None
        self._latest_frame_time = None
        self._latest_frame_size = 0

    def connect(self, host, port, timeout=3.0):
        with self._lock:
            self.disconnect()
            sock = socket.create_connection((host, int(port)), timeout=timeout)
            sock.settimeout(0.5)
            self._socket = sock
            self._host = host
            self._port = int(port)
            self._connected = True
            self._last_error = ""
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

    def disconnect(self):
        sock = self._socket
        self._connected = False
        self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def send_command(self, payload):
        raw = json.dumps(payload).encode("utf-8") + b"\n"
        with self._lock:
            if not self._socket or not self._connected:
                raise RuntimeError("ROV TCP client is not connected")
            self._socket.sendall(raw)

    def send_rov_command(self, command):
        self.send_command(build_command_payload(rov=command))

    def send_fish_command(self, command):
        self.send_command(build_command_payload(fish=command))

    def snapshot(self):
        with self._lock:
            return {
                "connected": self._connected,
                "host": self._host,
                "port": self._port,
                "last_status": self._last_status,
                "last_message_time": self._last_message_time,
                "last_error": self._last_error,
                "video": {
                    "has_frame": self._latest_frame is not None,
                    "latest_frame_time": self._latest_frame_time,
                    "latest_frame_size": self._latest_frame_size,
                },
            }

    def latest_frame(self):
        with self._lock:
            return self._latest_frame

    def latest_frame_snapshot(self):
        with self._lock:
            return {
                "frame": self._latest_frame,
                "latest_frame_time": self._latest_frame_time,
                "latest_frame_size": self._latest_frame_size,
            }

    def handle_received_bytes(self, data):
        now = time.time()
        with self._lock:
            messages, video_frames, self._buffer = parse_surface_stream(self._buffer + data)
            for frame in video_frames:
                self._latest_frame = bytes(frame)
                self._latest_frame_time = now
                self._latest_frame_size = len(frame)
            for message in messages:
                if message.get("type") == "status":
                    self._last_status = message.get("data", {})
                    self._last_message_time = now

    def _read_loop(self):
        while True:
            with self._lock:
                sock = self._socket
                connected = self._connected
            if not sock or not connected:
                return
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    self._mark_disconnected("remote disconnected")
                    return
                self.handle_received_bytes(chunk)
            except socket.timeout:
                continue
            except OSError as exc:
                self._mark_disconnected(str(exc))
                return

    def _mark_disconnected(self, error):
        with self._lock:
            self._connected = False
            self._last_error = error
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
