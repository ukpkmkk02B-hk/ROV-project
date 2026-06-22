import socket
import threading
import json

class SurfaceComm:
    def __init__(self, cfg):
        host = cfg.get("host", "0.0.0.0")
        port = cfg.get("port", 9002)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(9)
        print(f"[SurfaceComm] 🚪 Listening on {host}:{port}")

        self.send_lock = threading.Lock()  # 新增线程锁
        self.client_socket = None
        self.client_address = None
        self.running = True

        self.handler = None  # 回调函数由主程序注册
        self.listen_thread = threading.Thread(target=self._accept_and_listen, daemon=True)
        self.listen_thread.start()

    def register_command_handler(self, handler_func):
        self.handler = handler_func

    def _accept_and_listen(self):
        while self.running:
            try:
                print("[SurfaceComm] 🕐 Waiting for connection...")
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"[SurfaceComm] ✅ Connected to {self.client_address}")

                while self.running:
                    data = self.client_socket.recv(1024)
                    if not data:
                        print("[SurfaceComm] ⚠️ Client disconnected")
                        break

                    try:
                        msg = data.decode('utf-8').strip()
                        print(f"[SurfaceComm] 📥 Received: {msg}")
                        if self.handler:
                            self.handler(msg)
                        else:
                            print("[SurfaceComm] ❗ No handler registered.")
                    except Exception as e:
                        print(f"[SurfaceComm] ❌ Message error: {e}")

            except Exception as e:
                print(f"[SurfaceComm] ❌ Socket error: {e}")

    def send_status(self, status: dict):
        self.send_json({"type": "status", "data": status})

    def send_json(self, data: dict):
        with self.send_lock:
            if self.client_socket:
                try:
                    type_prefix = b'\x02'  # 1字节JSON类型标识
                    json_str = json.dumps(data)
                    json_data = json_str.encode('utf-8')
                    self.client_socket.sendall(type_prefix + json_data + b'\n')
                    # print(f"[SurfaceComm] 📤 Sent JSON: {json_str.strip()}")
                except Exception as e:
                    print(f"[SurfaceComm] ❌ JSON send error: {e}")

    def send_video_packet(self, data: bytes,main_type: int, sub_type: int = 0):
        with self.send_lock:  # 加锁保护
            if self.client_socket:
                try:
                    prefix = main_type.to_bytes(1, 'big') + sub_type.to_bytes(1, 'big')
                    size_prefix = len(data).to_bytes(4, 'big')
                    self.client_socket.sendall(prefix + size_prefix + data)
                    # print(f"[SurfaceComm] Sent packet type {main_type} sub-type {sub_type} size {len(data)}")
                except Exception as e:
                    print(f"[SurfaceComm] Send failed: {e}")

    def stop(self):
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except:
                pass
        if self.server_socket:
            self.server_socket.close()

