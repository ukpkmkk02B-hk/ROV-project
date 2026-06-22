import cv2
import threading
import time
from modules.comms.surface_comm import SurfaceComm

class ROVVideoForwarder:
    def __init__(self, surface: SurfaceComm, camera_id=0):
        # 初始化摄像头与通信接口
        self.surface = surface
        self.camera_id = camera_id
        self.running = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        # 启动摄像头采集线程
        self.running = True
        self.thread.start()
        print("[ROVVideoForwarder] 🟢 Started")

    def stop(self):
        # 停止线程
        self.running = False

    def _loop(self):
        # 摄像头采集并转发图像主循环
        cap = cv2.VideoCapture(self.camera_id)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                try:
                    # 图像压缩并转为字节发送
                    success, jpg = cv2.imencode('.jpg', frame, encode_param)
                    if success:
                        self.surface.send_video_packet(jpg.tobytes())
                except Exception as e:
                    print(f"[ROVVideoForwarder] ❌ Send error: {e}")
            time.sleep(0.1)

        cap.release()