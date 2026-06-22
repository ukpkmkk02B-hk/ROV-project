import socket
import time
import struct
import json
import threading


HOST = '127.0.0.1'
PORT = 9002

def send_command(sock, msg: str):
    """发送字符串或JSON命令"""
    if not msg.strip():
        return
    try:
        # 如果输入看起来是 JSON，就原样发
        if msg.strip().startswith("{"):
            sock.sendall((msg.strip() + "\n").encode('utf-8'))
        else:
            # 普通字符串命令
            sock.sendall(msg.encode('utf-8'))
        print(f"[Client] 📤 已发送: {msg.strip()}")
    except Exception as e:
        print(f"[Client] ❌ 发送失败: {e}")

def input_thread(sock):
    """专门处理用户输入"""
    while True:
        try:
            user_cmd = input("请输入指令 > ")  # 输入提示
            if user_cmd.lower() in ("exit", "quit"):
                print("[Client] 🛑 退出输入线程")
                sock.close()
                break
            send_command(sock, user_cmd)
        except Exception as e:
            print(f"[Client] ❌ 输入线程错误: {e}")
            break

def main():
    try:
        with socket.create_connection((HOST, PORT), timeout=5) as sock:
            print(f"[Client] ✅ 已连接至 {HOST}:{PORT}，进入长连接模式")
            
            # 启动输入线程
            # threading.Thread(target=input_thread, args=(sock,), daemon=True).start()

            buffer = b""  # 接收缓冲区
            expected_video_size = None  # 视频帧期望长度
            # cmd = "start_docking"  # 示例指令
            # sock.sendall(cmd.encode('utf-8'))
            # print(f"[Client] 📤 已发送: {cmd.strip()}")
            while True:
                # 1. 发送指令（示例）
                
                
                
                # 2. 接收数据（增大缓冲区）
                chunk = sock.recv(8192)
                if not chunk: 
                    print("[Client] ⚠️ 连接已关闭")
                    break
                buffer += chunk
                
                # 3. 协议解析状态机（核心改进）
                while len(buffer) > 0:
                    if expected_video_size is not None:
                        # 处理剩余数据
                        if len(buffer) >= expected_video_size:
                            video_data = buffer[:expected_video_size]
                            buffer = buffer[expected_video_size:]
                            expected_video_size = None
                            # 处理视频帧，sub_type等信息如何使用可以加到视频帧处理逻辑
                        else:
                            break
                    else:
                        # 还没开始接收一帧数据
                        if len(buffer) < 6:  # 至少2字节头 + 4字节长度
                            break
                        main_type = buffer[0]
                        sub_type = buffer[1]
                        if main_type == 0x01:
                            if sub_type == 0x01:
                                print("[Client] 🖼 JPEG 帧处理中...")
                                expected_video_size = struct.unpack(">I", buffer[2:6])[0]
                                if expected_video_size > 10 * 1024 * 1024:
                                    print(f"[Client] 🛑 异常帧大小: {expected_video_size}字节，断开连接")
                                    return
                                buffer = buffer[6:]
                                print(f"[Client] 🚩 检测到视频帧头，类型: {main_type} 子类型: {sub_type} 预期长度: {expected_video_size}字节")
                        elif main_type == 0x02:
                            # JSON包处理还是保持旧逻辑：包头1字节 + JSON + \n
                            newline_index = buffer.find(b'\n', 1)
                            if newline_index == -1:
                                break
                            json_bytes = buffer[1:newline_index]
                            buffer = buffer[newline_index + 1:]
                            try:
                                json_str = json_bytes.decode('utf-8')
                                parsed = json.loads(json_str)
                                # print(f"[Client] 📥 收到JSON: {json.dumps(parsed, indent=2)[:200]}...")
                            except Exception as e:
                                print(f"[Client] ❌ JSON解析失败: {e}\n原始数据: {json_bytes[:20]}...")
                        else:
                            print(f"[Client] ❓ 未知协议头: 0x{main_type:02X}，丢弃首字节")
                            buffer = buffer[1:]
                time.sleep(0.01)  # 降低CPU占用
    
    except socket.timeout:
        print("[Client] ⏱️ 连接超时")
    except ConnectionResetError:
        print("[Client] 🔌 连接被服务端重置")
    except Exception as e:
        print(f"[Client] ❌ 错误: {e}")
    finally:
        print("[Client] 🛑 连接关闭")

if __name__ == "__main__":
    main()