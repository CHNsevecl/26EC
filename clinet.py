#!/usr/bin/env python3
import zmq
import asyncio
import websockets
import base64
import signal
import sys
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os
import json
from datetime import datetime
import cv2
import numpy as np

# ZMQ配置
context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect("tcp://localhost:5555")

connected_clients = set()

# 获取脚本所在目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDING_DIR = os.path.join(SCRIPT_DIR, "recordings")

# 创建录制目录
os.makedirs(RECORDING_DIR, exist_ok=True)

class VideoRecorder:
    def __init__(self):
        self.is_recording = False
        self.frames = []
        self.filename = ""
        self.frame_count = 0
        self.width = 0
        self.height = 0
        self.fps = 30
        
    def start_recording(self):
        if self.is_recording:
            return False
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"recording_{timestamp}.mp4"
        self.frame_count = 0
        self.frames = []
        self.is_recording = True
        self.width = 0
        self.height = 0
        print(f"🎥 开始录制MP4: {self.filename}")
        return True
        
    def add_frame(self, jpeg_data):
        """添加一帧图像（JPEG -> 解码 -> 存储）"""
        if not self.is_recording:
            return
        
        # 解码JPEG为BGR（OpenCV格式）
        nparr = np.frombuffer(jpeg_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is not None:
            # 记录第一帧的尺寸
            if self.width == 0:
                self.height, self.width = frame.shape[:2]
                print(f"📐 视频尺寸: {self.width}x{self.height}")
            self.frames.append(frame)
            self.frame_count += 1
            
    def stop_recording(self):
        if not self.is_recording:
            return None
        
        self.is_recording = False
        
        if not self.frames or self.width == 0:
            print("⚠️ 没有录制到任何帧")
            return None
        
        # 保存为MP4
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mp4_filename = f"recording_{timestamp}.mp4"
        mp4_path = os.path.join(RECORDING_DIR, mp4_filename)
        
        try:
            # 创建VideoWriter
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(mp4_path, fourcc, self.fps, (self.width, self.height))
            
            # 写入所有帧
            for frame in self.frames:
                writer.write(frame)
            writer.release()
            
            file_size = os.path.getsize(mp4_path) / (1024 * 1024)
            print(f"✅ 录制完成: {mp4_filename} ({self.frame_count} 帧, {file_size:.2f} MB)")
            
            self.frames = []
            return mp4_filename
            
        except Exception as e:
            print(f"❌ 保存MP4失败: {e}")
            if os.path.exists(mp4_path):
                os.remove(mp4_path)
            return None
    
    def get_status(self):
        return {
            'is_recording': self.is_recording,
            'frame_count': self.frame_count,
            'filename': self.filename if self.is_recording else None
        }

recorder = VideoRecorder()

class CustomHandler(SimpleHTTPRequestHandler):
    """自定义HTTP处理器"""
    def do_GET(self):
        print(f"📥 GET请求: {self.path}")
        
        if self.path == '/':
            self.path = '/web.html'
        elif self.path.startswith('/download'):
            self.handle_download()
            return
        elif self.path == '/status':
            self.handle_status()
            return
        return super().do_GET()
    
    def handle_download(self):
        """处理视频下载请求"""
        try:
            # 解析URL参数
            parsed = self.path.split('?')
            if len(parsed) > 1:
                params = parsed[1].split('=')
                if len(params) == 2 and params[0] == 'file':
                    filename = params[1]
                    print(f"📥 请求下载文件: {filename}")
                    
                    # 安全检查
                    import re
                    if not re.match(r'^[\w\-\.]+$', filename):
                        self.send_error(400, "Invalid filename")
                        return
                    
                    # 构建完整路径
                    filepath = os.path.join(RECORDING_DIR, filename)
                    print(f"📁 查找文件: {filepath}")
                    
                    # 检查文件是否存在
                    if os.path.exists(filepath):
                        file_size = os.path.getsize(filepath)
                        print(f"✅ 找到文件: {filename} ({file_size} bytes)")
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'video/mp4')
                        self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
                        self.send_header('Content-Length', str(file_size))
                        self.end_headers()
                        
                        with open(filepath, 'rb') as f:
                            self.wfile.write(f.read())
                        print(f"✅ 下载完成: {filename}")
                        return
                    else:
                        print(f"❌ 文件不存在: {filepath}")
                        self.send_error(404, f"File not found: {filename}")
                        return
            
            self.send_error(400, "Missing file parameter")
            
        except Exception as e:
            print(f"❌ 下载错误: {e}")
            import traceback
            traceback.print_exc()
            self.send_error(500, f"Download error: {str(e)}")
    
    def handle_status(self):
        """获取录制状态"""
        status = recorder.get_status()
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(status).encode('utf-8'))
    
    def do_POST(self):
        """处理POST请求（录制控制）"""
        if self.path == '/control':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))
                action = data.get('action')
                
                print(f"🎮 控制命令: {action}")
                
                if action == 'start_record':
                    success = recorder.start_recording()
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = json.dumps({
                        'success': success,
                        'message': '录制已开始' if success else '录制已在进行中'
                    })
                    self.wfile.write(response.encode('utf-8'))
                    
                elif action == 'stop_record':
                    filename = recorder.stop_recording()
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    if filename:
                        download_url = f'/download?file={filename}'
                        print(f"✅ 录制完成，下载URL: {download_url}")
                        response = json.dumps({
                            'success': True,
                            'message': f'录制完成: {filename}',
                            'filename': filename,
                            'download_url': download_url
                        })
                    else:
                        response = json.dumps({
                            'success': False,
                            'message': '没有录制到任何帧'
                        })
                    self.wfile.write(response.encode('utf-8'))
                    
                elif action == 'list_recordings':
                    recordings = []
                    if os.path.exists(RECORDING_DIR):
                        for f in os.listdir(RECORDING_DIR):
                            if f.startswith('recording_') and f.endswith('.mp4'):
                                filepath = os.path.join(RECORDING_DIR, f)
                                size = os.path.getsize(filepath) / (1024 * 1024)
                                recordings.append({
                                    'name': f,
                                    'size': f'{size:.2f} MB',
                                    'date': datetime.fromtimestamp(os.path.getctime(filepath)).strftime('%Y-%m-%d %H:%M:%S')
                                })
                    recordings.sort(key=lambda x: x['name'], reverse=True)
                    
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    response = json.dumps({'recordings': recordings})
                    self.wfile.write(response.encode('utf-8'))
                    
                else:
                    self.send_response(400)
                    self.end_headers()
                    
            except Exception as e:
                print(f"❌ POST处理错误: {e}")
                import traceback
                traceback.print_exc()
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = json.dumps({'success': False, 'message': str(e)})
                self.wfile.write(response.encode('utf-8'))

def start_http_server(port=8080):
    """启动HTTP服务器"""
    os.chdir(SCRIPT_DIR)
    handler = CustomHandler
    httpd = HTTPServer(('0.0.0.0', port), handler)
    print(f"🌐 HTTP服务器启动在 http://0.0.0.0:{port}")
    print(f"📁 HTML文件目录: {SCRIPT_DIR}")
    print(f"📁 录制文件保存在: {RECORDING_DIR}")
    httpd.serve_forever()

async def websocket_handler(websocket):
    connected_clients.add(websocket)
    print(f"👤 客户端连接: {len(connected_clients)} 个客户端")
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)
        print(f"👤 客户端断开: {len(connected_clients)} 个客户端")

async def zmq_to_websocket():
    print("📷 等待ZMQ图像数据...")
    frame_count = 0
    last_log_time = time.time()
    
    while True:
        try:
            message = socket.recv(flags=zmq.NOBLOCK)
            if message:
                frame_count += 1
                
                current_time = time.time()
                if current_time - last_log_time >= 1.0:
                    status = recorder.get_status()
                    if status['is_recording']:
                        print(f"📊 接收帧率: {frame_count} FPS | 录制帧数: {status['frame_count']}")
                    else:
                        print(f"📊 接收帧率: {frame_count} FPS")
                    frame_count = 0
                    last_log_time = current_time
                
                # 录制MP4
                if recorder.is_recording:
                    recorder.add_frame(message)
                
                # 转发到WebSocket
                img_base64 = base64.b64encode(message).decode('utf-8')
                if connected_clients:
                    await asyncio.gather(*[
                        client.send(img_base64) 
                        for client in connected_clients
                    ], return_exceptions=True)
                    
        except zmq.Again:
            await asyncio.sleep(0.001)
        except Exception as e:
            print(f"❌ 错误: {e}")
            await asyncio.sleep(0.01)

async def main():
    ws_server = await websockets.serve(
        websocket_handler, 
        "0.0.0.0", 
        8765,
        max_size=10**7
    )
    print("🔌 WebSocket服务器启动在 ws://0.0.0.0:8765")
    
    http_thread = threading.Thread(target=start_http_server, args=(8080,), daemon=True)
    http_thread.start()
    
    print("\n" + "="*50)
    print("🎯 系统已启动！")
    print(f"   在浏览器中输入: http://树莓派IP:8080")
    print(f"   录制文件保存在: {RECORDING_DIR}")
    print("   按 Ctrl+C 停止服务")
    print("="*50 + "\n")
    
    await zmq_to_websocket()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⏹️ 正在关闭服务...")
        sys.exit(0)