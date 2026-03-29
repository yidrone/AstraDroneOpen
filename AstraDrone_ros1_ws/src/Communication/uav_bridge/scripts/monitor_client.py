#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import os
import sys
import threading
import datetime

# === 配置 ===
TARGET_IP = '127.0.0.1'  # 如果 SDK 在另一台电脑，请修改为那台电脑的 IP
TARGET_PORT = 9527

# === 颜色代码 ===
class C:
    RES = '\033[0m'
    RED = '\033[31m'
    GRN = '\033[32m'
    YEL = '\033[33m'
    BLU = '\033[36m'
    WHT = '\033[37m'

class MonitorClient:
    def __init__(self):
        self.sock = None
        self.connected = False
        
        # 数据缓存
        self.pos = [0.0, 0.0, 0.0]
        self.battery = 0
        self.status = {
            "Overall": 0,
            "Lidar": 0,
            "Comp": 0,
            "Depth": 0,
            "CSI": 0,
            "Local": 0,
            "FlightState": 0,
            "Mapping": 0  # <--- 【新增】初始化建图状态
        }
        
        # 用于存储最近一次航点上传的反馈信息
        self.last_ack_msg = "No Ack Received"
        self.last_ack_color = C.WHT

    def connect(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3)
                print(f"{C.YEL}[INFO] Connecting to {TARGET_IP}:{TARGET_PORT}...{C.RES}")
                self.sock.connect((TARGET_IP, TARGET_PORT))
                self.sock.settimeout(None)
                self.connected = True
                print(f"{C.GRN}[INFO] Connected! Waiting for data stream...{C.RES}")
                break
            except Exception as e:
                print(f"{C.RED}[ERR] Connection failed: {e}. Retrying in 2s...{C.RES}")
                time.sleep(2)

    def run(self):
        self.connect()
        buffer = b''
        
        # 发送心跳包以防服务端断开 (可选)
        threading.Thread(target=self.heartbeat_thread, daemon=True).start()

        while True:
            try:
                data = self.sock.recv(1024)
                if not data:
                    print(f"{C.RED}[ERR] Server closed connection.{C.RES}")
                    self.sock.close()
                    self.connect()
                    buffer = b''
                    continue
                
                buffer += data
                
                # 循环解析包
                while len(buffer) >= 5:
                    # 1. 检查包头 FF FE
                    if buffer[0] != 0xFF or buffer[1] != 0xFE:
                        buffer = buffer[1:] # 滑动窗口寻找包头
                        continue
                    
                    # 2. 获取长度
                    data_len = struct.unpack('<H', buffer[2:4])[0]
                    total_len = 4 + data_len
                    #print(data_len)
                    # 3. 检查缓冲区是否完整
                    if len(buffer) < total_len:
                        break 
                    
                    # 4. 提取 Payload 并解析
                    payload = buffer[4:total_len]
                    self.parse_packet(payload)
                    
                    # 5. 移除已处理数据
                    buffer = buffer[total_len:]
                    
            except KeyboardInterrupt:
                print("\nBye.")
                sys.exit(0)
            except Exception as e:
                print(f"{C.RED}[ERR] {e}{C.RES}")
                self.sock.close()
                self.connect()
                buffer = b''

    def parse_packet(self, payload):
        if not payload: return
        cmd = payload[0]
        data = payload[1:]

        # --- 0x01: 坐标 (3 float) ---
        if cmd == 0x01 and len(data) >= 12:
            self.pos = struct.unpack('<fff', data[:12])
            # 坐标更新频率高，不触发重绘，以免闪烁

        # --- 0x0D: 电量 (1 int) ---
        elif cmd == 0x0D and len(data) >= 4:
            self.battery = struct.unpack('<i', data[:4])[0]

        # --- 0x0E: 系统状态 (【修改】变为 8 个字节) ---
        # 注意：这里我们兼容旧版协议(7字节)和新版协议(8字节)
        elif cmd == 0x0E:
            s = struct.unpack('BBBBBBBB', data[:8])
            self.status["Overall"] = s[0]
            self.status["Lidar"]   = s[1]
            self.status["Comp"]    = s[2]
            self.status["Depth"]   = s[3]
            self.status["CSI"]     = s[4]
            self.status["Local"]   = s[5]
            self.status["FlightState"] = s[6]
            self.status["Mapping"] = s[7]
            self.print_dashboard()
        # --- 0x02: 航点反馈 (1 char) ---
        elif cmd == 0x02 and len(data) >= 1:
            ack_val = data[0] # 取出第1个字节
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            
            if ack_val == 1:
                self.last_ack_msg = f"[{timestamp}] SUCCESS (Ready)"
                self.last_ack_color = C.GRN
            else:
                self.last_ack_msg = f"[{timestamp}] FAILED (Check Status)"
                self.last_ack_color = C.RED
            
            self.print_dashboard()

    def print_dashboard(self):
        # 清屏 (兼容 Linux/Windows)
        os.system('cls' if os.name == 'nt' else 'clear')
        
        now = datetime.datetime.now().strftime("%H:%M:%S")
        
        # 格式化状态显示
        def st_str(key):
            val = self.status[key]
            if val == 1: return f"{C.GRN}NORMAL (1){C.RES}"
            return f"{C.RED}ERROR  (0){C.RES}"

        # 飞行状态解析
        fs_map = {0: "STATIC", 1: "AUTO/OFFBOARD", 2: "MANUAL", 3: "UNKNOWN"}
        fs_val = self.status["FlightState"]
        fs_str = f"{C.BLU}{fs_map.get(fs_val, 'UNK')}{C.RES}"

        print(f"{C.WHT}=========================================={C.RES}")
        print(f"   UAV MONITOR CLIENT      Time: {now}")
        print(f"{C.WHT}=========================================={C.RES}")
        print(f"")
        print(f"   {C.WHT}POSITION (m){C.RES}")
        print(f"     X : {self.pos[0]:.2f}")
        print(f"     Y : {self.pos[1]:.2f}")
        print(f"     Z : {self.pos[2]:.2f}")
        print(f"")
        print(f"   {C.WHT}SYSTEM INFO{C.RES}")
        print(f"     Battery      : {self.battery}%")
        print(f"     Flight Mode  : {fs_str}")
        print(f"")
        print(f"   {C.WHT}SENSOR STATUS{C.RES}")
        print(f"     [1] Lidar    : {st_str('Lidar')}")
        print(f"     [2] Compute  : {st_str('Comp')}")
        print(f"     [3] Depth    : {st_str('Depth')}")
        print(f"     [4] CSI Cam  : {st_str('CSI')}")
        print(f"     [5] Local    : {st_str('Local')}")
        # 【新增】显示建图状态
        print(f"     [6] Mapping  : {st_str('Mapping')}")
        print(f"     -------------------------")
        print(f"     [0] OVERALL  : {st_str('Overall')}")
        print(f"")
        print(f"   {C.WHT}NOTIFICATIONS{C.RES}")
        print(f"     Waypoint ACK : {self.last_ack_color}{self.last_ack_msg}{C.RES}")
        print(f"")
        print(f"{C.WHT}=========================================={C.RES}")
        print(f"Press Ctrl+C to exit.")

    def heartbeat_thread(self):
        import time
        while True:
            if self.sock and self.connected:
                try:
                    # 发送 0x00 心跳包 (4字节: FF FE 01 00 00)
                    pkt = b'\xFF\xFE\x01\x00\x00'
                    self.sock.sendall(pkt)
                except:
                    self.connected = False
            time.sleep(1)

if __name__ == '__main__':
    client = MonitorClient()
    client.run()

