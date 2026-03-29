#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import os
import sys
import threading
import select
import datetime
from collections import deque

# === 配置 ===
TARGET_IP = '127.0.0.1' 
TARGET_PORT = 9527

# === 界面美化 ===
class C:
    RES = '\033[0m'
    RED = '\033[31m'   # 故障/拒绝
    GRN = '\033[32m'   # 正常/接受
    YEL = '\033[33m'   # 警告/发送中
    BLU = '\033[36m'   # 状态值
    WHT = '\033[37m'
    CYN = '\033[96m'   # 标题
    BOLD = '\033[1m'
    CLR_LINE = '\033[K' # 清除行尾

class UltimateClient:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.running = True
        self.refresh_rate = 1.0 # 界面刷新频率
        
        # --- 数据缓存 ---
        self.uav_pos = [0.0, 0.0, 0.0]
        self.uav_yaw = 0.0
        self.gimbal_pitch = 0.0 # 0x01协议第5个float
        self.battery = 0
        
        # --- 状态位 (对应 SDK 0x0E 的 8 个字节) ---
        self.status = {
            "Overall": 0, 
            "Lidar": 0, 
            "Base": 0,    # Index 2: Base Algo (Modified)
            "Depth": 0,
            "CSI": 0, 
            "Local": 0,   # Index 5: Localization
            "FlightState": 0, 
            "Mapping": 0 
        }
        
        # --- 日志历史 ---
        self.msg_history = deque(maxlen=5)
        self.add_log("System Ready. Waiting for connection...", C.WHT)

    def add_log(self, msg, color=C.WHT):
        t_str = datetime.datetime.now().strftime("%H:%M:%S")
        self.msg_history.append(f" [{t_str}] {color}{msg}{C.RES}")

    def connect(self):
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3)
                print(f"{C.YEL}[INFO] Connecting to {TARGET_IP}:{TARGET_PORT}...{C.RES}")
                self.sock.connect((TARGET_IP, TARGET_PORT))
                self.sock.settimeout(None)
                self.connected = True
                self.add_log("TCP Connected!", C.GRN)
                
                t = threading.Thread(target=self.rx_thread, daemon=True)
                t.start()
                break
            except Exception as e:
                print(f"{C.RED}[ERR] Connect fail: {e}. Retry in 2s...{C.RES}")
                time.sleep(2)

    # --- 接收线程 ---
    def rx_thread(self):
        buffer = b''
        while self.connected and self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    self.connected = False
                    self.add_log("Disconnected from server", C.RED)
                    break
                buffer += data
                
                # 粘包处理
                while len(buffer) >= 5:
                    if buffer[0] != 0xFF or buffer[1] != 0xFE:
                        buffer = buffer[1:]
                        continue
                    
                    data_len = struct.unpack('<H', buffer[2:4])[0]
                    total_len = 4 + data_len
                    if len(buffer) < total_len: break
                    
                    self.parse_packet(buffer[0:total_len])
                    buffer = buffer[total_len:]
            except:
                self.connected = False
                break

    def parse_packet(self, raw_pkt):
        payload = raw_pkt[4:]
        if not payload: return
        cmd = payload[0]
        data = payload[1:]

        # 0x01: 坐标与姿态 (5 floats)
        if cmd == 0x01 and len(data) >= 20:
            vals = struct.unpack('<fffff', data[:20])
            self.uav_pos = vals[:3]
            self.uav_yaw = vals[3]
            self.gimbal_pitch = vals[4]

        # 0x0D: 电量
        elif cmd == 0x0D and len(data) >= 4:
            self.battery = struct.unpack('<i', data[:4])[0]

        # 0x0E: 系统状态 (8 bytes)
        elif cmd == 0x0E and len(data) >= 8:
            s = struct.unpack('BBBBBBBB', data[:8])
            self.status["Overall"] = s[0]
            self.status["Lidar"]   = s[1]
            self.status["Base"]    = s[2] # Base Algo
            self.status["Depth"]   = s[3]
            self.status["CSI"]     = s[4]
            self.status["Local"]   = s[5] # Localization
            self.status["FlightState"] = s[6]
            self.status["Mapping"] = s[7]

        # 0x02: ACK 回执
        elif cmd == 0x02 and len(data) >= 1:
            code = data[0]
            if code == 1: self.add_log("SERVER ACK: OK / ACCEPTED", C.GRN)
            else: self.add_log("SERVER ACK: REJECTED", C.RED)

        # 0x18: 文本日志 (SDK 未实现但预留)
        # 如果你的 SDK 还没加这个，客户端收到也没事，不崩就行

    def send_packet(self, cmd_id, content):
        if not self.sock or not self.connected: return
        try:
            length = 1 + len(content)
            header = b'\xFF\xFE' + struct.pack('<H', length) + struct.pack('B', cmd_id)
            self.sock.sendall(header + content)
            self.add_log(f"CLIENT: Sent Cmd 0x{cmd_id:02X}", C.YEL)
        except:
            self.connected = False

    # --- 仪表盘绘制 ---
    def print_dashboard(self):
        sys.stdout.write("\033[H") # 光标回 Home，不清屏，防闪烁
        
        def yn(val): return f"{C.GRN}[ OK ]{C.RES}" if val==1 else f"{C.RED}[FAIL]{C.RES}"
        def onoff(val): return f"{C.GRN}[ ON ]{C.RES}" if val==1 else f"{C.WHT}[OFF ]{C.RES}"
        
        # 飞行模式解析 (0=Static, 1=Manual, 2=Auto)
        fs_map = {0: "STATIC/IDLE", 1: "MANUAL/POSCTL", 2: "AUTO/OFFBOARD"}
        mode_str = fs_map.get(self.status['FlightState'], "UNKNOWN")
        
        # 电量颜色
        bat_col = C.GRN if self.battery > 30 else C.RED

        print(f"{C.BOLD}{C.CYN}=== ULTIMATE UAV TEST PANEL (SDK V5.7) ==={C.RES}{C.CLR_LINE}")
        
        # 1. 核心状态检查 (Gatekeepers)
        print(f"\n{C.WHT}>>> FLIGHT PRE-CHECKS <<<{C.RES}{C.CLR_LINE}")
        print(f" 1. Base Algo (0x13) : {onoff(self.status['Base'])}  <-- Must be ON to fly{C.CLR_LINE}")
        print(f" 2. Localization     : {yn(self.status['Local'])}  <-- Needs Rosbag/Sim{C.CLR_LINE}")
        print(f" 3. Battery (>30%)   : {bat_col}{self.battery}%{C.RES}      <-- Use '9' to spoof{C.CLR_LINE}")
        print(f" 4. Overall Status   : {yn(self.status['Overall'])}{C.CLR_LINE}")

        # 2. 遥测数据
        print(f"\n{C.WHT}>>> TELEMETRY & SENSORS <<<{C.RES}{C.CLR_LINE}")
        print(f" Pos (m)  : X={self.uav_pos[0]:5.2f}  Y={self.uav_pos[1]:5.2f}  Z={self.uav_pos[2]:5.2f}{C.CLR_LINE}")
        print(f" Attitude : Yaw={self.uav_yaw:5.1f}°  GimbalP={self.gimbal_pitch:5.1f}°{C.CLR_LINE}")
        print(f" Mode     : {C.BLU}{mode_str}{C.RES}{C.CLR_LINE}")
        print(f" Sensors  : Lidar:{yn(self.status['Lidar'])} | Depth:{yn(self.status['Depth'])} | CSI:{yn(self.status['CSI'])}{C.CLR_LINE}")
        print(f" Modules  : Mapping:{onoff(self.status['Mapping'])}{C.CLR_LINE}")

        # 3. 日志区
        print(f"\n{C.WHT}>>> MESSAGE LOG (Last 5) <<<{C.RES}{C.CLR_LINE}")
        logs = list(self.msg_history)
        while len(logs) < 5: logs.append("")
        for log in logs: print(f"{log}{C.CLR_LINE}")

        # 4. 指令菜单 (涵盖所有 SDK 指令)
        print(f"-"*60 + f"{C.CLR_LINE}")
        print(f"{C.YEL}COMMANDS:{C.RES}{C.CLR_LINE}")
        print(f" [1] FlyTo (Patrol Test)      [2] Upload Waypoints (List){C.CLR_LINE}")
        print(f" [3] Start Base (0x13)        [4] Resume Mission (0x14){C.CLR_LINE}")
        print(f" [5] RTH (0x0F)               [6] Emergency Land (0x11){C.CLR_LINE}")
        print(f" [7] Reboot (0x15)            [8] Color Map (0x12){C.CLR_LINE}")
        print(f" [9] Bat Spoof=60% (0x16)     [0] Bat Spoof=OFF{C.CLR_LINE}")
        print(f" {C.WHT}--- Manual Control ---{C.RES}{C.CLR_LINE}")
        print(f" [w/s] Fwd/Back  [a/d] L/R    [q/e] Up/Down  [z/c] Yaw L/R{C.CLR_LINE}")
        print(f" {C.WHT}--- Payload ---{C.RES}{C.CLR_LINE}")
        print(f" [i/k] Gimbal P  [u/j] Gimbal Y  [n/m] Zoom{C.CLR_LINE}")
        print(f"-"*60 + f"{C.CLR_LINE}")
        
        sys.stdout.write("\033[J") # 清除剩余屏幕
        sys.stdout.write(f"{C.CYN}Input > {C.RES}")
        sys.stdout.flush()

    def run(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        self.connect()
        while self.running:
            self.print_dashboard()
            try:
                i, o, e = select.select([sys.stdin], [], [], self.refresh_rate)
                if i:
                    cmd = sys.stdin.readline().strip()
                    self.handle_input(cmd)
            except: pass

    # --- 核心：指令映射表 ---
    def handle_input(self, key):
        if key == 'x' or key == 'quit':
            self.running = False
            sys.exit(0)

        # === 任务类 ===
        elif key == '1': # 0x03 FlyTo
            # 模拟飞到 x=10, y=0, z=2, yaw=0, pitch=0, zoom=1, time=5
            payload = struct.pack('<fffffff', 10.0, 0.0, 2.0, 0.0, 0.0, 1.0, 5.0)
            self.send_packet(0x03, payload)
        
        elif key == '2': # 0x02 Waypoints
            # 模拟 3 个点
            wps = [
                (5.0, 0.0, 2.0, 0.0, -30.0, 1.0, 3.0),
                (5.0, 5.0, 2.0, 90.0, -45.0, 1.0, 3.0),
                (0.0, 0.0, 2.0, 180.0, 0.0, 1.0, 3.0)
            ]
            payload = struct.pack('<H', len(wps)) # Count
            for wp in wps:
                payload += struct.pack('<fffffff', *wp)
            self.send_packet(0x02, payload)

        elif key == '3': self.send_packet(0x13, b'') # Start Base
        elif key == '4': self.send_packet(0x14, b'') # Resume
        elif key == '5': self.send_packet(0x0F, b'') # RTH
        elif key == '6': self.send_packet(0x11, b'') # Emergency
        elif key == '7': self.send_packet(0x15, b'') # Reboot
        elif key == '8': self.send_packet(0x12, b'') # Color Map

        # === 调试类 ===
        elif key == '9': self.send_packet(0x16, struct.pack('<i', 60)) # Bat=60%
        elif key == '0': self.send_packet(0x16, struct.pack('<i', -1)) # Bat=Real

        # === 手动控制 (0x04-0x09) ===
        # 步进值: Pos=1.0m, Yaw=15deg
        elif key == 'w': self.send_packet(0x04, struct.pack('<f', 1.0))
        elif key == 's': self.send_packet(0x05, struct.pack('<f', 1.0))
        elif key == 'a': self.send_packet(0x06, struct.pack('<f', 1.0))
        elif key == 'd': self.send_packet(0x07, struct.pack('<f', 1.0))
        elif key == 'q': self.send_packet(0x09, struct.pack('<f', 0.5))  # Up
        elif key == 'e': self.send_packet(0x09, struct.pack('<f', -0.5)) # Down
        elif key == 'z': self.send_packet(0x08, struct.pack('<f', -15.0)) # Yaw Left
        elif key == 'c': self.send_packet(0x08, struct.pack('<f', 15.0))  # Yaw Right

        # === 负载控制 (0x0A, 0x0B, 0x0C, 0x10) ===
        elif key == 'i': self.send_packet(0x0B, struct.pack('<f', 5.0))   # Gimbal Pitch Up
        elif key == 'k': self.send_packet(0x0B, struct.pack('<f', -5.0))  # Gimbal Pitch Down
        elif key == 'u': self.send_packet(0x0A, struct.pack('<f', -5.0))  # Gimbal Yaw Left
        elif key == 'j': self.send_packet(0x0A, struct.pack('<f', 5.0))   # Gimbal Yaw Right
        elif key == 'n': self.send_packet(0x10, struct.pack('<f', 5.0))   # Zoom 5x
        elif key == 'm': self.send_packet(0x10, struct.pack('<f', 1.0))   # Zoom 1x

if __name__ == '__main__':
    client = UltimateClient()
    try:
        client.run()
    except KeyboardInterrupt:
        pass
