#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import threading
import time
import sys
import os
import datetime

# ==========================================
#    配置与常量
# ==========================================
TARGET_IP = '127.0.0.1' 
TARGET_PORT = 9527

# --- 协议常量表 ---
CMD_HEARTBEAT   = 0x00 
CMD_WAYPOINTS   = 0x02 # 上传航点
CMD_FLY_TO      = 0x03 # 飞向坐标
CMD_MOVE_FWD    = 0x04 # 前进
CMD_MOVE_BCK    = 0x05 # 后退
CMD_MOVE_RIGHT  = 0x06 # 右移
CMD_MOVE_LEFT   = 0x07 # 左移
CMD_YAW         = 0x08 # 偏航
CMD_UP_DOWN     = 0x09 # 高度
CMD_GIMBAL_YAW  = 0x0A # 云台Yaw
CMD_GIMBAL_PIT  = 0x0B # 云台Pitch
CMD_INTERRUPT   = 0x0C # 中断/悬停
CMD_RESUME      = 0x12 # 恢复任务
CMD_RETURN      = 0x0F # 返航
CMD_LAND        = 0x10 # 降落
CMD_REBOOT      = 0x11 # 重启

# 回传指令
RESP_POS        = 0x01
RESP_BATTERY    = 0x0D
RESP_STATUS     = 0x0E

# 颜色代码
class C:
    RES = '\033[0m'
    RED = '\033[31m'
    GRN = '\033[32m'
    YEL = '\033[33m'
    BLU = '\033[36m'
    GRY = '\033[90m'
    WHT = '\033[37m'
    CYN = '\033[36m'
    MAG = '\033[35m'

class DebugStation:
    def __init__(self):
        # 通信变量
        self.sock = None
        self.running = True
        self.connected = False
        
        # 界面控制: 0=Menu(详细), 1=Monitor(持续输出)
        self.view_mode = 0 
        self.header_printed = False # 用于Monitor模式控制表头
        self.show_hex = False
        self.log_buffer = [] 
        self.max_logs = 8
        
        # 数据缓存
        self.uav_pos = [0.0, 0.0, 0.0]
        self.battery = 0
        
        # 【修改】移除了 FlightState，保留6个核心状态
        self.status = {
            "Overall": 0, "Lidar": 0, "Comp": 0, "Depth": 0,
            "CSI": 0, "Local": 0
        }
        self.last_input_cmd = "None"
        
        # 云台状态
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0

    # ==========================================
    #            UI 渲染核心
    # ==========================================
    def add_log(self, msg, type="INFO"):
        """添加日志到滚动缓冲区 (仅在Menu模式显示详细日志)"""
        t = datetime.datetime.now().strftime("%H:%M:%S")
        if type == "TX": prefix = f"{C.BLU}[TX]{C.RES}"
        elif type == "RX": prefix = f"{C.YEL}[RX]{C.RES}"
        elif type == "ERR": prefix = f"{C.RED}[ERR]{C.RES}"
        else: prefix = f"{C.GRN}[SYS]{C.RES}"
        
        entry = f"{C.GRY}{t}{C.RES} {prefix} {msg}"
        self.log_buffer.append(entry)
        if len(self.log_buffer) > self.max_logs:
            self.log_buffer.pop(0)

    def render_menu(self):
        """模式0: 详细菜单模式 (清屏显示)"""
        output = "\033[2J\033[H" # ANSI清屏
        
        conn_str = f"{C.GRN}CONNECTED{C.RES}" if self.connected else f"{C.RED}DISCONNECTED{C.RES}"
        bat_str = f"{self.battery}%" if self.connected else "--"
        
        output += f"{C.WHT}=========================================={C.RES}\n"
        output += f"   ASTRA DEBUG STATION V6.3 (NoFlightState)|  {conn_str}\n"
        output += f"{C.WHT}=========================================={C.RES}\n"
        
        output += f"  Target IP : {TARGET_IP}\n"
        output += f"  Battery   : {bat_str}\n"
        output += f"  Gimbal    : P={self.gimbal_pitch:.1f} Y={self.gimbal_yaw:.1f}\n"
        output += f"  Last Cmd  : {C.CYN}{self.last_input_cmd}{C.RES}\n\n"
        
        output += f"{C.WHT}  [Flight Controls] (WASD+RF+QE){C.RES}\n"
        output += f"    {C.YEL}W / S{C.RES}   : Forward / Back  (0x04/05)\n"
        output += f"    {C.YEL}A / D{C.RES}   : Left / Right    (0x07/06)\n"
        output += f"    {C.YEL}R / F{C.RES}   : Up / Down       (0x09)\n"
        output += f"    {C.YEL}Q / E{C.RES}   : Yaw Left/Right  (0x08)\n"
        output += f"    {C.YEL}SPACE{C.RES}   : Brake / Hover   (0x0C)\n\n"

        output += f"{C.WHT}  [Gimbal Controls] (IJKL){C.RES}\n"
        output += f"    {C.MAG}I / K{C.RES}   : Pitch Up / Down (0x0B)\n"
        output += f"    {C.MAG}J / L{C.RES}   : Yaw Left / Right(0x0A)\n"
        output += f"    {C.MAG}U{C.RES}       : Center Gimbal\n\n"
        
        output += f"{C.WHT}  [System Functions]{C.RES}\n"
        output += f"    {C.GRN}1{C.RES} : Upload New Waypoints (Test Hover) {C.GRN}2{C.RES} : RTH (Return)\n"
        output += f"    {C.GRN}3{C.RES} : Land (Descend)                    {C.GRN}0{C.RES} : Remote Reboot\n"
        output += f"    {C.GRN}4{C.RES} : Fly To Test Point (3 Float)       {C.GRN}5{C.RES} : Resume Mission (0x12)\n\n"
        
        output += f"{C.WHT}------------------------------------------{C.RES}\n"
        output += f"  > Press {C.CYN}'m'{C.RES} to start CONTINUOUS MONITOR\n"
        output += f"  > Press {C.RED}'9'{C.RES} to Exit\n"
        
        sys.stdout.write(output)
        sys.stdout.flush()

    def render_dashboard(self):
        """模式1: 持续输出模式 (不清屏，Stream Log)"""
        # 1. 打印表头
        if not self.header_printed:
            print(f"\n{C.WHT}--- STARTING CONTINUOUS MONITOR (Press 'n' to return) ---{C.RES}")
            header = f"{'TIME':<12} | {'BAT':<4} | {'POS (X, Y, Z)':<24} | {'LID':<3} {'CMP':<3} {'DEP':<3} {'CSI':<3} {'LOC':<3} {'SYS':<3} | {'CMD':<10}"
            print(f"{C.BLU}{header}{C.RES}")
            print("-" * 95)
            self.header_printed = True

        # 2. 准备数据
        t_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        bat_str = f"{C.GRN if self.battery > 30 else C.RED}{self.battery:>3}%{C.RES}"
        
        # 坐标格式化
        pos_str = f"({self.uav_pos[0]:6.2f}, {self.uav_pos[1]:6.2f}, {self.uav_pos[2]:6.2f})"
        
        # 状态标志位
        def get_mark(key):
            return f"{C.GRN}O{C.RES}" if self.status.get(key, 0) == 1 else f"{C.RED}X{C.RES}"

        st_str = f"{get_mark('Lidar')}   {get_mark('Comp')}   {get_mark('Depth')}   {get_mark('CSI')}   {get_mark('Local')}   {get_mark('Overall')}  "
        cmd_str = f"{self.last_input_cmd}"

        # 3. 打印一行
        line = f"{t_str:<12} | {bat_str} | {pos_str:<24} | {st_str}| {cmd_str}"
        print(line)
        
        # 【诊断建议】如果 POS 全是 0，请检查 SDK 终端是否有报错，或者 rostopic list 确认话题名称
        def get_mark(key):
            val = self.status.get(key, 0)
            if val == 1: return f"{C.GRN}O{C.RES}"
            return f"{C.RED}X{C.RES}"

        # 获取所有6个状态
        s_lid = get_mark('Lidar')
        s_cmp = get_mark('Comp')
        s_dep = get_mark('Depth')
        s_csi = get_mark('CSI')
        s_loc = get_mark('Local')
        s_sys = get_mark('Overall')
        
        # 拼接状态块
        st_str = f"{s_lid}   {s_cmp}   {s_dep}   {s_csi}   {s_loc}   {s_sys}  "

        cmd_str = f"{self.last_input_cmd}"

        # 3. 打印一行
        line = f"{t_str:<12} | {bat_str} | {pos_str:<24} | {st_str}| {cmd_str}"
        print(line)

    # ==========================================
    #            网络通信
    # ==========================================
    def connect_loop(self):
        while self.running:
            if self.connected:
                time.sleep(1)
                continue
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect((TARGET_IP, TARGET_PORT))
                self.sock.settimeout(None) 
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connected = True
                self.add_log(f"Connected to {TARGET_IP}")
            except:
                self.connected = False
                time.sleep(2)

    def heartbeat_loop(self):
        while self.running:
            if self.connected: self.send_packet(CMD_HEARTBEAT)
            time.sleep(1.0)

    def send_packet(self, cmd_id, payload=b''):
        if not self.connected or not self.sock: return
        try:
            length = 1 + len(payload)
            header = b'\xFF\xFE' + struct.pack('<H', length) + struct.pack('B', cmd_id)
            self.sock.sendall(header + payload)
            if self.show_hex and cmd_id != CMD_HEARTBEAT:
                self.add_log(f"TX: {header.hex()}", "TX")
        except:
            self.connected = False
            self.sock.close()

    def rx_loop(self):
        buffer = b''
        while self.running and self.connected:
            try:
                data = self.sock.recv(1024)
                if not data: 
                    self.connected = False
                    self.add_log("Disconnected", "ERR")
                    break
                buffer += data
                while len(buffer) >= 5:
                    if buffer[0] != 0xFF or buffer[1] != 0xFE:
                        buffer = buffer[1:]
                        continue
                    data_len = struct.unpack('<H', buffer[2:4])[0]
                    total_len = 4 + data_len
                    if len(buffer) < total_len: break
                    payload = buffer[4:total_len]
                    self.parse_payload(payload)
                    buffer = buffer[total_len:]
            except:
                self.connected = False
                break

    def parse_payload(self, payload):
        if not payload: return
        cmd = payload[0]
        data = payload[1:]
        if cmd == RESP_POS and len(data) >= 12:
            self.uav_pos = struct.unpack('<fff', data[:12])
        elif cmd == RESP_BATTERY and len(data) >= 4:
            self.battery = struct.unpack('<i', data[:4])[0]
        # 【修改】这里改为 >= 6，且只解包 6 个字节
        elif cmd == RESP_STATUS and len(data) >= 6:
            s = struct.unpack('BBBBBB', data[:6])
            self.status.update({
                "Overall": s[0], "Lidar": s[1], "Comp": s[2],
                "Depth": s[3], "CSI": s[4], "Local": s[5]
                # 注意：移除了 FlightState
            })
        elif cmd == 0x02:
            res = data[0]
            msg = "WPs Upload: SUCCESS" if res == 1 else "WPs Upload: FAILED"
            self.add_log(msg, "RX")

    # ==========================================
    #            业务逻辑函数
    # ==========================================
    def send_waypoints_test(self):
        # (x,y,z, yaw, hover_time, gimbal_pitch,gimbal_yaw)
        waypoints = [
            (0.0, 0.0, 1.0,  0.0,  0.0,  0.0, 0.0), 
            (0.94, -0.25, 0.8,  0.0,  0.0,  0.0, 0.0),
            (2.58, -0.25, 0.8,  0.0,  0.0,  0.0, 0.0),
            (2.58, 0.3, 0.8,  0.0,  0.0,  0.0, 0.0),
            (0.94, 0.3, 0.8,  0.0,  0.0,  0.0, 0.0),
            # (3.6, 0.2, 0.7,  90.0,  0.0,  0.0, 0.0),
            # (2.3, 0.2, 0.7,  90.0,  0.0,  0.0, 0.0),
            
        ]
        count = len(waypoints)
        if count > 255: return
        payload = struct.pack('B', count)
        for i, wp in enumerate(waypoints):
            wp_bytes = struct.pack('<fffffff', *wp)
            payload += wp_bytes
        self.add_log(f"Upload {count} New WPs", "TX")
        self.send_packet(CMD_WAYPOINTS, payload)
        self.last_input_cmd = "Upload WPs"

    def update_gimbal(self, pitch_delta, yaw_delta, reset=False):
        if reset:
            self.gimbal_pitch = 0.0
            self.gimbal_yaw = 0.0
        else:
            self.gimbal_pitch += pitch_delta
            self.gimbal_yaw += yaw_delta
            
        self.gimbal_pitch = max(-90.0, min(90.0, self.gimbal_pitch))
        self.gimbal_yaw = max(-90.0, min(90.0, self.gimbal_yaw))
        
        self.send_packet(CMD_GIMBAL_PIT, struct.pack('<f', self.gimbal_pitch))
        self.send_packet(CMD_GIMBAL_YAW, struct.pack('<f', self.gimbal_yaw))
        
        self.last_input_cmd = f"Gim P{self.gimbal_pitch:.0f} Y{self.gimbal_yaw:.0f}"
        self.add_log(f"Gimbal P{self.gimbal_pitch} Y{self.gimbal_yaw}", "TX")

    # ==========================================
    #            主循环 (按键映射)
    # ==========================================
    def run(self):
        threading.Thread(target=self.connect_loop, daemon=True).start()
        threading.Thread(target=self.rx_loop, daemon=True).start()
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        
        import tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())

        try:
            while self.running:
                # 渲染逻辑
                if self.view_mode == 0:
                    self.render_menu()
                    time.sleep(0.2) 
                else:
                    self.render_dashboard()
                    time.sleep(0.25) 
                
                # 非阻塞输入检测
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    
                    # === 模式切换 ===
                    if ch == 'm': 
                        self.view_mode = 1
                        self.header_printed = False 
                        continue
                    elif ch == 'n': 
                        self.view_mode = 0
                        continue
                    elif ch == '9': 
                        break

                    # === 指令映射表 ===
                    cmd_map = {
                        'w': (CMD_MOVE_FWD,   b'', "Move Fwd"),
                        's': (CMD_MOVE_BCK,   b'', "Move Back"),
                        'a': (CMD_MOVE_LEFT,  b'', "Move Left"), 
                        'd': (CMD_MOVE_RIGHT, b'', "Move Right"),
                        'q': (CMD_YAW, struct.pack('<f', 1.0),  "Yaw Left"),
                        'e': (CMD_YAW, struct.pack('<f', -1.0), "Yaw Right"),
                        'r': (CMD_UP_DOWN, struct.pack('<f', 1.0),  "Up"),
                        'f': (CMD_UP_DOWN, struct.pack('<f', -1.0), "Down"),
                        ' ': (CMD_INTERRUPT, b'', "BRAKE"),
                        '2': (CMD_RETURN,    b'', "RTH"),
                        '3': (CMD_LAND,      b'', "Land"),
                        '0': (CMD_REBOOT,    b'', "Reboot"),
                        '5': (CMD_RESUME,    b'', "Resume"),
                    }

                    if ch in cmd_map:
                        c_id, c_data, c_desc = cmd_map[ch]
                        self.send_packet(c_id, c_data)
                        self.last_input_cmd = c_desc
                        self.add_log(f"{c_desc}", "TX")
                    
                    elif ch == '1': self.send_waypoints_test()
                    elif ch == '4':
                        self.send_packet(CMD_FLY_TO, struct.pack('<fff', 2.0, 0.0, 1.0))###定点
                        self.last_input_cmd = "FlyTo (1,-0.5,1)"
                        self.add_log("FlyTo 0x03", "TX")
                    
                    elif ch == 'i': self.update_gimbal(10.0, 0.0)
                    elif ch == 'k': self.update_gimbal(-10.0, 0.0)
                    elif ch == 'j': self.update_gimbal(0.0, -10.0)
                    elif ch == 'l': self.update_gimbal(0.0, 10.0)
                    elif ch == 'u': self.update_gimbal(0.0, 0.0, reset=True)

        except Exception as e:
            print(f"\nError: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.running = False
            print("\nExited.")

if __name__ == '__main__':
    DebugStation().run()