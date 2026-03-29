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
#    配置与常量 (严格遵循 Protocol 3.4)
# ==========================================
TARGET_IP = '127.0.0.1' 
TARGET_PORT = 9527

# --- 协议指令表 ---
CMD_HEARTBEAT   = 0x00 
CMD_WAYPOINTS   = 0x02 # 上传航点
CMD_FLY_TO      = 0x03 # 指定位置飞行
CMD_MOVE_FWD    = 0x04 # 前进 (Float step)
CMD_MOVE_BCK    = 0x05 # 后退 (Float step)
CMD_MOVE_RIGHT  = 0x06 # 右移 (Float step)
CMD_MOVE_LEFT   = 0x07 # 左移 (Float step)
CMD_YAW         = 0x08 # 偏航 (Float step)
CMD_UP_DOWN     = 0x09 # 高度 (Float step)
CMD_GIMBAL_YAW  = 0x0A # 云台Yaw
CMD_GIMBAL_PIT  = 0x0B # 云台Pitch
CMD_EXPOSURE    = 0x0C # 相机曝光 (Protocol 3.4 New)
CMD_BATTERY     = 0x0D # 电量(通常是回传，但也可能查询)
CMD_RETURN      = 0x0F # 一键返航/结束任务 (Protocol 3.4 Modified)
CMD_ZOOM        = 0x10 # 相机变焦 (Protocol 3.4 New)
CMD_EMERG_LAND  = 0x11 # 紧急降落 (Protocol 3.4 Modified)
CMD_COLOR_MAP   = 0x12 # 彩色建图 (Protocol 3.4 ID Fixed)
CMD_BASE_SYS    = 0x13 # 基础算法 (Protocol 3.4 ID Fixed)
CMD_RESUME      = 0x14 # 继续任务 (Protocol 3.4 ID Fixed)
CMD_REBOOT      = 0x15 # [新增] 一键重启
# --- 协议回传表 ---
RESP_POS        = 0x01 # 坐标
RESP_ACK        = 0x02 # 航点确认
RESP_BATTERY    = 0x0D # 电量
RESP_STATUS     = 0x0E # 综合状态

# --- 默认步进值 ---
STEP_MOVE = 0.1  # 每次按键移动 0.2米
STEP_YAW  = 0.1  # 每次按键转动 0.2弧度
STEP_ALT  = 0.1  # 每次按键升降 0.2米
STEP_ZOOM = 1.0  # 变焦步进值（debug中为了方便，用步进值演示）
STEP_EXP  = 50.0 # 曝光步进值

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
        self.header_printed = False 
        self.show_hex = False
        self.log_buffer = [] 
        self.max_logs = 10
        
        # 数据缓存
        self.uav_pos = [0.0, 0.0, 0.0]
        self.uav_yaw_feedback = 0.0  # 【新增】 用于存储接收到的Yaw
        self.gimbal_pitch_feedback = 0.0 # 【新增】 用于存储接收到的云台Pitch
        self.battery = 0
        
        # 状态字典 (Protocol 3.4: 7 bytes)
        self.status = {
            "Overall": 0, "Lidar": 0, "Comp": 0, "Depth": 0,
            "CSI": 0, "Local": 0, "FlightState": 0
        }
        self.last_input_cmd = "None"
        
        # 模拟的用户设定值
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.cam_zoom = 1.0
        self.cam_exp = 100.0

    # ==========================================
    #            UI 渲染核心
    # ==========================================
    def add_log(self, msg, type="INFO"):
        """添加日志到滚动缓冲区"""
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
        """模式0: 详细菜单模式"""
        output = "\033[2J\033[H" # 清屏
        
        conn_str = f"{C.GRN}CONNECTED{C.RES}" if self.connected else f"{C.RED}DISCONNECTED{C.RES}"
        bat_str = f"{self.battery}%" if self.connected else "--"
        # 【修改】 显示实时反馈的 Pitch 值 (FB)
        output += f"  Gimbal    : P={self.gimbal_pitch_feedback:.1f} (FB) Y={self.gimbal_yaw:.1f} (CMD)\n"
        output += f"  Camera    : Zoom={self.cam_zoom:.1f}x Exp={self.cam_exp:.0f}\n"

        output += f"{C.WHT}=========================================={C.RES}\n"
        output += f"   UAV DEBUG STATION V7.0 (Proto 3.4)  |  {conn_str}\n"
        output += f"{C.WHT}=========================================={C.RES}\n"
        
        output += f"  Target IP : {TARGET_IP}:{TARGET_PORT}\n"
        output += f"  Battery   : {bat_str}\n"
        output += f"  Gimbal    : P={self.gimbal_pitch:.1f} Y={self.gimbal_yaw:.1f}\n"
        output += f"  Camera    : Zoom={self.cam_zoom:.1f}x Exp={self.cam_exp:.0f}\n"
        output += f"  Last Cmd  : {C.CYN}{self.last_input_cmd}{C.RES}\n\n"
        
        output += f"{C.WHT}  [Motion] (Step: {STEP_MOVE}m){C.RES}\n"
        output += f"    {C.YEL}W/S{C.RES}: Fwd/Back   {C.YEL}A/D{C.RES}: Left/Right\n"
        output += f"    {C.YEL}R/F{C.RES}: Up/Down    {C.YEL}Q/E{C.RES}: Yaw L/R\n"
        output += f"    {C.YEL}SPC{C.RES}: Brake/Hover\n\n"

        output += f"{C.WHT}  [Payload] (Step: 5deg/1x/50){C.RES}\n"
        output += f"    {C.MAG}I/K{C.RES}: Gim Pitch  {C.MAG}J/L{C.RES}: Gim Yaw\n"
        output += f"    {C.MAG}Z/X{C.RES}: Zoom -/+   {C.MAG}V/B{C.RES}: Exp -/+\n\n"
        
        output += f"{C.WHT}  [Mission & System]{C.RES}\n"
        output += f"    {C.GRN}1{C.RES}: Upload WPs (Test)     {C.GRN}2{C.RES}: RTH/Kill (0x0F)\n"
        output += f"    {C.GRN}3{C.RES}: Emergency Land (0x11) {C.GRN}4{C.RES}: FlyTo (0x03)\n"
        output += f"    {C.GRN}5{C.RES}: Resume (0x14)         {C.GRN}6{C.RES}: ColorMap (0x12)\n"
        output += f"    {C.GRN}7{C.RES}: Base Algo (0x13)\n"
        output += f"            {C.RED}0: REBOOT (0x15){C.RES}\n" # [新增]

        output += f"{C.WHT}------------------------------------------{C.RES}\n"
        for log in self.log_buffer:
            output += log + "\n"
        output += f"{C.WHT}------------------------------------------{C.RES}\n"
        output += f"  > 'm': Monitor Mode | '9': Exit\n"
        
        sys.stdout.write(output)
        sys.stdout.flush()

    def render_dashboard(self):
        """模式1: 持续输出模式"""
        if not self.header_printed:
            print(f"\n{C.WHT}--- CONTINUOUS MONITOR (Press 'n' to return) ---{C.RES}")
            # 【修改】 表头增加 GimPitch (GPIT)
            header = f"{'TIME':<10} | {'BAT':<3} | {'POS (X, Y, Z)':<22} | {'YAW':<6} | {'GPIT':<6} | {'LID':<3} {'CMP':<3} {'DEP':<3} {'CSI':<3} {'LOC':<3} {'SYS':<3} {'FLY':<3} | {'CMD':<10}"
            print(f"{C.BLU}{header}{C.RES}")
            print("-" * 125) # 调整分隔线长度
            self.header_printed = True

        t_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-4]
        bat_str = f"{self.battery}%"
        pos_str = f"({self.uav_pos[0]:5.2f},{self.uav_pos[1]:5.2f},{self.uav_pos[2]:5.2f})"
        
        # 【新增】 格式化 Yaw 字符串
        yaw_str = f"{self.uav_yaw_feedback:5.1f}"
        # 【新增】 格式化 Pitch 字符串
        gpit_str = f"{self.gimbal_pitch_feedback:5.1f}"
        def gm(k): return f"{C.GRN}1{C.RES}" if self.status.get(k,0)==1 else f"{C.RED}0{C.RES}"
        
        fs_val = self.status.get("FlightState", 0)
        fs_str = f"{C.CYN}{fs_val}{C.RES}"

        st_str = f"{gm('Lidar')}   {gm('Comp')}   {gm('Depth')}   {gm('CSI')}   {gm('Local')}   {gm('Overall')}   {fs_str}  "
        
        # 【修改】 打印行增加 yaw_str
        print(f"{t_str:<10} | {bat_str:<3} | {pos_str:<22} | {yaw_str:<6} | {st_str}| {self.last_input_cmd}")
        # 【修改】 打印行增加 gpit_str
        print(f"{t_str:<10} | {bat_str:<3} | {pos_str:<22} | {yaw_str:<6} | {gpit_str:<6} | {st_str}| {self.last_input_cmd}")
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
        """发送心跳"""
        while self.running:
            if self.connected: self.send_packet(CMD_HEARTBEAT)
            time.sleep(1.0)

    def send_packet(self, cmd_id, payload=b''):
        if not self.connected or not self.sock: return
        try:
            length = 1 + len(payload) # Data + Mode(1)
            # 协议: 0xFF 0xFE + Len(2) + Type(1) + Data(n)
            header = b'\xFF\xFE' + struct.pack('<H', length) + struct.pack('B', cmd_id)
            self.sock.sendall(header + payload)
            if self.view_mode == 0 and cmd_id != CMD_HEARTBEAT:
                # 记录简要TX日志
                pass 
        except Exception as e:
            self.connected = False
            self.add_log(f"Send Error: {e}", "ERR")
            self.sock.close()

    def rx_loop(self):
        buffer = b''
        while self.running:
            if not self.connected:
                time.sleep(0.5)
                continue
            try:
                data = self.sock.recv(1024)
                if not data: 
                    self.connected = False
                    self.add_log("Connection Lost", "ERR")
                    continue
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
            except Exception as e:
                self.connected = False
                self.add_log(f"RX Error: {e}", "ERR")

    def parse_payload(self, payload):
        if not payload: return
        cmd = payload[0]
        data = payload[1:]
        
        # 0x01: 坐标 + 姿态
        # 【修改】 长度检查由 >=16 改为 >=20 (5个float)，并解析第5个float
        if cmd == RESP_POS and len(data) >= 20:
            vals = struct.unpack('<fffff', data[:20])
            self.uav_pos = [vals[0], vals[1], vals[2]]
            self.uav_yaw_feedback = vals[3] 
            self.gimbal_pitch_feedback = vals[4] # 【新增】 获取云台Pitch
            
        # 0x0D: 电量 (4 bytes)
        elif cmd == RESP_BATTERY and len(data) >= 4:
            self.battery = struct.unpack('<i', data[:4])[0]
            
        # 0x0E: 综合状态 (7 bytes - Protocol 3.4)
        elif cmd == RESP_STATUS and len(data) >= 7:
            s = struct.unpack('BBBBBBB', data[:7])
            self.status.update({
                "Overall": s[0], "Lidar": s[1], "Comp": s[2],
                "Depth": s[3], "CSI": s[4], "Local": s[5],
                "FlightState": s[6]
            })
            
        # 0x02: 航点确认
        elif cmd == RESP_ACK:
            res = data[0] if len(data) > 0 else 0
            msg = "WP Upload: OK (0x01)" if res == 1 else "WP Upload: FAIL (0x00)"
            self.add_log(msg, "RX")

    # ==========================================
    #            控制逻辑
    # ==========================================
    def send_waypoints_test(self):
        """
        上传测试航点 (Protocol 3.4)
        格式: Count(2B) + Points(n * 28B)
        每点: x, y, z, uav_yaw, pitch, zoom, time (7个float)
        """
        # 定义测试点: (x, y, z, uav_yaw,  pitch,  zoom,  time)
        points = [
            (0.0, 0.0, 0.8, 0.0,  0.0,   1.0,   0.0),  # Takeoff (Time=0)

            (0.63, -0.79, 1.55, -90.0,  0.0,   1.0,   0.0),

            (1.62, -0.79, 1.55, -90.0,  6.0,   1.0,   1.2),
            (2.62, -0.79, 1.55, -90.0,  6.0,   1.0,   1.2), # 1.8s 适合
            (3.62, -0.79, 1.55, -90.0,  6.0,   1.0,   1.2),

            (3.62, -0.79, 0.5, -90.0,  6.0,   1.0,   1.2),
            (2.62, -0.79, 0.5, -90.0,  6.0,   1.0,   1.2),
            (1.62, -0.79, 0.5, -90.0,  6.0,   1.0,   1.2),

            (0.63, -0.79, 0.5, 90.0,  0.0,   1.0,   0.0),

            (1.62, -0.79, 0.5, 90.0,  6.0,   1.0,   1.2),
            (2.62, -0.79, 0.5, 90.0,  6.0,   1.0,   1.2),
            (3.62, -0.79, 0.5, 90.0,  6.0,   1.0,   1.2),

            (3.62, -0.79, 1.55, 90.0,  6.0,   1.0,   1.2),
            (2.62, -0.79, 1.55, 90.0,  6.0,   1.0,   1.2),
            (1.62, -0.79, 1.55, 90.0,  6.0,   1.0,   1.2),

            (0.50, 0.0, 1.55, 90.0,  0.0,   1.0,   0.0),

            (1.62, 1.43, 1.55, 90.0,  6.0,   1.0,   1.2),
            (2.62, 1.43, 1.55, 90.0,  6.0,   1.0,   1.2),
            (3.62, 1.43, 1.55, 90.0,  6.0,   1.0,   1.2),

            (3.62, 1.43, 0.5, 90.0,  6.0,   1.0,   1.2),
            (2.62, 1.43, 0.5, 90.0,  6.0,   1.0,   1.2),
            (1.62, 1.43, 0.5, 90.0,  6.0,   1.0,   1.2),

            (0.63, 1.43, 0.5, -90.0,  0.0,   1.0,   0.0),

            (1.62, 1.43, 0.5, -90.0,  6.0,   1.0,   1.2),
            (2.62, 1.43, 0.5, -90.0,  6.0,   1.0,   1.2),
            (3.62, 1.43, 0.5, -90.0,  6.0,   1.0,   1.2),

            (3.62, 1.43, 1.55, -90.0,  6.0,   1.0,   1.2),
            (2.62, 1.43, 1.55, -90.0,  6.0,   1.0,   1.2),
            (1.62, 1.43, 1.55, -90.0,  6.0,   1.0,   1.2),

            (0.50, 0.0, 0.7, 0.0,  0.0,   1.0,   0.0),
        ]
        
        count = len(points)
        # 构造负载: Count(1B) + Points(n * 24B)
        payload = struct.pack('<H', count)
        for p in points:
            # 协议要求: x, y, z, uav_yaw, pitch, zoom, time
            # p列表当前只有6个元素，我们需要插入 uav_yaw (设为0.0)
            x, y, z, uav_yaw,pitch, zoom, time = p[0], p[1], p[2], p[3], p[4], p[5],p[6]
            
            # 【修改2】打包 7个 Float
            payload += struct.pack('<fffffff', x, y, z, uav_yaw, pitch, zoom, time)            
        self.add_log(f"Uploading {count} WPs...", "TX")
        self.send_packet(CMD_WAYPOINTS, payload)
        self.last_input_cmd = "Upload WPs"


    def send_fly_to(self):
        """发送指定位置飞行指令 (Protocol 3.5 Compliance Fix)"""
        # 测试: 飞到 x=3.0, y=0.0, z=0.7
        x, y, z = 3.0, 1.4, 1.0
        uav_yaw = 0.0        # 【新增】 协议要求的无人机偏航角
        pitch, zoom = 0.0, 1.0
        hover_time = 3.0 
        
        
        # 修改后 (7 floats - 严格对应协议 0x03):
        # 顺序: x, y, z, UAV_YAW, GIM_PITCH, ZOOM, TIME
        payload = struct.pack('<fffffff', x, y, z, uav_yaw, pitch, zoom, hover_time)
        
        self.send_packet(CMD_FLY_TO, payload)
        
        # 更新本地状态用于UI显示
        # 注意：实际位置等回传才会变，这里只是更新“目标值”显示
        self.gimbal_pitch = pitch
        self.cam_zoom = zoom
        
        self.last_input_cmd = f"FlyTo {x},{y},{z} T={hover_time}s"
        self.add_log(f"FlyTo ({x},{y},{z}) Time={hover_time}", "TX")
    def update_gimbal(self, step_pitch, step_yaw):
        """
        发送云台步进控制指令
        step_pitch: 俯仰步进值 (例如 5.0 或 -5.0)
        step_yaw: 偏航步进值 (例如 5.0 或 -5.0)
        """
        # 注意：这里不再累加本地变量 self.gimbal_pitch/yaw
        # 而是直接发送步进值给 SDK
        
        # 发送 Pitch 步进
        if abs(step_pitch) > 0:
            self.send_packet(CMD_GIMBAL_PIT, struct.pack('<f', step_pitch))
            
        # 发送 Yaw 步进
        if abs(step_yaw) > 0:
            self.send_packet(CMD_GIMBAL_YAW, struct.pack('<f', step_yaw))
            
        self.last_input_cmd = f"Gim Step P{step_pitch:+.1f} Y{step_yaw:+.1f}"

    def update_camera(self, d_zoom, d_exp):
        self.cam_zoom += d_zoom
        self.cam_exp += d_exp
        
        self.cam_zoom = max(1.0, min(10.0, self.cam_zoom))
        self.cam_exp = max(0.0, min(10000.0, self.cam_exp))
        
        if abs(d_zoom) > 0:
            self.send_packet(CMD_ZOOM, struct.pack('<f', self.cam_zoom))
            self.last_input_cmd = f"Zoom {self.cam_zoom:.1f}x"
        if abs(d_exp) > 0:
            self.send_packet(CMD_EXPOSURE, struct.pack('<f', self.cam_exp))
            self.last_input_cmd = f"Exp {self.cam_exp:.0f}"

    def send_move(self, cmd_id, val):
        """发送移动指令 (带步进值)"""
        # Protocol 3.4: 必须发送 Float 步进值
        self.send_packet(cmd_id, struct.pack('<f', val))

    # ==========================================
    #            主循环
    # ==========================================
    def run(self):
        # 启动线程
        threading.Thread(target=self.connect_loop, daemon=True).start()
        threading.Thread(target=self.rx_loop, daemon=True).start()
        threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        
        # 终端设置
        import tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

        try:
            while self.running:
                # 刷新界面
                if self.view_mode == 0:
                    self.render_menu()
                    time.sleep(0.1) 
                else:
                    self.render_dashboard()
                    time.sleep(0.25) 
                
                # 键盘输入
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    
                    if ch == 'm': 
                        self.view_mode = 1
                        self.header_printed = False 
                        continue
                    elif ch == 'n': 
                        self.view_mode = 0
                        continue
                    elif ch == '9': 
                        break

                    # --- 运动控制 (发送步进值) ---
                    if ch == 'w':   self.send_move(CMD_MOVE_FWD, STEP_MOVE); self.last_input_cmd="Fwd"
                    elif ch == 's': self.send_move(CMD_MOVE_BCK, STEP_MOVE); self.last_input_cmd="Back"
                    elif ch == 'a': self.send_move(CMD_MOVE_LEFT, STEP_MOVE); self.last_input_cmd="Left"
                    elif ch == 'd': self.send_move(CMD_MOVE_RIGHT, STEP_MOVE); self.last_input_cmd="Right"
                    elif ch == 'r': self.send_move(CMD_UP_DOWN, STEP_MOVE); self.last_input_cmd="Up"
                    elif ch == 'f': self.send_move(CMD_UP_DOWN, -STEP_MOVE); self.last_input_cmd="Down"
                    elif ch == 'q': self.send_move(CMD_YAW, STEP_YAW); self.last_input_cmd="Yaw L"
                    elif ch == 'e': self.send_move(CMD_YAW, -STEP_YAW); self.last_input_cmd="Yaw R"
                    
                    # --- 负载控制 ---
                    elif ch == 'i': self.update_gimbal(6.0, 0)
                    elif ch == 'k': self.update_gimbal(-6.0, 0)
                    elif ch == 'j': self.update_gimbal(0, 6.0)
                    elif ch == 'l': self.update_gimbal(0, -6.0)
                    elif ch == 'z': self.update_camera(-STEP_ZOOM, 0)
                    elif ch == 'x': self.update_camera(STEP_ZOOM, 0)
                    elif ch == 'v': self.update_camera(0, -STEP_EXP)
                    elif ch == 'b': self.update_camera(0, STEP_EXP)

                    # --- 系统指令 ---
                    elif ch == ' ': 
                        # 空格键：刹车/切换回手动/恢复任务 (Protocol 0x14)
                        self.send_packet(CMD_RESUME, b'') 
                        self.last_input_cmd="Stop/Resume (0x14)"
                        self.add_log("Sent Resume (0x14)", "TX")
                    
                    elif ch == '1': self.send_waypoints_test()
                    elif ch == '2': self.send_packet(CMD_RETURN, b''); self.last_input_cmd="RTH (0x0F)"
                    elif ch == '3': self.send_packet(CMD_EMERG_LAND, b''); self.last_input_cmd="Emerg Land (0x11)"
                    elif ch == '4': self.send_fly_to()
                    elif ch == '5': self.send_packet(CMD_RESUME, b''); self.last_input_cmd="Resume (0x14)"
                    elif ch == '6': self.send_packet(CMD_COLOR_MAP, b''); self.last_input_cmd="ColorMap (0x12)"
                    elif ch == '7': self.send_packet(CMD_BASE_SYS, b''); self.last_input_cmd="BaseAlgo (0x13)"
                    # [新增] 0: Reboot
                    elif ch=='0': 
                        self.send_packet(CMD_REBOOT, b'')
                        self.last_input_cmd = "Reboot (0x15)"
                        self.add_log("Sent Reboot (0x15)", "TX")

        except Exception as e:
            print(f"\nInternal Error: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.running = False
            print("\nStation Exited.")

if __name__ == '__main__':
    DebugStation().run()
