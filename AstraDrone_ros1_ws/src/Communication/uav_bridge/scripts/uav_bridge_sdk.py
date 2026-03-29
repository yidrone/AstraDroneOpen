#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import struct
import threading
import os
import subprocess
import time
import math
from tf.transformations import euler_from_quaternion

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("[WARN] psutil not installed. CPU monitoring disabled.")

# ROS 消息
from std_msgs.msg import Float32, Int8, Bool, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Point, Twist
from mavros_msgs.msg import State
from sensor_msgs.msg import Image, Imu, BatteryState
from nav_msgs.msg import Odometry

try:
    from uav_bridge.msg import CustomMsg
except ImportError:
    pass

class UAVBridgeSDK:
    def __init__(self):
        rospy.init_node('uav_bridge_sdk', anonymous=False)
        rospy.loginfo("========================================")
        rospy.loginfo("   UAV Bridge SDK V5.7 (Battery Spoof)  ")
        rospy.loginfo("========================================")

        # --- 1. 参数配置 ---
        self.tcp_port = rospy.get_param('~port', 9527)
        self.script_path = rospy.get_param('~script_path', '/home/uav/AstraDroneOpen/scripts/run_sh/patrol_sh')
        self.yaml_path = rospy.get_param('~yaml_path', '/home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_control/config/astra_real.yaml')
        
        if not self.script_path.endswith('/'): self.script_path += '/'
           
        # --- 2. 通信变量 ---
        self.client_socket = None
        self.lock = threading.Lock()
        self.rx_buffer = b''
        
        # --- 3. 状态缓存 ---
        self.uav_pos = [0.0, 0.0, 0.0]
        self.uav_yaw = 0.0              
        self.uav_vel = [0.0, 0.0, 0.0] 
        self.battery_percent = 0
        self.is_connected = False
        self.is_armed = False
        self.current_mode = "MANUAL"
        self.odom_covariance = 99.9    
        self.spoof_battery_enabled = False

        # --- 4. 频率监测缓存 ---
        self.hist_lidar = []
        self.hist_depth_d = [] 
        self.hist_depth_r = [] 
        self.hist_csi   = []
        self.hist_odom  = []
        
        # --- 5. 状态标志位 (Protocol 0x0E) ---
        self.status_overall = 0
        self.status_lidar = 0
        self.status_comp = 0
        self.status_depth = 0
        self.status_csi = 0
        self.status_loc = 0
        self.flight_state = 0
        self.status_mapping = 0 

        # --- 6. 云台/相机反馈 ---
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.real_gimbal_pitch = 0.0 
        self.real_gimbal_yaw = 0.0 

        # --- 7. 启动逻辑相关 ---
        self.launched_scripts = set()   
        self.lio_launched = False       
        self.base_launched = False      
        self.imu_stable_start_time = None
        self.level_threshold_rad = math.radians(5.0)
        self.imu_stable_duration_sec = 5.0

        # --- 8. ROS 订阅 ---
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_pose)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.cb_odom) 
        rospy.Subscriber("/mavros/state", State, self.cb_state)
        rospy.Subscriber("/mavros/battery", BatteryState, self.cb_battery)
        rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu_mavros) 
        rospy.Subscriber("/gimbal/attitude", Float32MultiArray, self.cb_gimbal_attitude)
        
        try:
            if 'CustomMsg' in globals():
                rospy.Subscriber("/livox/lidar", CustomMsg, self.cb_lidar)
            else:
                pass 
        except:
            pass
        
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cb_depth_d)
        rospy.Subscriber("/camera/depth/color/image_raw", Image, self.cb_depth_r)
        self.sub_csi = rospy.Subscriber("/csi_camera/image_raw", Image, self.cb_csi)
        self.sub_gimbal = rospy.Subscriber("/gimbal/cmd_angle", Point, self.cb_gimbal)

        # --- 9. ROS 发布 ---
        self.pub_mode_switch = rospy.Publisher("/astra_control/mode_switch", Int8, queue_size=1)
        self.pub_manual_cmd = rospy.Publisher("/astra_control/manual_cmd", Twist, queue_size=1)
        self.pub_gimbal_angle = rospy.Publisher("/gimbal/cmd_angle", Point, queue_size=1)
        self.pub_fly_to_cmd = rospy.Publisher("/astra_control/fly_to_cmd", PoseStamped, queue_size=1)
        
        self.pub_camera_exposure = rospy.Publisher("/camera/cmd_exposure", Float32, queue_size=1)
        self.pub_camera_zoom = rospy.Publisher("/zoom_value", Float32, queue_size=1)

        # --- 10. 初始启动 ---
        self.tcp_thread = threading.Thread(target=self.tcp_server_loop)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

        rospy.Timer(rospy.Duration(0.1), self.timer_status_callback)
        self.slow_sync_tick = 0

        rospy.loginfo(f"[READY] Listening on Port {self.tcp_port}")
        rospy.loginfo("[AUTO-START] Launching base.sh automatically on startup...")
        self.launch_script('base.sh', 'BASIC_SYSTEM')
        self.base_launched = True
        self.lio_launched = False
        self.imu_stable_start_time = None        

    def update_freq(self, history_list):
        now = time.time()
        history_list.append(now)
        while history_list and (now - history_list[0] > 1.0):
            history_list.pop(0)
        return len(history_list)

    def clean_and_get_count(self, history_list):
        now = time.time()
        while history_list and (now - history_list[0] > 1.0):
            history_list.pop(0)
        return len(history_list)

    def cb_imu_mavros(self, msg):
        if not self.base_launched or self.lio_launched: return     
        orientation_q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        if abs(roll) < self.level_threshold_rad and abs(pitch) < self.level_threshold_rad:
            if self.imu_stable_start_time is None: self.imu_stable_start_time = rospy.Time.now()
            elif (rospy.Time.now() - self.imu_stable_start_time).to_sec() > self.imu_stable_duration_sec:
                rospy.loginfo("[BOOT] IMU Stable. Launching Fast-LIO...")
                self.launch_script('lio.sh', 'LIO_SLAM')
                self.lio_launched = True 
        else:
            self.imu_stable_start_time = None
            rospy.loginfo("[BOOT] IMU Not Stable. !!!")
            
    def cb_gimbal_attitude(self, msg):
        if msg.data and len(msg.data) >= 2:
            try:
                self.real_gimbal_pitch = round(msg.data[1], 1)
                self.real_gimbal_yaw = round(msg.data[0], 1)
            except Exception as e:
                rospy.logwarn(f"[SDK] Gimbal attitude parse error: {e}")
                
    def cb_pose(self, msg):
        self.uav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        try:
            orientation_q = msg.pose.orientation
            (_, _, yaw_rad) = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            self.uav_yaw = math.degrees(yaw_rad)
        except Exception:
            pass

    def cb_odom(self, msg):
        self.update_freq(self.hist_odom)
        self.uav_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.odom_covariance = msg.pose.covariance[0]

    def cb_state(self, msg):
        self.is_connected = msg.connected
        self.is_armed = msg.armed 
        self.current_mode = msg.mode

    def cb_battery(self, msg):

        val = msg.percentage
        if self.spoof_battery_enabled:
            val = 0.6
        
        if val <= 1.0: val *= 100
        self.battery_percent = int(val)

    def cb_lidar(self, msg): self.update_freq(self.hist_lidar)
    def cb_depth_d(self, msg): self.update_freq(self.hist_depth_d)
    def cb_depth_r(self, msg): self.update_freq(self.hist_depth_r)
    def cb_csi(self, msg):   self.update_freq(self.hist_csi)
    def cb_gimbal(self, msg): pass
    
    def launch_script(self, script_name, title):
        if script_name in self.launched_scripts:
            rospy.logwarn(f"[SCRIPT] '{script_name}' is already running. Skipping.")
            return

        full_path = self.script_path + script_name
        if os.path.exists(full_path):
            rospy.loginfo(f"[SCRIPT] Executing {script_name} directly...")
            cmd = f"bash {full_path}"
            subprocess.Popen(cmd, shell=True)
            self.launched_scripts.add(script_name)
        else:
            rospy.logerr(f"[SCRIPT] Not found: {full_path}")

    def try_start_patrol(self):
        if self.status_loc != 1:
            rospy.logwarn("[PATROL] Reject: Localization not ready (Status=0).")
            return False
        if not self.base_launched:
            rospy.logwarn("[PATROL] Reject: Basic Algorithm not launched.")
            return False
        if self.battery_percent < 30:
            rospy.logwarn(f"[PATROL] Reject: Battery too low ({self.battery_percent}% < 30%).")
            return False

        rospy.loginfo("[PATROL] Systems GREEN. Starting Patrol Script.")
        self.launch_script('patrol.sh', 'PATROL_LOGIC')
        return True

    def tcp_server_loop(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind(('0.0.0.0', self.tcp_port))
            server_socket.listen(1)
        except Exception as e:
            rospy.logerr(f"[TCP] Bind Failed: {e}")
            return

        while not rospy.is_shutdown():
            try:
                rospy.loginfo("[TCP] Waiting for connection...")
                conn, addr = server_socket.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.lock:
                    self.client_socket = conn
                    self.rx_buffer = b''
                rospy.loginfo(f"[TCP] Client Connected: {addr}")
                while not rospy.is_shutdown():
                    try:
                        data = conn.recv(1024)
                        if not data: break
                        self.rx_buffer += data
                        self.parse_buffer()
                    except: break
            except Exception as e:
                rospy.logerr(f"[TCP] Server Error: {e}")
            finally:
                with self.lock:
                    if self.client_socket:
                        try: self.client_socket.close()
                        except: pass
                        self.client_socket = None
                time.sleep(1)

    def parse_buffer(self):
        while len(self.rx_buffer) >= 5:
            if self.rx_buffer[0] != 0xFF or self.rx_buffer[1] != 0xFE:
                self.rx_buffer = self.rx_buffer[1:]
                continue
            data_len = struct.unpack('<H', self.rx_buffer[2:4])[0]
            total_len = 4 + data_len
            if len(self.rx_buffer) < total_len: break 
            payload = self.rx_buffer[4:total_len]
            self.process_command(payload)
            self.rx_buffer = self.rx_buffer[total_len:]

    def send_packet(self, type_id, content):
        sock = None
        with self.lock: sock = self.client_socket
        if sock is None: return
        payload_len = 1 + len(content)
        header = b'\xFF\xFE' + struct.pack('<H', payload_len) + struct.pack('B', type_id)
        try: sock.sendall(header + content)
        except: pass

    # =========================================
    #            [核心] 指令处理
    # =========================================
    def process_command(self, payload):
        if not payload: return
        cmd = payload[0]
        data = payload[1:]
        
        # --- 0x02: 航点上传 (列表) ---
        if cmd == 0x02:
            if self.handle_waypoints(data):
                self.send_packet(0x02, b'\x01')
                self.try_start_patrol() 
            else:
                self.send_packet(0x02, b'\x00')

        # --- 0x03: 指定位置飞行 (单点模式) ---
        elif cmd == 0x03:
            if len(data) >= 28:
                x, y, z, uav_yaw, g_pitch, zoom, hover_time = struct.unpack('<fffffff', data[:28])
   
                rospy.loginfo(f"[SDK] FlyTo: x={x:.2f}, y={y:.2f}, z={z:.2f}, Y={uav_yaw:.1f}, P={g_pitch:.1f}, Z={zoom:.1f}, T={hover_time:.1f}")
                wps = []
                wps.append({
                    'x': 0.0, 'y': 0.0, 'z': 1.0, 'yaw': 0.0, 
                    'mode': "Takeoff_point", 'time': 0, 
                    'cp': 0, 'cz': 0
                })
                wps.append({
                    'x': x, 'y': y, 'z': z, 'yaw': uav_yaw, 
                    'mode': "Detect_point", 'time': hover_time, 
                    'cp': g_pitch, 'cz': zoom
                })
                wps.append({
                    'x': 0.0, 'y': 0.0, 'z': 0.7, 'yaw': 0.0, 
                    'mode': "Land_point", 'time': 1.0, 
                    'cp': 0, 'cz': 0
                })

        # --- 0x04-0x09: 手动移动控制 ---
        elif cmd in [0x04, 0x05, 0x06, 0x07, 0x08, 0x09]:
            self.pub_mode_switch.publish(Int8(3)) 
            step_val = 0.0
            if len(data) >= 4:
                step_val = struct.unpack('<f', data[:4])[0]

            if cmd == 0x04: self.send_manual_twist(step_val, 0, 0)
            elif cmd == 0x05: self.send_manual_twist(-step_val, 0, 0)
            elif cmd == 0x06: self.send_manual_twist(0, -step_val, 0)
            elif cmd == 0x07: self.send_manual_twist(0, step_val, 0)
            elif cmd == 0x08: self.send_manual_twist(0, 0, 0, yaw_val=step_val)
            elif cmd == 0x09: self.send_manual_twist(0, 0, step_val)

        # --- 相机云台控制 ---
        elif cmd == 0x0A and len(data) >= 4: 
            step_val = struct.unpack('<f', data[:4])[0]
            self.gimbal_yaw += step_val
            if self.gimbal_yaw > 135.0: self.gimbal_yaw = 135.0
            if self.gimbal_yaw < -135.0: self.gimbal_yaw = -135.0
            self.publish_gimbal()

        elif cmd == 0x0B and len(data) >= 4: 
            step_val = struct.unpack('<f', data[:4])[0]
            self.gimbal_pitch += step_val
            if self.gimbal_pitch > 25.0: self.gimbal_pitch = 25.0
            if self.gimbal_pitch < -90.0: self.gimbal_pitch = -90.0
            self.publish_gimbal()

        elif cmd == 0x0C and len(data)>=4:
            val = struct.unpack('<f', data[:4])[0]
            self.pub_camera_exposure.publish(val)
        elif cmd == 0x10 and len(data)>=4:
            val = struct.unpack('<f', data[:4])[0]
            self.pub_camera_zoom.publish(val)
        
        # --- 系统指令 ---
        elif cmd == 0x0F: 
            if self.is_armed and self.flight_state != 0:
                rospy.logwarn("[SDK] RTH triggered.")
                self.pub_mode_switch.publish(Int8(1))
                threading.Thread(target=self.wait_for_land_and_kill).start()
            else:
                rospy.logwarn("[SDK] Not flying. Kill skipped/Handled elsewhere.")
                self.handle_reboot()

        elif cmd == 0x11:
            rospy.logwarn("[SDK] Emergency Land!")
            self.pub_mode_switch.publish(Int8(2))

        elif cmd == 0x12:
            rospy.loginfo("[CMD] Color Map (0x12)")
            self.launch_script('color_map.sh', 'COLOR_MAP')

        elif cmd == 0x13:
            if self.base_launched: return
            rospy.loginfo("[CMD] Base Algo (0x13)")
            self.launch_script('base.sh', 'BASIC_SYSTEM')
            self.base_launched = True
            self.lio_launched = False
            self.imu_stable_start_time = None

        elif cmd == 0x14:
             rospy.loginfo("[SDK] Resume Mission (0x14)")
             self.pub_mode_switch.publish(Int8(5)) 

        elif cmd == 0x15:
             if not self.is_armed:
                 rospy.logwarn("[SDK] Reboot (0x15). Drone is DISARMED. Executing Kill Script.")
                 self.handle_reboot()
             else:
                 rospy.logerr("[SDK] REJECT REBOOT: Drone is ARMED! Please Land & Disarm first.")
        
        # --- 【新增】 0x16: 电池电量欺骗指令 ---
        elif cmd == 0x16 and len(data) >= 4:
            val = struct.unpack('<i', data[:4])[0]
            
            # 如果发送负数（如 -1），则关闭欺骗，恢复真实数据
            if val < 0:
                self.spoof_battery_enabled = False
                rospy.logwarn("[SDK] Battery Spoofing: OFF (Resuming Real Data)")
            else:
                self.spoof_battery_enabled = True
                self.battery_percent = val
                rospy.logwarn(f"[SDK] Battery Spoofing: ON -> Set to {val}%")

    def send_manual_twist(self, x, y, z, yaw_val=0.0):
        t = Twist()
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.linear.z = float(z)
        t.angular.z = float(yaw_val)
        self.pub_manual_cmd.publish(t)

    def publish_gimbal(self):
        msg = Point()
        msg.x, msg.y, msg.z = self.gimbal_pitch, self.gimbal_yaw, 0.0
        self.pub_gimbal_angle.publish(msg)

    def wait_for_land_and_kill(self):
        timeout = 300 
        start_t = time.time()
        rospy.loginfo("[SDK] Waiting for landing and disarm to kill programs...")
        while time.time() - start_t < timeout:
            if not self.is_armed: 
                time.sleep(5)
                rospy.logwarn("[SDK] Drone Disarmed.")
                self.handle_reboot() 
                return
            time.sleep(1)
        rospy.logerr("[SDK] Return Timeout.")

    def handle_reboot(self):
        kill_script = os.path.join(self.script_path, "kill_all.sh") 
        if os.path.exists(kill_script):
            subprocess.Popen(f"bash {kill_script}", shell=True)
            self.base_launched = False
            self.lio_launched = False
            self.imu_stable_start_time = None
            self.status_overall = 0
            self.launched_scripts.clear()
        else:
            rospy.logerr(f"[SCRIPT] Kill script not found: {kill_script}")

    def write_mission_to_yaml(self, wps):
        try:
            with open(self.yaml_path, 'w') as f:
                f.write("waypoints:\n")
                for wp in wps:
                    f.write(f"  - {{x: {wp['x']:.2f}, y: {wp['y']:.2f}, z: {wp['z']:.2f}, yaw: {wp['yaw']:.2f}, pointmode: \"{wp['mode']}\", dwell_time: {wp['time']:.1f}, gimbal_pitch: {wp['cp']:.1f}, cam_zoom: {wp['cz']:.1f}}}\n")
                
                f.write("land_height: 0.2\npx4_max_distance: 1.0\nmax_yaw_change: 0.2\n")
                f.write("threshould:\n  takeoff_threshould: 0.3\n  waypoint_threshould: 0.10\n")
                f.write("  aligning_threshould: 0.15\n  landing_threshould: 0.15\n")
                f.write("  arrive_yaw_threshould: 0.3\n  times_detect_threshould: 30\n")
                f.write("  waypoint_adjust_max_second_threshould: 5\n  land_adjust_max_second_threshould: 10\n")
                f.write("  planner_cmd_stale_threshould: 0.3\n  planner_min_pub_threshould: 0.02\n")
                f.write("switch:\n  flag_planner_px4: 0\n  flag_landing_detect: 0\n  auto_land: 1\n")
            return True
        except Exception as e:
            rospy.logerr(f"[SDK] YAML Write Error: {e}")
            return False

    def handle_waypoints(self, data):
        try:
            if len(data) < 2: return False
            count = struct.unpack('<H', data[0:2])[0]
            wps = []
            
            point_size = 28 
            ptr = 2
            
            for i in range(count):
                if ptr + point_size > len(data): break
                
                raw = data[ptr : ptr + point_size]
                vals = struct.unpack('<fffffff', raw)
                
                x, y, z, uav_yaw = vals[0], vals[1], vals[2], vals[3]
                cam_pitch = vals[4]
                cam_zoom  = vals[5]
                hover_time = vals[6]
                
                if i == 0: mode = "Takeoff_point"
                elif hover_time > 0: mode = "Detect_point"
                else: mode = "Nothing_point"

                wps.append({
                    'x': x, 'y': y, 'z': z, 'yaw': uav_yaw, 
                    'mode': mode, 'time': hover_time,
                    'cp': cam_pitch,
                    'cz': cam_zoom 
                })
                ptr += point_size

            wps.append({'x': 0.0, 'y': 0.0, 'z': 0.7, 'yaw': 0.0, 'mode': "Land_point", 'time':1.0, 'cp':0, 'cz':1})
            return self.write_mission_to_yaml(wps)

        except Exception as e:
            rospy.logerr(f"[SDK] Waypoint Parse Error: {e}")
            return False

    def timer_status_callback(self, event):
        if self.client_socket is None: return

        # 1. 坐标回传 (Protocol 0x01)
        try:
            payload = struct.pack('<fffff', 
                self.uav_pos[0], 
                self.uav_pos[1], 
                self.uav_pos[2], 
                self.uav_yaw,
                self.real_gimbal_pitch
            )
            self.send_packet(0x01, payload)
        except Exception as e: 
            pass

        # 2. 低频状态
        self.slow_sync_tick += 1
        if self.slow_sync_tick >= 10:
            self.slow_sync_tick = 0
            
            lidar_freq = self.clean_and_get_count(self.hist_lidar)
            self.status_lidar = 1 if (2 <= lidar_freq <= 20) else 0
            
            depth_freq = self.clean_and_get_count(self.hist_depth_d)
            self.status_depth = 1 if depth_freq > 2 else 0 
            
            csi_freq = self.clean_and_get_count(self.hist_csi)
            self.status_csi = 1 if csi_freq > 2 else 0

            odom_freq = self.clean_and_get_count(self.hist_odom)
            self.status_loc = 1 if (self.is_connected and odom_freq > 2 and self.odom_covariance < 0.2 and self.status_lidar == 1) else 0

            cpu_total = 0
            if HAS_PSUTIL:
                cpu_total = sum(psutil.cpu_percent(interval=None, percpu=True))
            self.status_comp = 1 if cpu_total < 500.0 else 0 

            self.status_overall = 1 if (self.status_loc == 1 and self.status_lidar == 1 and self.battery_percent > 30) else 0

            v = math.sqrt(self.uav_vel[0]**2 + self.uav_vel[1]**2 + self.uav_vel[2]**2)
            if self.current_mode in ["MANUAL", "POSCTL", "STABILIZED", "ALTCTL"]:
                self.flight_state = 1
            elif self.current_mode in ["OFFBOARD", "AUTO.MISSION"]:
                self.flight_state = 2 
            elif v < 0.1 and self.status_loc == 1:
                self.flight_state = 0 
            else:
                self.flight_state = 0 
            
            self.status_mapping = 1 if self.lio_launched else 0

            val_base = 1 if self.base_launched else 0
            
            try:
                self.send_packet(0x0D, struct.pack('<i', self.battery_percent))
                
                # 【修改】将 base_launched 放入第 3 个字节 (索引2)
                payload = struct.pack('BBBBBBBB', 
                    self.status_overall, 
                    self.status_lidar, 
                    val_base,          # <--- 【修改】现在这里代表 Base Algo 是否启动
                    self.status_depth, 
                    self.status_csi, 
                    self.status_loc,   # <--- 这里本身就是 status_loc
                    self.flight_state,
                    self.status_mapping 
                )
                self.send_packet(0x0E, payload)
            except: pass

if __name__ == '__main__':
    try:
        node = UAVBridgeSDK()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
