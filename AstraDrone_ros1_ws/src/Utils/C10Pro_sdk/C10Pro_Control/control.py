#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time

import rospy
from geometry_msgs.msg import Point


class GimbalController:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False

    def connect(self):
        """连接到云台（UDP）"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(5)  # 5秒超时
            self.connected = True
            rospy.loginfo("GimbalController: 已创建 UDP socket")
            return True
        except Exception as e:
            rospy.logerr("GimbalController: 连接失败: %s", e)
            self.connected = False
            return False

    def disconnect(self):
        """断开连接"""
        if self.socket:
            self.socket.close()
            self.connected = False
            rospy.loginfo("GimbalController: 已断开连接")

    def calculate_crc(self, cmd):
        """计算校验码（ASCII 累加和，低 8 位）"""
        crc = 0
        for char in cmd:
            crc = (crc + ord(char)) & 0xFF  # 确保是8位
        return crc

    def angle_to_hex(self, angle):
        """将角度转换为16位有符号十六进制字符串（大端）"""
        # 角度乘以100，因为单位是0.01度
        value = int(angle * 100)

        # 确保值在16位有符号范围内
        if value > 32767:
            value = 32767
        elif value < -32768:
            value = -32768

        # >h 表示大端16位有符号整数
        value_bytes = struct.pack('>h', value)
        hex_str = value_bytes.hex().upper()
        return hex_str

    def speed_to_hex(self, speed):
        """将速度转换为8位无符号十六进制字符串"""
        # 速度乘以10，因为单位是0.1度/秒（按你之前脚本的设定）
        value = int(speed * 10)

        # 确保值在8位无符号范围内
        if value > 255:
            value = 255
        elif value < 0:
            value = 0

        value_bytes = struct.pack('B', value)  # B 表示8位无符号整数
        hex_str = value_bytes.hex().upper()
        return hex_str

    def generate_command(self, axis, angle, speed=5.0):
        """生成控制命令字符串（单轴 GAY/GAP）"""
        # 固定前缀：#TP + UG + 数据长度(6) + w
        prefix = "#TPUG6w"

        axis_cmd = {
            "yaw": "GAY",
            "pitch": "GAP",
        }.get(axis.lower(), "GAY")  # 默认为 yaw

        angle_hex = self.angle_to_hex(angle)
        speed_hex = self.speed_to_hex(speed)

        # 不含 CRC 的命令
        cmd_without_crc = f"{prefix}{axis_cmd}{angle_hex}{speed_hex}"

        # 计算 CRC
        crc = self.calculate_crc(cmd_without_crc)
        crc_hex = f"{crc:02X}"

        full_cmd = f"{cmd_without_crc}{crc_hex}"
        return full_cmd

    def send_angle_command(self, axis, angle, speed=5.0):
        """发送角度控制命令到云台"""
        if not self.connected:
            rospy.logwarn("GimbalController: 未连接，忽略命令")
            return False

        try:
            command = self.generate_command(axis, angle, speed)
            self.socket.sendto(command.encode('utf-8'), (self.host, self.port))
            rospy.logdebug("GimbalController: 发送命令 [%s]", command)
            return True
        except Exception as e:
            rospy.logerr("GimbalController: 发送命令失败: %s", e)
            self.connected = False
            return False

    def send_custom_command(self, cmd):
        """发送自定义命令（例如 PTZ 回中等）"""
        if not self.connected:
            rospy.logwarn("GimbalController: 未连接，忽略自定义命令")
            return False

        try:
            self.socket.sendto(cmd.encode('utf-8'), (self.host, self.port))
            rospy.logdebug("GimbalController: 发送自定义命令 [%s]", cmd)
            return True
        except Exception as e:
            rospy.logerr("GimbalController: 发送自定义命令失败: %s", e)
            self.connected = False
            return False


class GimbalAngleNode:
    """
    ROS 节点：
        订阅 /gimbal/cmd_angle [geometry_msgs/Point]
        msg.x = pitch  (俯仰)
        msg.y = yaw    (航向)
        msg.z = 未用（保留）

    收到消息后，分别发送 pitch/yaw 的角度控制命令。
    如果在一段时间内没有收到消息，则自动回中(0,0)，并保持回中状态。
    """

    def __init__(self):
        # 从参数服务器读取 IP 和端口（可在 launch 里覆盖）
        host = rospy.get_param("~gimbal_ip", "192.168.144.108")
        port = rospy.get_param("~gimbal_port", 5000)
        speed = rospy.get_param("~gimbal_speed", 5.0)   # 度/秒
        # 未收到命令的超时时间（秒），超过该时间则自动回中
        self.timeout_sec = rospy.get_param("~cmd_timeout", 20.0)

        self.speed = speed
        self.controller = GimbalController(host, port)

        if not self.controller.connect():
            rospy.logerr("GimbalAngleNode: 连接云台失败，节点仍会继续运行，但不会真正控制云台")

        # 订阅话题 /gimbal/cmd_angle
        self.sub = rospy.Subscriber("/gimbal/cmd_angle", Point,
                                    self.cmd_angle_callback,
                                    queue_size=1)

        # 上一次收到有效指令的时间。
        # 初始设为 0，使得启动后如果一直没有话题，也会触发回中。
        self.last_cmd_time = 0.0

        # 定时器：定期检查是否超时没收到指令
        self.timer = rospy.Timer(rospy.Duration(0.5), self.watchdog_callback)

        rospy.loginfo("GimbalAngleNode: 已启动，订阅 /gimbal/cmd_angle")
        rospy.loginfo("GimbalAngleNode: 云台 IP: %s, 端口: %d, 速度: %.2f deg/s, 超时: %.2f s",
                      host, port, self.speed, self.timeout_sec)

    def cmd_angle_callback(self, msg: Point):
        """
        回调函数：
        msg.x -> pitch
        msg.y -> yaw
        """
        self.last_cmd_time = time.time()   # 更新最后一次收到指令的时间

        pitch = msg.x
        yaw = msg.y

        # 保护：限制角度在 [-90, 90]
        if pitch > 90.0:
            pitch = 90.0
        if pitch < -90.0:
            pitch = -90.0
        if yaw > 90.0:
            yaw = 90.0
        if yaw < -90.0:
            yaw = -90.0

        rospy.loginfo("GimbalAngleNode: 收到角度指令 pitch=%.2f, yaw=%.2f", pitch, yaw)

        # 先发 yaw 再发 pitch（或者反过来也行，看你习惯）
        ok_yaw = self.controller.send_angle_command("yaw", yaw, self.speed)
        time.sleep(0.01)  # 稍微间隔一下，避免粘在一起
        ok_pitch = self.controller.send_angle_command("pitch", pitch, self.speed)

        if not (ok_yaw and ok_pitch):
            rospy.logwarn("GimbalAngleNode: 发送角度命令失败（可能断线）")

    def watchdog_callback(self, event):
        """
        定时器回调：
        如果超过 timeout_sec 没有收到新的角度指令，则自动回中。
        """
        if not self.controller.connected:
            return

        now = time.time()
        # last_cmd_time 初始为 0，启动后一旦没话题就会触发这里
        if (self.last_cmd_time == 0.0) or (now - self.last_cmd_time > self.timeout_sec):
            # 自动回中
            rospy.logwarn_throttle(2.0, "GimbalAngleNode: 超时未收到话题，自动回中 (0,0)")
            self.controller.send_angle_command("yaw", 0.0, self.speed)
            time.sleep(0.01)
            self.controller.send_angle_command("pitch", 0.0, self.speed)

    def shutdown(self):
        self.controller.disconnect()


def main():
    rospy.init_node("gimbal_angle_node")
    node = GimbalAngleNode()

    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()

