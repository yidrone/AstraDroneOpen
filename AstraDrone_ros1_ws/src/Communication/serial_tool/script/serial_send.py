#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Pose

# 配置串口
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# 发送端节点
def sender_node():
    rospy.init_node('serial_sender', anonymous=True)
    rospy.Subscriber('/pose_topic', Pose, pose_callback)
    rospy.spin()

# 发送校验和计算
def calculate_checksum(data):
    return sum(data) % 256

# 发送回调函数
def pose_callback(msg):
    try:
        # 数据打包：'6d' 表示 6 个 float64
        data = struct.pack('6d', msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        checksum = calculate_checksum(data)
        
        # 构造完整数据包：前导码 00 02 17 + 数据 + 校验和
        packet = bytes([0x00, 0x02, 0x17]) + data + struct.pack('B', checksum)
        
        ser.write(packet)
        rospy.loginfo(f"Sent data: {msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, checksum: {checksum}")
    except Exception as e:
        rospy.logerr(f"Failed to send data: {e}")

if __name__ == '__main__':
    try:
        sender_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
