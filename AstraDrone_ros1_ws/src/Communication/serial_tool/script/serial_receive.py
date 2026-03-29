#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Pose

# 配置串口
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# 接收端节点
def receiver_node():
    rospy.init_node('serial_receiver', anonymous=True)
    pub = rospy.Publisher('/received_pose', Pose, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            # 数据包大小：6个float64 + 1个校验位
            packet_size = 48 + 1
            packet = ser.read(packet_size)

            if len(packet) == packet_size:
                data = packet[:-1]
                checksum = packet[-1]
                calculated_checksum = sum(data) % 256

                if checksum == calculated_checksum:
                    # 解包数据
                    values = struct.unpack('6d', data)
                    pose_msg = Pose()
                    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = values[0:3]
                    pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = values[3:6]
                    pose_msg.orientation.w = 1.0  # 默认设为1.0，后续可优化

                    pub.publish(pose_msg)
                    rospy.loginfo(f"Received data: {values}, checksum: {checksum}")
                else:
                    rospy.logwarn("Checksum mismatch!")
        except Exception as e:
            rospy.logerr(f"Failed to receive data: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        receiver_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
