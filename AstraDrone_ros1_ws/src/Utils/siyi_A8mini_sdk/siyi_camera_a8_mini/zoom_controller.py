#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32
import socket
import struct
import threading
from datetime import datetime

class SiyiGimbalZoomController:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('siyi_gimbal_zoom_controller', anonymous=True)
        
        # 云台相机连接参数
        self.gimbal_ip = rospy.get_param('~gimbal_ip', '192.168.1.25')
        self.gimbal_port = rospy.get_param('~gimbal_port', 37260)
        
        # 变焦参数 - A8 mini 只有数码变焦，最大6倍
        self.min_zoom = 1  # 最小变焦倍数
        self.max_zoom = 6  # 最大变焦倍数（数码变焦）
        
        # 检查使用整数还是浮点数话题
        self.use_int_topic = rospy.get_param('~use_int_topic', False)
        
        # UDP连接
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 序列号计数器
        self.seq_counter = 0
        
        # 当前变焦状态
        self.current_zoom = 1
        
        # ROS订阅者
        if self.use_int_topic:
            rospy.Subscriber('zoom_value_int', Int32, self.zoom_int_callback)
            rospy.loginfo("使用整数变焦话题: /zoom_value_int")
        else:
            rospy.Subscriber('zoom_value', Float32, self.zoom_float_callback)
            rospy.loginfo("使用浮点数变焦话题: /zoom_value")
        
        # 可选：发布当前变焦状态
        self.zoom_status_pub = rospy.Publisher('zoom_status', Int32, queue_size=10)
        
        rospy.loginfo(f"思翼A8 mini云台变焦控制器已启动，IP: {self.gimbal_ip}:{self.gimbal_port}")
        rospy.loginfo(f"变焦范围: {self.min_zoom} - {self.max_zoom}倍（数码变焦）")
        
        # 初始化时获取当前变焦状态（可选）
        self.get_current_zoom()
        
        # 定时发布状态（可选）
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        
        # 保持节点运行
        rospy.spin()
        
        # 清理资源
        self.sock.close()
    
    def zoom_int_callback(self, msg):
        """处理ROS整数变焦值话题回调"""
        zoom_value = msg.data
        
        # 检查变焦值范围
        if zoom_value < self.min_zoom or zoom_value > self.max_zoom:
            rospy.logwarn(f"变焦值 {zoom_value} 超出范围 [{self.min_zoom}, {self.max_zoom}]")
            zoom_value = max(self.min_zoom, min(self.max_zoom, zoom_value))
        
        # 发送变焦命令
        success = self.send_zoom_command(zoom_value)
        
        if success:
            self.current_zoom = zoom_value
            rospy.loginfo(f"已设置变焦倍数: {zoom_value}倍")
        else:
            rospy.logerr(f"设置变焦倍数 {zoom_value}倍 失败")
    
    def zoom_float_callback(self, msg):
        """处理ROS浮点数变焦值话题回调"""
        zoom_value = msg.data
        
        # 转换为整数（四舍五入）
        zoom_int = int(round(zoom_value))
        
        # 检查变焦值范围
        if zoom_int < self.min_zoom or zoom_int > self.max_zoom:
            rospy.logwarn(f"变焦值 {zoom_value} (四舍五入为 {zoom_int}) 超出范围 [{self.min_zoom}, {self.max_zoom}]")
            zoom_int = max(self.min_zoom, min(self.max_zoom, zoom_int))
        
        # 发送变焦命令
        success = self.send_zoom_command(zoom_int)
        
        if success:
            self.current_zoom = zoom_int
            rospy.loginfo(f"已设置变焦倍数: {zoom_int}倍 (输入值: {zoom_value})")
        else:
            rospy.logerr(f"设置变焦倍数 {zoom_int}倍 失败")
    
    def send_zoom_command(self, zoom_value):
        """发送变焦命令到云台相机"""
        try:
            # 根据SDK文档，数码变焦可能需要使用不同的命令
            # 检查SDK文档中是否有专门的数码变焦命令
            
            # 方法1: 尝试使用绝对变倍命令 (0x0F)
            # 虽然文档中0x0F可能是为光学变焦设计的，但可能也支持数码变焦
            success = self.send_absolute_zoom_command(zoom_value)
            
            # 方法2: 如果0x0F不工作，尝试使用手动变倍命令 (0x05)
            # 或者尝试其他可能的变焦控制方式
            
            return success
            
        except Exception as e:
            rospy.logerr(f"发送变焦命令时出错: {e}")
            return False
    
    def send_absolute_zoom_command(self, zoom_value):
        """发送绝对变倍命令 (CMD_ID: 0x0F)"""
        try:
            # 拆分整数和小数部分
            # 对于整数变焦，小数部分为0
            zoom_int = zoom_value
            zoom_frac = 0
            
            # 确保在有效范围内
            if zoom_int < 1 or zoom_int > 30:
                rospy.logwarn(f"变焦值 {zoom_int} 超出0x0F命令的范围(1-30)，尝试发送...")
            
            # 构建数据包
            data_packet = self.build_zoom_packet(zoom_int, zoom_frac)
            
            # 发送到云台相机
            self.sock.sendto(data_packet, (self.gimbal_ip, self.gimbal_port))
            
            # 可选：等待并检查响应
            # return self.check_zoom_response()
            
            # 暂时假设发送成功
            return True
            
        except Exception as e:
            rospy.logerr(f"发送绝对变倍命令时出错: {e}")
            return False
    
    def build_zoom_packet(self, zoom_int, zoom_frac):
        """构建绝对变倍数据包 (CMD_ID: 0x0F)"""
        # 数据部分：整数部分 + 小数部分
        data_bytes = bytes([zoom_int, zoom_frac])
        data_len = len(data_bytes)
        
        # 递增序列号
        self.seq_counter = (self.seq_counter + 1) % 65536
        
        # 构建基本数据包（不含CRC）
        packet = bytearray()
        
        # STX (0x6655，低字节在前)
        packet.extend([0x55, 0x66])
        
        # CTRL (需要ACK)
        packet.append(0x01)
        
        # Data_len (低字节在前)
        packet.extend([data_len & 0xFF, (data_len >> 8) & 0xFF])
        
        # SEQ (低字节在前)
        packet.extend([self.seq_counter & 0xFF, (self.seq_counter >> 8) & 0xFF])
        
        # CMD_ID
        packet.append(0x0F)  # 绝对变倍命令
        
        # DATA
        packet.extend(data_bytes)
        
        # 计算CRC16
        crc = self.crc16_cal(packet)
        
        # 添加CRC16 (低字节在前)
        packet.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        rospy.logdebug(f"构建的变焦数据包: {packet.hex()}")
        return bytes(packet)
    
    def get_current_zoom(self):
        """获取当前变焦倍数 (CMD_ID: 0x18)"""
        try:
            # 构建获取当前变焦倍数的数据包
            packet = self.build_get_zoom_packet()
            
            # 发送到云台相机
            self.sock.sendto(packet, (self.gimbal_ip, self.gimbal_port))
            
            # 等待响应
            self.sock.settimeout(1.0)  # 设置超时时间
            response = self.sock.recv(1024)
            
            # 解析响应
            if self.parse_zoom_response(response):
                rospy.loginfo(f"当前变焦倍数: {self.current_zoom}倍")
                return True
            else:
                rospy.logwarn("获取当前变焦倍数失败")
                return False
                
        except socket.timeout:
            rospy.logwarn("获取当前变焦倍数超时")
            return False
        except Exception as e:
            rospy.logerr(f"获取当前变焦倍数时出错: {e}")
            return False
    
    def build_get_zoom_packet(self):
        """构建获取当前变焦倍数数据包 (CMD_ID: 0x18)"""
        # 递增序列号
        self.seq_counter = (self.seq_counter + 1) % 65536
        
        # 构建基本数据包（不含CRC）
        packet = bytearray()
        
        # STX (0x6655，低字节在前)
        packet.extend([0x55, 0x66])
        
        # CTRL (需要ACK)
        packet.append(0x01)
        
        # Data_len (低字节在前) - 0字节数据
        packet.extend([0x00, 0x00])
        
        # SEQ (低字节在前)
        packet.extend([self.seq_counter & 0xFF, (self.seq_counter >> 8) & 0xFF])
        
        # CMD_ID
        packet.append(0x18)  # 获取当前变焦倍数命令
        
        # 计算CRC16
        crc = self.crc16_cal(packet)
        
        # 添加CRC16 (低字节在前)
        packet.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        return bytes(packet)
    
    def parse_zoom_response(self, response):
        """解析变焦响应数据包"""
        if len(response) < 10:  # 最小响应包长度
            rospy.logwarn(f"响应包长度不足: {len(response)} 字节")
            return False
        
        # 检查STX
        if response[0] != 0x55 or response[1] != 0x66:
            rospy.logwarn(f"无效的STX: {response[0]:02x} {response[1]:02x}")
            return False
        
        # 解析数据长度
        data_len = response[3] | (response[4] << 8)
        
        # 检查命令ID
        cmd_id = response[7]
        
        # 如果是命令0x18的响应，解析变焦数据
        if cmd_id == 0x18 and data_len >= 2:
            zoom_int = response[8]
            zoom_frac = response[9]
            self.current_zoom = zoom_int + zoom_frac / 10.0
            rospy.logdebug(f"解析到变焦: {self.current_zoom}")
            return True
        # 如果是命令0x0F的ACK响应
        elif cmd_id == 0x0F:
            rospy.logdebug("收到绝对变倍命令的ACK响应")
            return True
        
        rospy.logwarn(f"未知的命令ID响应: {cmd_id:02x}")
        return False
    
    def publish_status(self, event):
        """定时发布当前变焦状态"""
        try:
            status_msg = Int32()
            status_msg.data = self.current_zoom
            self.zoom_status_pub.publish(status_msg)
        except Exception as e:
            rospy.logerr(f"发布变焦状态时出错: {e}")
    
    def crc16_cal(self, data, crc_init=0):
        """计算CRC16校验码"""
        crc16_tab = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ee, 0xf1ef,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
            0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
            0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
            0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
            0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
            0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
            0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
            0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
            0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
            0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
            0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
            0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
            0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
            0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
            0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
            0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
            0xcb7d, 0xdb5c, 0xeb3f, 0xfbfe, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
            0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
            0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
            0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
            0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
            0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
        ]
        
        crc = crc_init
        for byte in data:
            temp = (crc >> 8) & 0xFF
            crc = ((crc << 8) & 0xFFFF) ^ crc16_tab[byte ^ temp]
        
        return crc


if __name__ == '__main__':
    try:
        controller = SiyiGimbalZoomController()
    except rospy.ROSInterruptException:
        pass
