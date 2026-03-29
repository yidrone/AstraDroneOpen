import socket
import struct
import time

class GimbalController:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect(self):
        """连接到云台"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(5)  # 5秒超时
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            self.socket.close()
            self.connected = False
            print("已断开连接")
    
    def calculate_crc(self, cmd):
        """计算校验码"""
        crc = 0
        for char in cmd:
            crc = (crc + ord(char)) & 0xFF  # 确保是8位
        return crc
    
    def angle_to_hex(self, angle):
        """将角度转换为16位有符号十六进制字符串（小端序）"""
        # 角度乘以100，因为单位是0.01度
        value = int(angle * 100)
        
        # 确保值在16位有符号范围内
        if value > 32767:
            value = 32767
        elif value < -32768:
            value = -32768
            
        # 转换为16位有符号整数，然后转换为小端序的十六进制
        value_bytes = struct.pack('>h', value)  # >h 表示大端16位有符号整数
        hex_str = value_bytes.hex().upper()
        return hex_str
    
    def speed_to_hex(self, speed):
        """将速度转换为16位无符号十六进制字符串（小端序）"""
        # 速度乘以10，因为单位是0.1度/秒
        value = int(speed * 10)
        # 确保值在8位无符号范围内
        if value > 255:
            value = 255
        elif value < 0:
            value = 0

        # 转换为8位无符号整数，然后转换为十六进制
        value_bytes = struct.pack('B', value)  # B 表示8位无符号整数
        hex_str = value_bytes.hex().upper()
        return hex_str
    
    def generate_command(self, axis, angle, speed=5.0):
        """生成控制命令字符串"""
        # 固定前缀
        prefix = "#TPUG6w"
        
        # 根据轴选择命令
        axis_cmd = {
            "yaw": "GAY",
            "pitch": "GAP", 
        }.get(axis.lower(), "GAY")  # 默认为yaw

        # 转换角度和速度
        angle_hex = self.angle_to_hex(angle)
        speed_hex = self.speed_to_hex(speed)

        # 构建命令（不含校验码）
        cmd_without_crc = f"{prefix}{axis_cmd}{angle_hex}{speed_hex}"

        # 计算校验码
        crc = self.calculate_crc(cmd_without_crc)
        crc_hex = f"{crc:02X}"

        # 完整命令
        full_cmd = f"{cmd_without_crc}{crc_hex}"

        return full_cmd
    
    def send_angle_command(self, axis, angle, speed=5.0):
        """发送控制命令到云台"""
        try:
            command = self.generate_command(axis, angle, speed)
            # 将命令转换为字节并发送
            self.socket.sendto(command.encode('utf-8'), (self.host, self.port))
            print(f"发送命令成功")
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            self.connected = False
            return False
    
    def send_custom_command(self, cmd):
        """发送自定义命令"""
        try:
            self.socket.sendto(cmd.encode('utf-8'), (self.host, self.port))
            print("发送命令成功")
            return True
        except Exception as e:
            print(f"发送回中命令失败: {e}")
            self.connected = False
            return False

def main():
    # 云台连接信息 - 请根据实际情况修改
    HOST = "192.168.144.108"  # 云台IP地址
    PORT = 5000             # 云台端口
    RECOVER_CMD = "#TPUG2wPTZ056FEB"  # 回中命令
    MODE_NORMAL_CMD = "#TPUG2wPTZ0A7B" #吊装模式命令
    MODE_REVERSE_CMD = "#TPUG2wPTZ0B7C" #倒装模式命令

    controller = GimbalController(HOST, PORT)
    
    # 连接云台
    if not controller.connect():
        return
    
    try:
        while True:
            print("\n" + "="*50)
            print("云卓云台角度控制程序")
            print("="*50)
            print("1. 控制YAW轴")
            print("2. 控制PITCH轴") 
            print("3. 回中")
            print("4. 吊装模式")
            print("5. 倒装模式")
            print("6. 退出")
            
            choice = input("请选择操作 (1-6): ").strip()
            
            if choice == "1":
                axis = "yaw"
            elif choice == "2":
                axis = "pitch"
            elif choice == "3":
                controller.send_custom_command(RECOVER_CMD)
                time.sleep(0.5)
                continue
            elif choice == "4":
                controller.send_custom_command(MODE_NORMAL_CMD)
                time.sleep(0.5)
                continue
            elif choice == "5":
                controller.send_custom_command(MODE_REVERSE_CMD)
                time.sleep(0.5)
                continue
            elif choice == "6":
                break
            else:
                print("无效选择，请重新输入")
                continue
            
            if choice in ["1", "2"]:
                try:
                    angle = float(input(f"请输入{axis}轴目标角度 (范围:[-90,90]): "))
                    speed = 5.0  # 固定速度5度/秒
                    # 发送控制命令
                    controller.send_angle_command(axis, angle, speed)
                    # 等待命令执行
                    time.sleep(0.5) 
                except ValueError:
                    print("请输入有效的数字")
                except Exception as e:
                    print(f"控制失败: {e}")
    
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()