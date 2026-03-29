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
    
   
    def send_custom_command(self, cmd):
        """发送回中命令"""
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
    MODE_NORMAL_CMD = "#TPUG2wPTZ0A7B" #吊装模式命令
    MODE_REVERSE_CMD = "#TPUG2wPTZ0B7C" #倒装模式命令
    UP_CMD = "#TPUG2wPTZ016BE3" #一键朝上看命令
    DOWN_CMD = "#TPUG2wPTZ026CE5" #一键朝下看命令


    controller = GimbalController(HOST, PORT)
    
    # 连接云台
    if not controller.connect():
        return

    try:
        controller.send_custom_command(MODE_NORMAL_CMD)

    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()