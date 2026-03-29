
import socket
from .core import *
from .utils import *
import time
from pynput import keyboard

class SIYISDK:
    def __init__(self, SERVER_IP, SERVER_PORT, BUFF_SIZE):
        # UDP初始化,ip,端口，字符长度
        self.SERVER_IP = SERVER_IP
        self.SERVER_PORT = SERVER_PORT
        self.send_addr = (self.SERVER_IP, self.SERVER_PORT)
        self.BUFF_SIZE = BUFF_SIZE
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    
    def send_receive_date(self,send_buf):
        """
            发送并接受数据
        """
        #print("Send HEX data")
        try:
            self.sock.sendto(send_buf, self.send_addr)
        except Exception as e:
            print(f"sendto error: {e}")
            return

        # 接收云台相机的返回数据
        try:
            recv_buf, recv_addr = self.sock.recvfrom(self.BUFF_SIZE)
            return recv_buf
        except Exception as e:
            print(f"recvfrom error: {e}")
            return
        
        


    def one_click_down(self):
        """ 
        一键朝下,直接旋转-90度
        """
        self.turn_to(0,-90)
        """
        #创建命令行类,计算send_buff
        cmd=CommandLine(CMD_ID=[0x0e],DATA=[0x00, 0x00, 0x3e, 0xfe])
        send_buf=cmd.create_send_buf()
        #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 

        # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        """

    def get_device_hardwareID(self):
        """
        获取云台相机硬件ID
        """
        cmd=CommandLine(CMD_ID=[0x02],DATA=[])
        #创建命令行类,计算send_buff
        send_buf=cmd.create_send_buf()
        
        #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        #解析接收到的数据
        cmd.recv_date_parser(recv_date);
        
        
        
    def get_device_workmode(self):
        """"
        获取云台相机工作模式
        """
        cmd=CommandLine(CMD_ID=[0x19],DATA=[])
        #创建命令行类,计算send_buff
        send_buf=cmd.create_send_buf()
        
        #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        #解析接收到的数据
        cmd.recv_date_parser(recv_date);

    
    def keep_turn(self):
        """
        控制云台持续转动
        """
        speed_yaw =50 #转动速度，默认为0，范围[0-100],0时静止
        speed_pitch=50
        turn=[0,0]#[yaw,pitch]
        last_turn = [0, 0]  # 记录上一次的 turn 状态
        running = True  # 用于控制主循环
         
        #让相机移动
        def camera_move():
            nonlocal last_turn, turn
            if turn != last_turn:
                last_turn = turn.copy()  # 更新 last_turn 的值
                #print(f"turn 发生变化: {turn}")
                #根据turn的值，发送控制信号
                ut =utils()
                turn_yaw_hex=ut.int_to_hex_array_uint8(turn[0])     #转换为16进制
                turn_pitch_hex=ut.int_to_hex_array_uint8(turn[1])
                cmd=CommandLine(CMD_ID=[0x07],DATA=turn_yaw_hex+turn_pitch_hex)
                #创建命令行类,计算send_buff
                send_buf=cmd.create_send_buf()
                #发送并接受数据
                recv_date=self.send_receive_date(send_buf) 


        # 处理按键按下的回调函数
        def on_press(key):
            nonlocal speed_yaw, speed_pitch,turn # 声明使用外部函数的局部变量
            try:

                if key ==keyboard.Key.up:#当按下↑ 
                    turn[1]= speed_pitch
                if key ==keyboard.Key.down:#当按下↓ 
                    turn[1]= speed_pitch*(-1)
                if key ==keyboard.Key.left:#当按下← 
                    turn[0]= speed_yaw*(-1)
                if key ==keyboard.Key.right:#当按下→
                    turn[0]= speed_yaw
                camera_move()
            except AttributeError:
                pass

        # 处理按键松开的回调函数
        def on_release(key):
            nonlocal speed_yaw, speed_pitch,running
            if key == keyboard.Key.esc:
                print("Esc 按键按下，退出监听")
                running = False  # 设置控制变量为 False 以终止主循环
                return False  # 返回 False 会自动停止监听器
                
            try:
                if key==keyboard.Key.up:
                    turn[1]=0
                if key ==keyboard.Key.down:
                    turn[1]=0
                if key ==keyboard.Key.left:
                    turn[0]=0
                if key ==keyboard.Key.right:
                    turn[0]=0
                camera_move()
                #使用WSAD加减速度，步长为10    
                if key.char =="w":
                    speed_pitch=speed_pitch+10
                    if(speed_pitch>100):
                        speed_pitch=100
                    if(speed_pitch<0):
                        speed_pitch=0
                    print(f"当前纵向速度：{speed_pitch}")
                if key.char =="s":
                    speed_pitch=speed_pitch-10
                    if(speed_pitch>100):
                        speed_pitch=100
                    if(speed_pitch<0):
                        speed_pitch=0
                    print(f"当前纵向速度：{speed_pitch}")
                if key.char =="a":
                    speed_yaw=speed_yaw-10
                    if(speed_yaw>100):
                        speed_yaw=100
                    if(speed_yaw<0):
                        speed_yaw=0
                    print(f"当前横向速度：{speed_yaw}")
                if key.char =="d":
                    speed_yaw=speed_yaw+10
                    if(speed_yaw>100):
                        speed_yaw=100
                    if(speed_yaw<0):
                        speed_yaw=0
                    print(f"当前横向速度：{speed_yaw}")
                
            except AttributeError:
                pass
            

        
        # 创建并启动监听器，非阻塞模式
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()  # 启动监听器（非阻塞）
        print("进入手动控制模式，↑↓←→控制摄像头移动，WSAD控制转动速度，ESC退出控制模式")
        #listener.join()  # 等待监听器停止
        # 使用控制变量保持主线程运行，避免退出
        while running:
            time.sleep(0.1)
            
            
    

   

    def one_click_back(self):
        """
        一键回中
        """
        cmd=CommandLine(CMD_ID=[0x08],DATA=[0x01])
        #创建命令行类,计算send_buff
        send_buf=cmd.create_send_buf()
        
        #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        
    def get_position(self):
        """
        获取云台姿态
        """
        cmd=CommandLine(CMD_ID=[0x0D],DATA=[])
        send_buf=cmd.create_send_buf()
         #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        cmd.recv_date_parser(recv_date)


    def get_attitude(self):
        """
        获取云台姿态数据，返回(yaw, pitch)，不打印任何信息
        返回: (yaw, pitch) 或 None（如果获取失败）
        """
        cmd = CommandLine(CMD_ID=[0x0D], DATA=[])
        send_buf = cmd.create_send_buf()
        
        # 发送并接收数据
        recv_data = self.send_receive_date(send_buf)
        
        if recv_data is None:
            return None
        
        try:
            # 解析数据
            datelength = int.from_bytes(recv_data[3:4], byteorder='little')
            date = recv_data[8:8+datelength]
            
            # 解析yaw和pitch（有符号16位整数，单位0.1度）
            yaw_raw = int.from_bytes(date[:2], byteorder='little', signed=True)
            pitch_raw = int.from_bytes(date[2:4], byteorder='little', signed=True)
            
            # 直接返回量化后的值（因为原始数据已经是0.1度的整数倍）
            # yaw_raw就是角度乘以10的值，所以直接除以10
            yaw = yaw_raw / 10.0
            pitch = pitch_raw / 10.0
            
            return yaw, pitch
            
        except Exception as e:
            # 出错时不打印，直接返回None
            return None
        
    
        
    def turn_to(self,yaw,pitch):
        """
        设置云台控制角度,精度为一位小数，支持yaw：-135.0至135.0；pitch：-90.0至25.0
        """
        if((-135<=yaw<=135)&(-90<=pitch<=25.0)):
            turn_yaw=int(yaw*10)     #取整
            turn_pitch=int(pitch*10)
            ut=utils()
            turn_yaw_hex=ut.int_to_hex_array_uint16(turn_yaw)     #转换为16进制
            turn_pitch_hex=ut.int_to_hex_array_uint16(turn_pitch)
            
            cmd=CommandLine(CMD_ID=[0x0e],DATA=turn_yaw_hex+turn_pitch_hex)
             #创建命令行类,计算send_buff
            send_buf=cmd.create_send_buf()
        
            #发送并接受数据
            recv_date=self.send_receive_date(send_buf) 
             # 十六进制形式打印接收到的数据
            print("Received HEX data: ", end="")
            print(" ".join(f"{byte:02x}" for byte in recv_date))
             #延迟一秒，等待动作完成
            time.sleep(0.2)#初始1.5
            
        else:
            print("设置角度不正确，支持yaw：-135.0至135.0；pitch：-90.0至25.0")
            
    def single_turn_to(self,angle,direction):
        """
        单轴角度控制，angle支持yaw：-135.0至135.0；pitch：-90.0至25.0；direction方向：0：yaw，1：pitch
        """
        if (direction==0):
            if(-135.0<angle<135.0):
                yaw=int(angle*10)
                ut=utils()
                yaw_hex=ut.int_to_hex_array_uint16(yaw)
                cmd=CommandLine(CMD_ID=[0x41],DATA=yaw_hex+[0x00])
                
                send_buf=cmd.create_send_buf()
                recv_date=self.send_receive_date(send_buf) 
                # 十六进制形式打印接收到的数据
                print("Received HEX data: ", end="")
                print(" ".join(f"{byte:02x}" for byte in recv_date))
                #延迟一秒，等待动作完成
                time.sleep(1.5)
            else:
                print("设置角度不正确")


        elif(direction==1):
            if(-90<angle<25):
                pitch=int(angle*10)
                ut=utils()
                pitch_hex=ut.int_to_hex_array_uint16(pitch)
                cmd=CommandLine(CMD_ID=[0x41],DATA=pitch_hex+[0x01])
                
                send_buf=cmd.create_send_buf()
                recv_date=self.send_receive_date(send_buf) 
                # 十六进制形式打印接收到的数据
                print("Received HEX data: ", end="")
                print(" ".join(f"{byte:02x}" for byte in recv_date))
                #延迟一秒，等待动作完成
                time.sleep(1.5)
            else:
                print("设置角度不正确")
        else:
            print("未知控制方向，请输入0或1")

    

    def get_config_info(self):
        """
        获取云台配置信息
        """
        cmd=CommandLine(CMD_ID=[0x0A],DATA=[])
        send_buf=cmd.create_send_buf()
         #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        #解析接收到的数据
        cmd.recv_date_parser(recv_date);
        

    def get_encode_info(self):
        """
         获取相机编码参数
        """
        cmd=CommandLine(CMD_ID=[0x20],DATA=[0x01])
        send_buf=cmd.create_send_buf()
         #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        #解析接收到的数据
        cmd.recv_date_parser(recv_date);
        
    def set_encode_info(self):
        """
        设置相机编码格式
        """
        "建议使用SIYI Assistance软件进行设置"
        pass

    def format_SDcard(self):
        """
        格式化SSD卡
        """
        cmd=CommandLine(CMD_ID=[0x48],DATA=[])
        send_buf=cmd.create_send_buf()
         #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))
        #解析接收到的数据
        cmd.recv_date_parser(recv_date);

        
    def device_restart(self,camera_restart,gimbal_restart):
        """
        相机或云台重启，0无动作；1重启
        """
        ut=utils()
        if((camera_restart==0)|(camera_restart==1)):
            camera_act=ut.int_to_hex_array_uint8(camera_restart)
        else:
            print("无效指令，请设定0（无动作）或1（重启）")
        
        if((gimbal_restart==0)|(gimbal_restart==1)):
            gimbal_act=ut.int_to_hex_array_uint8(gimbal_restart)
        else:
            print("无效指令，请设定0（无动作）或1（重启）")
        cmd=CommandLine(CMD_ID=[0x80],DATA=camera_act+gimbal_act)
        send_buf=cmd.create_send_buf()
         #发送并接受数据
        recv_date=self.send_receive_date(send_buf) 
          
         # 十六进制形式打印接收到的数据
        print("Received HEX data: ", end="")
        print(" ".join(f"{byte:02x}" for byte in recv_date))


    def close(self):
        # 关闭套接字
        self.sock.close()
        
