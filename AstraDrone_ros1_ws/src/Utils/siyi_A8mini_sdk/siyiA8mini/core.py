import inspect
from .crc16 import CRC16

class CommandLine:
    def __init__(self, STX=[0x55, 0x66], CTRL=[0x01], SEQ=[0x00,0x00], CMD_ID=[], DATA=[]):
        # send_buff构造
        self.STX = STX                                  #起始标志
        self.CTRL = CTRL                                #数据包是否需要ack，0需要，1不需要
        self.DATA=DATA                                  #发送的数据
        self.Data_len= self.calculate_data_len()        #要发送的数据字节长度
        self.SEQ = SEQ                                  #帧序列，范围（0-65535）
        self.CMD_ID = CMD_ID                            #命令ID
        
        self.CRC16=self.calculate_crc16()               #CRC校验码
        

    def calculate_data_len(self):
        #计算data_len
        data_len = len(self.DATA)
        return [(data_len & 0xFF), (data_len >> 8) & 0xFF]



    def calculate_crc16(self):
        #计算CRC16
        data_to_crc = self.STX+self.CTRL+self.Data_len+self.SEQ+self.CMD_ID+self.DATA
        crc_calculator = CRC16()
        crc = crc_calculator.calculate(data_to_crc)  #  CRC16 计算逻辑
        return crc
    
    def create_send_buf(self):
        #构造send_buff
        return bytes(self.STX) + bytes(self.CTRL) + bytes(self.Data_len) +bytes(self.SEQ)+ bytes(self.CMD_ID)+bytes(self.DATA) + bytes(self.CRC16)
    
    def recv_date_parser(self,recv_data):
        #根据接收到的数据，解析为对应值,根据调用函数不同，自动对应解析过程       
        caller_function_name = inspect.stack()[1].function #调用此函数的函数名称
        datelength=int.from_bytes(recv_data[3:4], byteorder='little') #计算数据长度
        date=recv_data[8:8+datelength] #获取数据段
        
        if(caller_function_name=="get_device_hardwareID"):
            ID_dict={0x6B:"ZR10",
                     0x73:"A8 mini",
                     0x75:"A2 mini",
                     0x78:"ZR30",
                     0x82:"ZT6",
                     0x7A:"ZT30"
                     }
            hex_value = int(date[:2].decode(), 16)
            hardware_ID=ID_dict.get(hex(hex_value),"未知设备")
            print(f"当前设备: {hardware_ID}")
            
        elif(caller_function_name=="get_device_workmode"):
            workmode_dict={0x00:"锁定模式",
                           0x01:"跟随模式",
                           0x02:"FPV模式"}
            # 根据模式字节查找对应的模式
            
            mode = workmode_dict.get(int.from_bytes(date, byteorder='little'), "未知模式")
            print(f"当前模式: {mode}")
             
        elif(caller_function_name=="get_position"):
            yaw=int.from_bytes(date[:2], byteorder='little')/10
            pitch=int.from_bytes(date[2:4], byteorder='little')/10
            roll=int.from_bytes(date[4:6], byteorder='little')/10
            yaw_velocity=int.from_bytes(date[6:8], byteorder='little')/10
            pitch_velocity=int.from_bytes(date[8:10], byteorder='little')/10
            roll_velocity=int.from_bytes(date[8:10], byteorder='little')/10
            print(f"偏航角:{yaw};  俯仰角:{pitch}:  横滚角：{roll};  偏航角速度:{yaw_velocity};  俯仰角速度:{pitch_velocity};  横滚角速度:{roll_velocity}")

        elif(caller_function_name=="get_config_info"):
            hdr_dict={0x00:"关闭",0x01:"开启"}
            record_dict={0x00:"未开启录像",
                         0x01:"已开启录像",
                         0x02:"未插入TF卡",
                         0x03:"TF卡录制视频丢失，请检查IF卡"}
            motion_mode_dict={0x00:"锁定模式",
                              0x01:"跟随模式",
                              0x02:"FPV模式"}
            mounting_dir_dict={0x00:"reseve",
                               0x01:"正常",
                               0x02:"倒置"}
            video_mode_dict={0x00:"HDMI输出",
                             0x01:"CVBS输出"}
            
            hdr=hdr_dict.get(int.from_bytes(date[1:2], byteorder='little'), "未知hdr模式")
            record=record_dict.get(int.from_bytes(date[3:4], byteorder='little'), "未知录像状态")
            motion=motion_mode_dict.get(int.from_bytes(date[4:5], byteorder='little'), "未知云台模式")
            mounting=mounting_dir_dict.get(int.from_bytes(date[5:6], byteorder='little'), "未知云台安装姿态")
            video=video_mode_dict.get(int.from_bytes(date[6:7], byteorder='little'), "未知视频输出模式")
            print(f"hdr模式：{hdr};  录像状态：{record};  云台模式：{motion};  云台安装姿态: {mounting}; 视频输出模式：{video}")

        elif(caller_function_name=="get_encode_info"):
            stream_type_dict={0x00:"录像流",
                              0x01:"主码流",
                              0x02:"副码流"}
            video_encode_type_dict={0x01:"H264",
                                    0x02:"H265"}
            stream=stream_type_dict.get(int.from_bytes(date[0:1], byteorder='little'), "未知码流")
            video_enc_type=video_encode_type_dict.get(int.from_bytes(date[1:2], byteorder='little'), "未知编码格式")
            resolution_L=int.from_bytes(date[2:4], byteorder='little')
            resolution_H=int.from_bytes(date[4:6], byteorder='little')
            video_bit_rate=int.from_bytes(date[6:8], byteorder='little')
            video_frame_rate=int.from_bytes(date[8:9], byteorder='little')
            print(f"当前码流：{stream};  编码格式：{video_enc_type};  分辨率宽：{resolution_L};  分辨率高: {resolution_H}; 视频码率Kbps：{video_bit_rate};  视频帧率：{video_frame_rate}")
        
        elif(caller_function_name=="format_SDcard"):  
            format_state_dict={0x00:"格式化失败",
                               0x01:"格式化成功"}
            format_s=format_state_dict.get(int.from_bytes(date[0:1], byteorder='little'), "未知格式化状态")
            print(f"格式化状态：{format_s}")
            
        else:pass

  


 
