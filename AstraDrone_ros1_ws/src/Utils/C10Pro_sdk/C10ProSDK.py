# -*- coding: utf-8 -*-
"""
C10ProSDK - 云卓 C10 Pro 云台相机 Python SDK（兼容 Python 3.8）

放置方式示例：
~/AstraDroneOpen/AstraDrone_ros1_ws/src/Utils/C10Pro_sdk/
    ├── C10ProSDK.py
    └── test.py   (from C10ProSDK import C10ProSDK)
"""

import socket
import time
from typing import Optional

try:
    # 可选依赖，用于键盘手动控制
    from pynput import keyboard
    _HAS_PYNPUT = True
except ImportError:
    _HAS_PYNPUT = False


class C10ProSDK:
    """
    云卓 C10 Pro 云台相机 Python SDK（UDP 协议）

    - 默认使用 UDP 9002 端口
    - 协议格式：
        #TP + 地址(2) + 长度(1) + 控制位(w/r) + 标识位(3) + Data + CRC(2)
      其中地址通常为：U + G/D/M/E，例如 UG、UD
      CRC 为前面所有字符的 ASCII 累加和，取低 8 位，再转两位 16 进制 ASCII。
    """

    def __init__(self, server_ip: str, server_port: int = 9002,
                 buff_size: int = 1024, timeout: float = 1.0) -> None:
        """
        :param server_ip: 云台 IP 地址
        :param server_port: UDP 端口（C10 Pro 默认 9002）
        :param buff_size: 接收缓冲区大小
        :param timeout: socket 接收超时时间（秒）
        """
        self.SERVER_IP = server_ip
        self.SERVER_PORT = server_port
        self.BUFF_SIZE = buff_size

        self.send_addr = (self.SERVER_IP, self.SERVER_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    # ====================  基础工具函数  ====================

    @staticmethod
    def _calc_crc(frame_without_crc: str) -> str:
        """
        按协议计算 CRC：
        将字符串的 ASCII 做累加和，取低 8 位，转两位 16 进制字符串（大写）。
        """
        crc_val = sum(frame_without_crc.encode("ascii")) & 0xFF
        return "{:02X}".format(crc_val)

    @staticmethod
    def _encode_signed_byte(v: int) -> str:
        """
        有符号 8 位整数 -> 2 位 HEX 字符串，范围 -128~127
        用于速度控制（GSY/GSP/GSM）
        """
        if v < -128 or v > 127:
            raise ValueError("signed byte out of range [-128, 127]")
        if v < 0:
            v = (1 << 8) + v  # 转成补码
        return "{:02X}".format(v)

    @staticmethod
    def _encode_angle_0_01deg(angle_deg: float) -> str:
        """
        将角度（度）转换成以 0.01 度为单位的 16 位有符号数，再转为 4 位 HEX 字符串。
        例如：
            -50.00° -> -5000 -> 0xEC78 -> 'EC78'
        """
        val = int(round(angle_deg * 100.0))
        if val < -32768 or val > 32767:
            raise ValueError("angle out of range for 16-bit signed")
        if val < 0:
            val = (1 << 16) + val
        return "{:04X}".format(val)

    @staticmethod
    def _encode_u8(v: int) -> str:
        """
        无符号 8 位整数 -> 2 位 HEX 字符串，范围 0~255。
        常用于速度（0~99）、频率（1~100）、图像参数等。
        """
        if v < 0 or v > 255:
            raise ValueError("u8 out of range [0, 255]")
        return "{:02X}".format(v)

    def _build_frame(self, header: str, dst: str,
                     control: str, mark: str, data: str) -> str:
        """
        组装协议帧。

        :param header: '#TP' 或 '#tp'（大小写取决于文档要求）
        :param dst: 目标地址单字符：'G', 'D', 'M', 'E'
        :param control: 'w'（写/设置）或 'r'（读/查询）
        :param mark: 标识位，3 字符，如 'PTZ' / 'GSY' / 'GAM' 等
        :param data: 数据区字符串（通常为 HEX 字符或可打印 ASCII）
        """
        if len(header) != 3:
            raise ValueError("header must be '#TP' or '#tp'")
        if len(dst) != 1:
            raise ValueError("dst must be single char, e.g. 'G'")
        if len(mark) != 3:
            raise ValueError("mark must be 3 chars, e.g. 'PTZ'")

        # 地址：外部控制模块 U + 目标地址 G/D/M/E
        addr = "U{}".format(dst)

        # 数据位字符数（HEX 0~F），这里按“字符个数”计
        length_char = "{:X}".format(len(data))[-1].upper()

        frame_wo_crc = "{}{}{}{}{}{}".format(
            header, addr, length_char, control, mark, data
        )
        crc = self._calc_crc(frame_wo_crc)
        return frame_wo_crc + crc

    def _send_and_recv(self, frame: str) -> Optional[str]:
        """
        发送命令并接收响应，返回响应字符串（ASCII）
        如超时或异常，返回 None。
        """
        try:
            self.sock.sendto(frame.encode("ascii"), self.send_addr)
            # 调试：查看发送内容
            # print("[C10ProSDK] send:", frame)
        except Exception as e:
            print("[C10ProSDK] send error:", e)
            return None

        try:
            recv_buf, _ = self.sock.recvfrom(self.BUFF_SIZE)
            resp = recv_buf.decode("ascii", errors="ignore")
            # 调试：查看接收内容
            # print("[C10ProSDK] recv:", resp)
            return resp
        except Exception as e:
            print("[C10ProSDK] recv error/timeout:", e)
            return None

    # ====================  云台 PTZ 控制 (PTZ)  ====================

    def _ptz_raw(self, code_hex: str) -> Optional[str]:
        """
        发送 PTZ 原始控制码（00~14），见协议 PTZ 命令表。
        """
        code = int(code_hex, 16)
        data = "{:02X}".format(code)
        frame = self._build_frame("#TP", "G", "w", "PTZ", data)
        return self._send_and_recv(frame)

    def stop(self) -> Optional[str]:
        """云台停止（PTZ 00）"""
        return self._ptz_raw("00")

    def move_up(self) -> Optional[str]:
        """云台向上（PTZ 01）"""
        return self._ptz_raw("01")

    def move_down(self) -> Optional[str]:
        """云台向下（PTZ 02）"""
        return self._ptz_raw("02")

    def move_left(self) -> Optional[str]:
        """云台向左（PTZ 03）"""
        return self._ptz_raw("03")

    def move_right(self) -> Optional[str]:
        """云台向右（PTZ 04）"""
        return self._ptz_raw("04")

    def one_click_back(self) -> Optional[str]:
        """一键回中（PTZ 05）"""
        return self._ptz_raw("05")

    def set_follow_mode(self) -> Optional[str]:
        """跟随模式（PTZ 06）"""
        return self._ptz_raw("06")

    def set_lock_mode(self) -> Optional[str]:
        """锁头模式（PTZ 07）"""
        return self._ptz_raw("07")

    def toggle_follow_lock(self) -> Optional[str]:
        """跟随/锁头模式切换（PTZ 08）"""
        return self._ptz_raw("08")

    def gimbal_calibrate(self) -> Optional[str]:
        """云台校准（PTZ 09）"""
        return self._ptz_raw("09")

    def gimbal_mount_upside(self) -> Optional[str]:
        """云台吊装模式（PTZ 0A）"""
        return self._ptz_raw("0A")

    def gimbal_mount_invert(self) -> Optional[str]:
        """云台倒装模式（PTZ 0B）"""
        return self._ptz_raw("0B")

    def horizontal_calibrate(self) -> Optional[str]:
        """水平校准（PTZ 0C）"""
        return self._ptz_raw("0C")

    def vertical_calibrate(self) -> Optional[str]:
        """垂直校准（PTZ 0D）"""
        return self._ptz_raw("0D")

    # ====================  云台速度模式 (GSY / GSP / GSM)  ====================

    def set_speed_yaw(self, speed: int) -> Optional[str]:
        """
        航向轴速度控制（GSY）
        :param speed: -127~127，数值越大速度越快，符号表示方向
        """
        speed = max(-127, min(127, speed))
        data = self._encode_signed_byte(speed)
        frame = self._build_frame("#TP", "G", "w", "GSY", data)
        return self._send_and_recv(frame)

    def set_speed_pitch(self, speed: int) -> Optional[str]:
        """
        俯仰轴速度控制（GSP）
        :param speed: -127~127
        """
        speed = max(-127, min(127, speed))
        data = self._encode_signed_byte(speed)
        frame = self._build_frame("#TP", "G", "w", "GSP", data)
        return self._send_and_recv(frame)

    def set_speed_yaw_pitch(self, yaw_speed: int, pitch_speed: int) -> Optional[str]:
        """
        航向 + 俯仰联合速度控制（GSM）
        :param yaw_speed: -127~127
        :param pitch_speed: -127~127
        """
        yaw_speed = max(-127, min(127, yaw_speed))
        pitch_speed = max(-127, min(127, pitch_speed))
        data = self._encode_signed_byte(yaw_speed) + self._encode_signed_byte(pitch_speed)
        frame = self._build_frame("#TP", "G", "w", "GSM", data)
        return self._send_and_recv(frame)

    # ====================  云台角度模式 (GAY / GAP / GAM)  ====================

    def set_yaw_angle(self, angle_deg: float, speed: int = 30) -> Optional[str]:
        """
        单轴航向角度控制（GAY）
        :param angle_deg: -90~+90（度）
        :param speed: 0~99（协议中为步进速度）
        """
        angle_deg = max(-90.0, min(90.0, angle_deg))
        speed = max(0, min(99, speed))
        angle_hex = self._encode_angle_0_01deg(angle_deg)
        speed_hex = self._encode_u8(speed)
        data = angle_hex + speed_hex
        frame = self._build_frame("#TP", "G", "w", "GAY", data)
        return self._send_and_recv(frame)

    def set_pitch_angle(self, angle_deg: float, speed: int = 30) -> Optional[str]:
        """
        单轴俯仰角度控制（GAP）
        :param angle_deg: -90~+90（度）
        :param speed: 0~99
        """
        angle_deg = max(-90.0, min(90.0, angle_deg))
        speed = max(0, min(99, speed))
        angle_hex = self._encode_angle_0_01deg(angle_deg)
        speed_hex = self._encode_u8(speed)
        data = angle_hex + speed_hex
        frame = self._build_frame("#TP", "G", "w", "GAP", data)
        return self._send_and_recv(frame)

    def turn_to(self, yaw_deg: float, pitch_deg: float,
                yaw_speed: int = 30, pitch_speed: int = 30) -> Optional[str]:
        """
        云台角度控制（GAM），类似你原来的 turn_to 接口：
        :param yaw_deg: -90~+90
        :param pitch_deg: -90~+90
        :param yaw_speed: 0~99
        :param pitch_speed: 0~99
        """
        yaw_deg = max(-90.0, min(90.0, yaw_deg))
        pitch_deg = max(-90.0, min(90.0, pitch_deg))
        yaw_speed = max(0, min(99, yaw_speed))
        pitch_speed = max(0, min(99, pitch_speed))

        yaw_angle_hex = self._encode_angle_0_01deg(yaw_deg)
        yaw_speed_hex = self._encode_u8(yaw_speed)
        pitch_angle_hex = self._encode_angle_0_01deg(pitch_deg)
        pitch_speed_hex = self._encode_u8(pitch_speed)

        data = yaw_angle_hex + yaw_speed_hex + pitch_angle_hex + pitch_speed_hex
        frame = self._build_frame("#TP", "G", "w", "GAM", data)
        resp = self._send_and_recv(frame)

        # 可根据实际动作时间适当等待
        time.sleep(1.0)
        return resp

    # ====================  姿态主动输出 (GAA)  ====================

    def enable_attitude_output(self, freq_hz: int = 10) -> Optional[str]:
        """
        使能云台姿态主动送出（GAA）
        :param freq_hz: 1~100 Hz
        """
        freq_hz = max(1, min(100, freq_hz))
        data = self._encode_u8(freq_hz)
        frame = self._build_frame("#TP", "G", "w", "GAA", data)
        return self._send_and_recv(frame)

    def disable_attitude_output(self) -> Optional[str]:
        """
        关闭云台姿态主动送出（GAA 00）
        """
        frame = self._build_frame("#TP", "G", "w", "GAA", "00")
        return self._send_and_recv(frame)

    # ====================  D 类命令：录像 / 拍照 / SD 卡 / 版本号  ====================

    def start_record(self) -> Optional[str]:
        """开始录像：REC 01"""
        frame = self._build_frame("#TP", "D", "w", "REC", "01")
        return self._send_and_recv(frame)

    def stop_record(self) -> Optional[str]:
        """停止录像：REC 00"""
        frame = self._build_frame("#TP", "D", "w", "REC", "00")
        return self._send_and_recv(frame)

    def toggle_record(self) -> Optional[str]:
        """录像状态翻转：REC 0A"""
        frame = self._build_frame("#TP", "D", "w", "REC", "0A")
        return self._send_and_recv(frame)

    def query_record_status(self) -> Optional[int]:
        """
        查询录像状态：REC（r, data=00）
        返回：
            0 -> 未录像
            1 -> 正在录像
            None -> 解析失败/超时
        """
        frame = self._build_frame("#TP", "D", "r", "REC", "00")
        resp = self._send_and_recv(frame)
        if resp and "REC" in resp:
            try:
                idx = resp.index("REC") + 3
                status_char = resp[idx:idx + 2]
                return int(status_char, 16)
            except Exception:
                pass
        return None

    def capture_photo(self) -> Optional[str]:
        """拍照：CAP 01"""
        frame = self._build_frame("#TP", "D", "w", "CAP", "01")
        return self._send_and_recv(frame)

    def query_sdcard_info(self) -> Optional[str]:
        """
        查询 SD 卡容量：SDC（r, data=00）
        返回字符串，解析逻辑可按协议文档补充。
        """
        frame = self._build_frame("#TP", "D", "r", "SDC", "00")
        return self._send_and_recv(frame)

    def query_version(self) -> Optional[str]:
        """读取软件版本号：VER（r, data=00）"""
        frame = self._build_frame("#TP", "D", "r", "VER", "00")
        return self._send_and_recv(frame)

    # ====================  手动键盘控制（类似 SIYISDK.keep_turn）  ====================

    def keep_turn(self) -> None:
        """
        进入键盘控制模式（需要安装 pynput）：
        - ↑ / ↓ 控制俯仰向上 / 向下
        - ← / → 控制航向向左 / 向右
        - W/S 调整俯仰速度
        - A/D 调整航向速度
        - ESC 退出
        """
        if not _HAS_PYNPUT:
            print("pynput 未安装，无法使用键盘控制模式。")
            print("请先安装：pip install pynput")
            return

        yaw_speed = 50
        pitch_speed = 50
        yaw_cmd = 0
        pitch_cmd = 0
        running = True

        def send_speed():
            self.set_speed_yaw_pitch(yaw_cmd, pitch_cmd)

        def on_press(key):
            nonlocal yaw_cmd, pitch_cmd
            try:
                if key == keyboard.Key.up:
                    pitch_cmd = pitch_speed
                elif key == keyboard.Key.down:
                    pitch_cmd = -pitch_speed
                elif key == keyboard.Key.left:
                    yaw_cmd = -yaw_speed
                elif key == keyboard.Key.right:
                    yaw_cmd = yaw_speed
                send_speed()
            except AttributeError:
                pass

        def on_release(key):
            nonlocal yaw_speed, pitch_speed, yaw_cmd, pitch_cmd, running
            if key == keyboard.Key.esc:
                print("ESC pressed, exit manual control")
                running = False
                return False
            try:
                # 松开方向键则停止对应轴
                if key == keyboard.Key.up or key == keyboard.Key.down:
                    pitch_cmd = 0
                if key == keyboard.Key.left or key == keyboard.Key.right:
                    yaw_cmd = 0
                send_speed()

                # WSAD 调整速度大小
                if hasattr(key, "char"):
                    if key.char == "w":
                        pitch_speed = min(127, pitch_speed + 10)
                        print("Pitch speed:", pitch_speed)
                    elif key.char == "s":
                        pitch_speed = max(0, pitch_speed - 10)
                        print("Pitch speed:", pitch_speed)
                    elif key.char == "a":
                        yaw_speed = max(0, yaw_speed - 10)
                        print("Yaw speed:", yaw_speed)
                    elif key.char == "d":
                        yaw_speed = min(127, yaw_speed + 10)
                        print("Yaw speed:", yaw_speed)
            except AttributeError:
                pass

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        print("进入手动控制模式：↑↓←→ 控制云台，WSAD 调整速度，ESC 退出")
        while running:
            time.sleep(0.1)

    # ====================  网络配置示例（IP/网关）  ====================

    def get_ip(self) -> Optional[str]:
        """
        读取相机 IP：IPV（r, data=00）
        文档示例：#tpUD2rIPV0093 -> 返回 #tpUDCrIPV192.168.31.22D2
        """
        frame = self._build_frame("#tp", "D", "r", "IPV", "00")
        return self._send_and_recv(frame)

    def set_ip(self, ip_str: str) -> Optional[str]:
        """
        设置相机 IP：IPV（w, data=字符串）
        示例：#tpUDDwIPV192.168.31.22D7
        """
        data = ip_str
        frame = self._build_frame("#tp", "D", "w", "IPV", data)
        return self._send_and_recv(frame)

    def get_gateway(self) -> Optional[str]:
        """读取网关：GTW（r, data=00）"""
        frame = self._build_frame("#tp", "D", "r", "GTW", "00")
        return self._send_and_recv(frame)

    def set_gateway(self, gw_str: str) -> Optional[str]:
        """设置网关：GTW（w, data=字符串）"""
        data = gw_str
        frame = self._build_frame("#tp", "D", "w", "GTW", data)
        return self._send_and_recv(frame)

    # ====================  资源释放  ====================

    def close(self) -> None:
        """关闭 UDP socket"""
        self.sock.close()

