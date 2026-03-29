#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# rosrun joy joy_node _dev:=/dev/input/js0 _deadzone:=0.05 _autorepeat_rate:=30

import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn, ManualControl

RC_OVERRIDE_CHANNELS = 18   # mavros_msgs/OverrideRCIn 在 ROS Noetic 常见为 18
RC_FORWARD_CHANNELS = 8     # 转发 8 个通道（CH1..CH8）


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def pwm_centered(axis_val: float, center: int = 1500, span: int = 500) -> int:
    """axis_val in [-1,1] -> [center-span, center+span]"""
    return int(clamp(center + axis_val * span, center - span, center + span))


def pwm_throttle(axis_val: float, lo: int = 1000, hi: int = 2000) -> int:
    """axis_val in [-1,1] -> [lo,hi]"""
    pwm = lo + (axis_val + 1.0) * 0.5 * (hi - lo)
    return int(clamp(pwm, lo, hi))


def pwm_from_button(btn: int, lo: int = 1000, hi: int = 2000) -> int:
    """button 0/1 -> lo/hi"""
    return hi if btn else lo


class JoyToPX4ManualAndRCOverride:
    """
    /joy -> /mavros/rc/override   (RC_CHANNELS_OVERRIDE)
        + /mavros/manual_control/send (MANUAL_CONTROL) [可选但强烈建议开启]
    """

    def __init__(self):
        # --- Rate ---
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        # --- Safety gate ---
        # 按住 enable_button 才会输出（-1 表示不门控，不推荐）
        self.enable_button = int(rospy.get_param("~enable_button", 1))

        # --- Publish MANUAL_CONTROL (recommended) ---
        self.publish_manual_control = bool(rospy.get_param("~publish_manual_control", True))

        # --- Primary stick axes mapping (from /joy.axes indices) ---
        # 你的 /joy.axes 示例是 8 路：
        # axes: [0..7]
        # 我们按 CH1..CH8 直接使用 axes[0..7] 来生成 PWM，避免 CH5..CH8 走按钮导致“不对”
        self.axis_roll = int(rospy.get_param("~axis_roll", 0))        # CH1
        self.axis_pitch = int(rospy.get_param("~axis_pitch", 1))      # CH2
        self.axis_throttle = int(rospy.get_param("~axis_throttle", 2))# CH3
        self.axis_yaw = int(rospy.get_param("~axis_yaw", 3))          # CH4

        # Invert flags
        # 你反馈“除了通道2都是反了”，因此：
        # - CH1 反
        # - CH2 不反
        # - CH3 反
        # - CH4 反
        # CH5..CH8 也按你的描述需要反（可通过参数覆盖）
        self.inv_roll = bool(rospy.get_param("~invert_roll", True))
        self.inv_pitch = bool(rospy.get_param("~invert_pitch", True))
        self.inv_throttle = bool(rospy.get_param("~invert_throttle", True))
        self.inv_yaw = bool(rospy.get_param("~invert_yaw", True))

        # --- Aux channels CH5..CH8 source configuration ---
        # 修改：默认全部从 axes[4..7] 读取（与你给的 /joy.axes 结构一致）
        self.aux = []
        for ch in range(5, 9):  # CH5..CH8
            prefix = f"~ch{ch}"
            default_axis_index = ch - 1  # CH5->axes[4], CH6->axes[5], CH7->axes[6], CH8->axes[7]
            cfg = {
                "type": str(rospy.get_param(prefix + "_type", "axis")),          # 默认 axis
                "index": int(rospy.get_param(prefix + "_index", default_axis_index)),
                "invert": bool(rospy.get_param(prefix + "_invert", True)),      # 默认反（你说除了CH2都反）
                "btn_lo": int(rospy.get_param(prefix + "_btn_lo", 1000)),
                "btn_hi": int(rospy.get_param(prefix + "_btn_hi", 2000)),
                "pwm_const": int(rospy.get_param(prefix + "_const", 1500)),
            }
            self.aux.append(cfg)

        self.last_joy = None

        # Publishers/Subscribers
        self.pub_rc = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        self.pub_manual = None
        if self.publish_manual_control:
            self.pub_manual = rospy.Publisher("/mavros/manual_control/send", ManualControl, queue_size=10)

        self.sub = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

        rospy.loginfo("joy_to_px4_manual_rc started. rate=%.1fHz, manual_control=%s",
                      self.rate_hz, str(self.publish_manual_control))

    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    @staticmethod
    def get_axis(msg: Joy, idx: int, invert: bool = False) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        v = float(msg.axes[idx])
        return -v if invert else v

    @staticmethod
    def get_button(msg: Joy, idx: int) -> int:
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return int(msg.buttons[idx])

    def enabled(self, joy: Joy) -> bool:
        if self.enable_button < 0:
            return True
        return self.get_button(joy, self.enable_button) == 1

    def build_aux_pwm(self, joy: Joy, cfg: dict) -> int:
        t = cfg["type"].lower()
        if t == "axis":
            a = self.get_axis(joy, cfg["index"], cfg["invert"])
            return pwm_centered(a)
        if t == "button":
            b = self.get_button(joy, cfg["index"])
            return pwm_from_button(b, cfg["btn_lo"], cfg["btn_hi"])
        if t == "const":
            return int(cfg["pwm_const"])
        return 1500

    def publish_rc_override(self, joy: Joy, enable: bool):
        msg = OverrideRCIn()
        msg.channels = [0] * RC_OVERRIDE_CHANNELS  # 必须填满消息定义长度

        if not enable:
            # 全 0 = release override
            self.pub_rc.publish(msg)
            return

        # CH1..CH4（按你“除CH2都反”的反馈修正 invert）
        roll = self.get_axis(joy, self.axis_roll, self.inv_roll)
        pitch = self.get_axis(joy, self.axis_pitch, self.inv_pitch)
        thr = self.get_axis(joy, self.axis_throttle, self.inv_throttle)
        yaw = self.get_axis(joy, self.axis_yaw, self.inv_yaw)

        ch1 = pwm_centered(roll)          # CH1 Roll
        ch2 = pwm_centered(pitch)         # CH2 Pitch
        ch3 = pwm_throttle(thr)           # CH3 Throttle
        ch4 = pwm_centered(yaw)           # CH4 Yaw

        # CH5..CH8：默认从 axes[4..7] 读取并转为 centered PWM
        ch5 = self.build_aux_pwm(joy, self.aux[0])
        ch6 = self.build_aux_pwm(joy, self.aux[1])
        ch7 = self.build_aux_pwm(joy, self.aux[2])
        ch8 = self.build_aux_pwm(joy, self.aux[3])

        forward = [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]

        # 写入 OverrideRCIn 前 8 个通道，其它保持 0（release）
        for i in range(RC_FORWARD_CHANNELS):
            msg.channels[i] = int(forward[i])

        self.pub_rc.publish(msg)

    def publish_manual_control_msg(self, joy: Joy, enable: bool):
        if not self.publish_manual_control or self.pub_manual is None:
            return

        mc = ManualControl()

        if not enable:
            mc.x = 0.0
            mc.y = 0.0
            mc.z = 0.0
            mc.r = 0.0
            mc.buttons = 0
            self.pub_manual.publish(mc)
            return

        # MANUAL_CONTROL 也跟随相同反向逻辑
        roll = self.get_axis(joy, self.axis_roll, self.inv_roll)
        pitch = self.get_axis(joy, self.axis_pitch, self.inv_pitch)
        thr = self.get_axis(joy, self.axis_throttle, self.inv_throttle)
        yaw = self.get_axis(joy, self.axis_yaw, self.inv_yaw)

        mc.x = float(clamp(pitch * 1000.0, -1000.0, 1000.0))
        mc.y = float(clamp(roll * 1000.0, -1000.0, 1000.0))
        mc.r = float(clamp(yaw * 1000.0, -1000.0, 1000.0))
        mc.z = float(clamp((thr + 1.0) * 0.5 * 1000.0, 0.0, 1000.0))

        mask = 0
        for i in range(min(len(joy.buttons), 16)):
            if joy.buttons[i]:
                mask |= (1 << i)
        mc.buttons = mask

        self.pub_manual.publish(mc)

    def on_timer(self, _evt):
        if self.last_joy is None:
            dummy = Joy()
            self.publish_rc_override(dummy, enable=False)
            self.publish_manual_control_msg(dummy, enable=False)
            return

        joy = self.last_joy
        enable = self.enabled(joy)

        self.publish_rc_override(joy, enable=enable)
        self.publish_manual_control_msg(joy, enable=enable)


def main():
    rospy.init_node("joy_to_px4_manual_rc")
    JoyToPX4ManualAndRCOverride()
    rospy.spin()


if __name__ == "__main__":
    main()
