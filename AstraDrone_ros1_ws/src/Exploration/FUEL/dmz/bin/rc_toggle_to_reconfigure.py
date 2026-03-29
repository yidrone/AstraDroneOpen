#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import RCIn
from dynamic_reconfigure.client import Client as DRClient


class RcToggleToReconfigure:
    """
    Toggle-trigger only:
      - Any change on CH12 (1050<->1950) triggers ONE pulse for Start_Explore
      - Any change on CH7  (1050<->1950) triggers ONE pulse for Return_Home

    Uses MAVROS topic: /mavros/rc/in  (mavros_msgs/RCIn)
    Updates dynamic_reconfigure server: dr_node (e.g., /exploration_node)
    """

    def __init__(self):
        # -------- Parameters --------
        self.rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")

        # IMPORTANT: set to your dynamic_reconfigure server node name
        # Check with: rosrun dynamic_reconfigure dynparam list
        self.dr_node = rospy.get_param("~dr_node", "/exploration_node")

        # RC channel numbers are 1-based; RCIn.channels[] is 0-based
        self.ch_start = int(rospy.get_param("~ch_start_explore", 12))  # CH12 -> Start_Explore
        self.ch_return = int(rospy.get_param("~ch_return_home", 6))    # CH6  -> Return_Home

        # Convert pwm to high/low with threshold
        self.threshold = int(rospy.get_param("~threshold", 1500))

        # Pulse: set True then auto reset False after pulse_sec
        self.pulse_sec = float(rospy.get_param("~pulse_sec", 0.20))

        # Debounce to avoid double triggers due to jitter
        self.debounce_sec = float(rospy.get_param("~debounce_sec", 0.10))

        # -------- State --------
        self.last_start_high = None
        self.last_return_high = None
        self.last_start_change_t = 0.0
        self.last_return_change_t = 0.0

        rospy.loginfo("Connecting dynamic_reconfigure client to: %s", self.dr_node)
        self.dr = DRClient(self.dr_node, timeout=30.0)

        rospy.Subscriber(self.rc_topic, RCIn, self.rc_cb, queue_size=10)

        rospy.loginfo(
            "rc_toggle_to_reconfigure started: rc_topic=%s, dr_node=%s, "
            "CH12->Start_Explore, CH6->Return_Home, threshold=%d, pulse_sec=%.2f, debounce_sec=%.2f",
            self.rc_topic, self.dr_node, self.threshold, self.pulse_sec, self.debounce_sec
        )

    def _get_ch_value(self, msg: RCIn, ch_num_1based: int):
        idx = ch_num_1based - 1
        if idx < 0 or idx >= len(msg.channels):
            return None
        return msg.channels[idx]

    def _is_high(self, pwm):
        return pwm is not None and pwm >= self.threshold

    def _debounced(self, last_change_t):
        now = rospy.get_time()
        return (now - last_change_t) >= self.debounce_sec

    def _set_params(self, **kwargs):
        try:
            self.dr.update_configuration(kwargs)
        except Exception as e:
            rospy.logwarn("dynamic_reconfigure update failed (%s): %s", self.dr_node, str(e))

    def _pulse_true_then_false(self, param_name: str):
        # True now
        self._set_params(**{param_name: True})

        # Reset False after pulse_sec
        rospy.Timer(
            rospy.Duration(self.pulse_sec),
            lambda _evt: self._set_params(**{param_name: False}),
            oneshot=True
        )

    def rc_cb(self, msg: RCIn):
        start_pwm = self._get_ch_value(msg, self.ch_start)     # CH12
        return_pwm = self._get_ch_value(msg, self.ch_return)   # CH6

        start_high = self._is_high(start_pwm)
        return_high = self._is_high(return_pwm)

        now = rospy.get_time()

        # Init on first message
        if self.last_start_high is None:
            self.last_start_high = start_high
        if self.last_return_high is None:
            self.last_return_high = return_high

        # -------- TOGGLE trigger: any change => one pulse --------
        # CH12 -> Start_Explore
        if start_high != self.last_start_high and self._debounced(self.last_start_change_t):
            self.last_start_change_t = now
            rospy.loginfo("CH%d toggled (pwm=%s) -> pulse Start_Explore", self.ch_start, str(start_pwm))
            self._pulse_true_then_false("Start_Explore")
            self.last_start_high = start_high

        # CH7 -> Return_Home
        if return_high != self.last_return_high and self._debounced(self.last_return_change_t):
            self.last_return_change_t = now
            rospy.loginfo("CH%d toggled (pwm=%s) -> pulse Return_Home", self.ch_return, str(return_pwm))
            self._pulse_true_then_false("Return_Home")
            self.last_return_high = return_high


def main():
    rospy.init_node("rc_toggle_to_reconfigure", anonymous=False)
    RcToggleToReconfigure()
    rospy.spin()


if __name__ == "__main__":
    main()

