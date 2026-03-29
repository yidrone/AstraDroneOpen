#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool

class RCEmergencyMonitor:
    def __init__(self):
        rospy.init_node('rc_emergency_monitor', anonymous=True)

        # 发布紧急信号
        self.emergency_pub = rospy.Publisher('/patrol/emergency', Bool, queue_size=10)

        # 订阅遥控输入
        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)

        rospy.loginfo("RC Emergency Monitor initialized.")
        rospy.spin()

    def rc_callback(self, msg):
        if len(msg.channels) >= 10:
            ch10_value = msg.channels[9]
            if ch10_value > 1500:
                emergency_msg = Bool(data=True)
                self.emergency_pub.publish(emergency_msg)
                rospy.logwarn("Emergency triggered: RC channel 10 = %d", ch10_value)
            else:
                rospy.loginfo("RC channel 10 = %d, no emergency.", ch10_value)
        else:
            rospy.logerr("RCIn message does not have enough channels!")

if __name__ == '__main__':
    try:
        RCEmergencyMonitor()
    except rospy.ROSInterruptException:
        pass