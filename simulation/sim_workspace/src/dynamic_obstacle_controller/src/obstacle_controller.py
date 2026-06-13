#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class ObstacleMover:
    def __init__(self):
        rospy.init_node('obstacle_controller', anonymous=False)

        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_state.wait_for_service(timeout=10.0)

        self.rate = rospy.get_param('~update_rate', 100)
        self.obstacles = rospy.get_param('~obstacles', [])

        # Pre-compute trajectory functions per obstacle type
        for obs in self.obstacles:
            obs['start_time'] = rospy.get_time()

        rospy.loginfo('Dynamic obstacle controller started with %d obstacles', len(self.obstacles))

    def compute_linear(self, obs, t):
        cx, cy, cz = obs['center']
        amp = obs['amplitude']
        speed = obs['speed']
        ax, ay, az = obs['axis']

        offset = amp * math.sin(speed * t / amp)
        x = cx + ax * offset
        y = cy + ay * offset
        z = cz + az * offset
        return x, y, z

    def compute_circle(self, obs, t):
        cx, cy, cz = obs['center']
        r = obs['radius']
        speed = obs['speed']

        angle = speed * t / r
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        z = cz
        return x, y, z

    def compute_waypoint(self, obs, t):
        waypoints = obs['waypoints']
        speed = obs['speed']

        # Compute total path length
        total_len = 0.0
        seg_lens = []
        for i in range(len(waypoints) - 1):
            dx = waypoints[i + 1][0] - waypoints[i][0]
            dy = waypoints[i + 1][1] - waypoints[i][1]
            seg_len = math.sqrt(dx * dx + dy * dy)
            seg_lens.append(seg_len)
            total_len += seg_len

        if total_len < 1e-6:
            return waypoints[0][0], waypoints[0][1], 0.0

        # Cycle through waypoints
        period = total_len / speed
        dist = (speed * t) % total_len

        accumulated = 0.0
        for i, seg_len in enumerate(seg_lens):
            if accumulated + seg_len >= dist:
                ratio = (dist - accumulated) / seg_len
                x = waypoints[i][0] + ratio * (waypoints[i + 1][0] - waypoints[i][0])
                y = waypoints[i][1] + ratio * (waypoints[i + 1][1] - waypoints[i][1])
                return x, y, 0.5
            accumulated += seg_len

        return waypoints[-1][0], waypoints[-1][1], 0.5

    def set_model_pose(self, name, x, y, z):
        state = ModelState()
        state.model_name = name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        state.reference_frame = 'world'
        try:
            self.set_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, 'Failed to set model state for %s: %s', name, str(e))

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            for obs in self.obstacles:
                t = now - obs['start_time']
                name = obs['name']
                obs_type = obs['type']

                if obs_type == 'linear':
                    x, y, z = self.compute_linear(obs, t)
                elif obs_type == 'circle':
                    x, y, z = self.compute_circle(obs, t)
                elif obs_type == 'waypoint':
                    x, y, z = self.compute_waypoint(obs, t)
                else:
                    rospy.logwarn_throttle(5.0, 'Unknown obstacle type: %s', obs_type)
                    continue

                self.set_model_pose(name, x, y, z)
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ObstacleMover()
        controller.run()
    except rospy.ROSInterruptException:
        pass
