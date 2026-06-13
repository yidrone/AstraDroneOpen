#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import threading
from datetime import datetime

import rospy
from sensor_msgs.msg import Joy, CompressedImage
from nav_msgs.msg import Odometry


def ros_time_to_float(t):
    return float(t.secs) + float(t.nsecs) * 1e-9


def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


class FlightDataRecorder:
    """
    Records UAV / Car / Platform odometry to CSV and saves an image every N seconds,
    gated by Joy axes[7] being -1 (with threshold).
    """

    def __init__(self):
        # --- Parameters ---
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.uav_odom_topic = rospy.get_param("~uav_odom_topic", "/mavros/truth/pose")
        self.car_odom_topic = rospy.get_param("~car_odom_topic", "/car/local_position/pose")
        self.plat_odom_topic = rospy.get_param("~plat_odom_topic", "/new_plat/pose")
        self.image_topic = rospy.get_param("~image_topic", "/iris_depth_camera/camera/rgb/image_raw/compressed")

        # Joy gating: axes index and threshold for "pressed to -1"
        self.gate_axis_index = int(rospy.get_param("~gate_axis_index", 7))  # 8th channel => index 7
        self.gate_threshold = float(rospy.get_param("~gate_threshold", -0.9))  # <= -0.9 treated as -1

        # Recording rates
        self.pose_log_hz = float(rospy.get_param("~pose_log_hz", 50.0))
        self.image_save_interval = float(rospy.get_param("~image_save_interval", 1.0))  # seconds

        # Output directory base
        self.output_root = rospy.get_param("~output_root", os.path.expanduser("~/.ros/astradrone/flight_logs"))
        ensure_dir(self.output_root)

        # Whether to store velocities too
        self.include_twist = bool(rospy.get_param("~include_twist", True))

        # --- Internal state ---
        self._lock = threading.Lock()

        self._is_recording = False
        self._session_dir = None
        self._csv_path = None
        self._csv_fp = None
        self._csv_writer = None

        self._last_image_save_wall = 0.0

        self._latest_uav = None
        self._latest_car = None
        self._latest_plat = None
        self._latest_img = None

        self._last_warn_no_data = 0.0

        # --- Subscribers ---
        self.sub_joy = rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=10)
        self.sub_uav = rospy.Subscriber(self.uav_odom_topic, Odometry, self.cb_uav, queue_size=50)
        self.sub_car = rospy.Subscriber(self.car_odom_topic, Odometry, self.cb_car, queue_size=50)
        self.sub_plat = rospy.Subscriber(self.plat_odom_topic, Odometry, self.cb_plat, queue_size=50)
        self.sub_img = rospy.Subscriber(self.image_topic, CompressedImage, self.cb_img, queue_size=5)

    # ---------------- Callbacks ----------------
    def cb_joy(self, msg: Joy):
        try:
            axis_val = msg.axes[self.gate_axis_index]
        except Exception:
            rospy.logwarn_throttle(1.0, "Joy axes[%d] not available; axes_len=%d",
                                   self.gate_axis_index, len(msg.axes))
            return

        want_record = (axis_val <= self.gate_threshold)

        # Edge-triggered start/stop
        with self._lock:
            if want_record and not self._is_recording:
                self._start_session_locked()
            elif (not want_record) and self._is_recording:
                self._stop_session_locked()

    def cb_uav(self, msg: Odometry):
        with self._lock:
            self._latest_uav = msg

    def cb_car(self, msg: Odometry):
        with self._lock:
            self._latest_car = msg

    def cb_plat(self, msg: Odometry):
        with self._lock:
            self._latest_plat = msg

    def cb_img(self, msg: CompressedImage):
        with self._lock:
            self._latest_img = msg

    # ---------------- Session management ----------------
    def _start_session_locked(self):
        """
        Create a new session directory and open CSV.
        Must be called with self._lock held.
        """
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = os.path.join(self.output_root, f"session_{ts}")
        ensure_dir(self._session_dir)
        ensure_dir(os.path.join(self._session_dir, "images"))

        self._csv_path = os.path.join(self._session_dir, "poses.csv")
        self._csv_fp = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_fp)

        header = [
            "t_ros",  # seconds float (from uav header if available, else rospy.Time.now)
            # UAV
            "uav_px", "uav_py", "uav_pz",
            "uav_qx", "uav_qy", "uav_qz", "uav_qw",
            # CAR
            "car_px", "car_py", "car_pz",
            "car_qx", "car_qy", "car_qz", "car_qw",
            # PLAT
            "plat_px", "plat_py", "plat_pz",
            "plat_qx", "plat_qy", "plat_qz", "plat_qw",
        ]

        if self.include_twist:
            header += [
                "uav_vx", "uav_vy", "uav_vz", "uav_wx", "uav_wy", "uav_wz",
                "car_vx", "car_vy", "car_vz", "car_wx", "car_wy", "car_wz",
                "plat_vx", "plat_vy", "plat_vz", "plat_wx", "plat_wy", "plat_wz",
            ]

        self._csv_writer.writerow(header)
        self._csv_fp.flush()

        self._is_recording = True
        self._last_image_save_wall = 0.0  # force save asap
        rospy.loginfo("Recording started: %s", self._session_dir)

    def _stop_session_locked(self):
        """
        Close CSV and end the session.
        Must be called with self._lock held.
        """
        self._is_recording = False

        if self._csv_fp is not None:
            try:
                self._csv_fp.flush()
                self._csv_fp.close()
            except Exception:
                pass

        rospy.loginfo("Recording stopped. Saved session: %s", self._session_dir)

        self._session_dir = None
        self._csv_path = None
        self._csv_fp = None
        self._csv_writer = None

    # ---------------- Logging helpers ----------------
    @staticmethod
    def _odom_to_fields(odom: Odometry):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        fields = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        return fields

    @staticmethod
    def _twist_to_fields(odom: Odometry):
        v = odom.twist.twist.linear
        w = odom.twist.twist.angular
        return [v.x, v.y, v.z, w.x, w.y, w.z]

    def _get_ros_time_for_row(self, uav: Odometry, car: Odometry, plat: Odometry):
        # Prefer UAV header stamp if valid, else car, else plat, else now
        if uav is not None and uav.header.stamp is not None and (uav.header.stamp.secs != 0 or uav.header.stamp.nsecs != 0):
            return ros_time_to_float(uav.header.stamp)
        if car is not None and car.header.stamp is not None and (car.header.stamp.secs != 0 or car.header.stamp.nsecs != 0):
            return ros_time_to_float(car.header.stamp)
        if plat is not None and plat.header.stamp is not None and (plat.header.stamp.secs != 0 or plat.header.stamp.nsecs != 0):
            return ros_time_to_float(plat.header.stamp)
        return ros_time_to_float(rospy.Time.now())

    def _save_image_if_needed_locked(self, now_wall):
        """
        Save one compressed image to disk if interval elapsed.
        Must be called with self._lock held.
        """
        if self._session_dir is None:
            return
        if self._latest_img is None:
            return
        if (now_wall - self._last_image_save_wall) < self.image_save_interval:
            return

        msg = self._latest_img
        # Use message stamp if available for filename
        if msg.header.stamp is not None and (msg.header.stamp.secs != 0 or msg.header.stamp.nsecs != 0):
            t = ros_time_to_float(msg.header.stamp)
            stamp_str = f"{t:.6f}".replace(".", "_")
        else:
            stamp_str = f"{now_wall:.6f}".replace(".", "_")

        # CompressedImage format could be "jpeg" or "jpg". We'll save as .jpg by default.
        img_dir = os.path.join(self._session_dir, "images")
        filename = os.path.join(img_dir, f"img_{stamp_str}.jpg")

        try:
            with open(filename, "wb") as f:
                f.write(msg.data)
            self._last_image_save_wall = now_wall
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Failed to write image: %s", str(e))

    # ---------------- Main loop ----------------
    def spin(self):
        rate = rospy.Rate(self.pose_log_hz)

        while not rospy.is_shutdown():
            now_wall = time.time()

            with self._lock:
                if not self._is_recording:
                    # Nothing to do
                    rate.sleep()
                    continue

                # Need all three odometries to log a row
                uav = self._latest_uav
                car = self._latest_car
                plat = self._latest_plat

                if uav is None or car is None or plat is None or self._csv_writer is None:
                    # Throttle warning
                    if now_wall - self._last_warn_no_data > 1.0:
                        rospy.logwarn("Recording enabled but missing messages: uav=%s car=%s plat=%s",
                                      "OK" if uav else "None",
                                      "OK" if car else "None",
                                      "OK" if plat else "None")
                        self._last_warn_no_data = now_wall
                    # Still try image save
                    self._save_image_if_needed_locked(now_wall)
                    rate.sleep()
                    continue

                t_ros = self._get_ros_time_for_row(uav, car, plat)

                row = [t_ros]
                row += self._odom_to_fields(uav)
                row += self._odom_to_fields(car)
                row += self._odom_to_fields(plat)

                if self.include_twist:
                    row += self._twist_to_fields(uav)
                    row += self._twist_to_fields(car)
                    row += self._twist_to_fields(plat)

                try:
                    self._csv_writer.writerow(row)
                    self._csv_fp.flush()
                except Exception as e:
                    rospy.logwarn_throttle(1.0, "Failed writing CSV row: %s", str(e))

                # Save image every N seconds
                self._save_image_if_needed_locked(now_wall)

            rate.sleep()

        # On shutdown, close if needed
        with self._lock:
            if self._is_recording:
                self._stop_session_locked()


if __name__ == "__main__":
    rospy.init_node("flight_data_recorder", anonymous=False)
    node = FlightDataRecorder()
    node.spin()
