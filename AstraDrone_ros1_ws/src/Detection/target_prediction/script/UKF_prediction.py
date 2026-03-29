#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf_conversions  # 用于四元数转换
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import matplotlib.pyplot as plt

# 定义状态转移函数
def fx(x, dt):
    x_pos = x[0]
    y_pos = x[1]
    phi = x[2]
    v = x[3]
    a = x[4]
    K = x[5]
    curvature_speed = x[6]  # 新增曲率速度

    # 非线性状态更新
    x_pos += v * np.cos(phi) * dt
    y_pos += v * np.sin(phi) * dt
    phi += K * v * dt
    v += a * dt
    K += curvature_speed * dt  # 更新曲率速度
    
    return np.array([x_pos, y_pos, phi, v, a, K, curvature_speed])

# 定义观测函数
def hx(x):
    return np.array([x[0], x[1], x[2]])  # 现在返回 [x, y, φ]

# 初始化UKF参数
dt = 0.1  # 时间步长
sigma_points = MerweScaledSigmaPoints(7, alpha=0.1, beta=2., kappa=0)
ukf = UKF(dim_x=7, dim_z=3, fx=fx, hx=hx, dt=dt, points=sigma_points)
ukf.x = np.zeros(7)  # 初始状态 [x, y, φ, v, a, K, curvature_speed]
ukf.P *= 10  # 初始协方差
ukf.R = np.diag([0.1, 0.1, 0.1])  # 观测噪声，添加 φ 的噪声
ukf.Q = np.eye(7) * 0.1  # 过程噪声

global first_measurement
first_measurement = True

# 初始化Matplotlib
plt.ion()  # 开启交互模式
fig, ax = plt.subplots()
predicted_positions = []
predicted_yaws = []

# 定时器回调函数
def timer_callback(event):
    # 进行预测
    ukf.predict()
    print("Predicted state:", ukf.x)
    print("\033[34m v,sin(phi)", ukf.x[3],np.sin(ukf.x[2]),"\n \033[0m")

# 初始化ROS节点
rospy.init_node('ukf_car_model')

# 创建TF监听器
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 创建定时器
rospy.Timer(rospy.Duration(dt), timer_callback)

# 主循环
while not rospy.is_shutdown():
    try:
        # 获取TF变换
        trans = tf_buffer.lookup_transform('map', 'aruco', rospy.Time(0), rospy.Duration(1.0))
        z = np.array([trans.transform.translation.x, trans.transform.translation.y])
        
        # 获取yaw角（phi）
        rotation = trans.transform.rotation
        euler = tf_conversions.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        phi_measure = euler[2]  # yaw角
        
        # 将phi加入观测向量
        z = np.append(z, phi_measure)  # 更新观测向量为 [x, y, φ]
        
        # 如果是第一次测量，设置初始状态
        if first_measurement:
            ukf.x[0] = z[0]  # 设置x位置
            ukf.x[1] = z[1]  # 设置y位置
            ukf.x[2] = phi_measure  # 设置初始φ值
            ukf.x[3] = 0  
            ukf.x[4] = 0  
            ukf.x[5] = 0  
            ukf.x[6] = 0  # 初始化曲率速度
            first_measurement = False
        else:
            # UKF更新
            ukf.update(z)

        # 更新预测结果
        predicted_positions.append((ukf.x[0], ukf.x[1]))
        predicted_yaws.append(ukf.x[2])

        # 清空当前图形
        ax.clear()
        ax.set_title("UKF Prediction")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_xlim(-10, 10)  # 根据实际情况设置坐标范围
        ax.set_ylim(-10, 10)  # 根据实际情况设置坐标范围

        # 绘制预测轨迹
        if len(predicted_positions) > 1:
            positions = np.array(predicted_positions)
            ax.plot(positions[:, 0], positions[:, 1], label='Predicted Path')
            ax.plot(ukf.x[0], ukf.x[1], 'ro')  # 当前预测点
            ax.legend()

        # 添加表示yaw角的箭头
        arrow_length = 0.5  # 箭头长度
        arrow_dx = arrow_length * np.cos(ukf.x[2])  # 计算箭头x方向分量
        arrow_dy = arrow_length * np.sin(ukf.x[2])  # 计算箭头y方向分量
        ax.quiver(ukf.x[0], ukf.x[1], arrow_dx, arrow_dy, angles='xy', scale_units='xy', scale=1, color='b', label='Yaw Angle')

        plt.pause(0.1)  # 暂停以更新图形

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass

    rospy.sleep(dt)
