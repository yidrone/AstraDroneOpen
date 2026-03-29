#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf_conversions
from astra_custom_msgs.msg import MarkerMeasurement 
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker 

# 定义状态转移函数 - 更新为9个状态变量
def fx(x, dt):
    """
    非线性状态转移函数
    状态向量: [x, y, z, φ, v, a, K, curvature_speed, vz]
    Args:
        x: 状态向量 (9维)
        dt: 时间步长
    Returns:
        更新后的状态向量
    """
    x_pos = x[0]           # X位置
    y_pos = x[1]           # Y位置
    z_pos = x[2]           # Z位置
    phi = x[3]             # 偏航角
    v = x[4]               # 速度
    a = x[5]               # 加速度
    K = x[6]               # 曲率
    curvature_speed = x[7] # 曲率变化速度
    vz = x[8]              # Z轴速度

    # 非线性状态更新
    x_pos += v * np.cos(phi) * dt      # X位置更新
    y_pos += v * np.sin(phi) * dt      # Y位置更新
    z_pos += vz * dt                   # Z位置更新（恒速模型）
    phi += K * v * dt                  # 偏航角更新
    v += a * dt                        # 速度更新
    K += curvature_speed * dt          # 曲率更新
    # vz保持不变（恒速模型）
    
    return np.array([x_pos, y_pos, z_pos, phi, v, a, K, curvature_speed, vz])

# 计算状态转移函数的雅可比矩阵 - 更新为9x9矩阵
def Fjacobian(x, dt):
    """
    计算状态转移函数的雅可比矩阵
    Args:
        x: 当前状态向量
        dt: 时间步长
    Returns:
        雅可比矩阵F (9x9)
    """
    F = np.eye(9)  # 9x9单位矩阵
    
    # XY平面模型部分的偏导数
    F[0, 4] = np.cos(x[3]) * dt           # dx/dv
    F[0, 3] = -x[4] * np.sin(x[3]) * dt   # dx/dφ
    F[1, 4] = np.sin(x[3]) * dt           # dy/dv
    F[1, 3] = x[4] * np.cos(x[3]) * dt    # dy/dφ
    F[3, 4] = x[6] * dt                   # dφ/dv
    F[3, 6] = x[4] * dt                   # dφ/dK
    
    # Z轴恒速模型部分
    F[2, 8] = dt  # dz/dvz
    
    return F

# 定义观测函数 - 现在包含Z位置
def hx(x):
    """
    观测函数，从状态向量中提取可观测的状态
    Args:
        x: 状态向量
    Returns:
        观测向量 [x, y, z, φ]
    """
    return np.array([x[0], x[1], x[2], x[3]])  # [x, y, z, φ]

# 计算观测函数的雅可比矩阵 - 更新为4x9矩阵
def Hjacobian(x):
    """
    计算观测函数的雅可比矩阵
    Args:
        x: 状态向量
    Returns:
        观测矩阵H (4x9)
    """
    H = np.zeros((4, 9))
    H[0, 0] = 1  # dx/dx
    H[1, 1] = 1  # dy/dy
    H[2, 2] = 1  # dz/dz
    H[3, 3] = 1  # dφ/dφ
    return H

# 初始化EKF参数
dt = 0.1  # 时间步长
x = np.zeros(9)  # 初始状态 [x, y, z, φ, v, a, K, curvature_speed, vz]
P = np.eye(9) * 10  # 初始协方差矩阵
R = np.diag([0.1, 0.1, 0.1, 0.1])  # 观测噪声协方差 [x, y, z, φ]
Q = np.eye(9) * 0.1  # 过程噪声协方差矩阵

# 初始化ROS节点和发布器
rospy.init_node('ekf_car_model')
rospy.loginfo("EKF Car Model Node Started")

# 创建发布器
pub = rospy.Publisher('/EKF/MarkerMeasurement/estimate', MarkerMeasurement, queue_size=10)  
msg = MarkerMeasurement()  # 自定义消息实例

msg_vis = Marker()
pub_vis = rospy.Publisher('/EKF/Marker/prediction', Marker, queue_size=10)  

# 创建PoseStamped发布器
pub_estimate = rospy.Publisher('/EKF/PoseStamped/estimate', PoseStamped, queue_size=10)  

# 获取参数并转换为浮点数 - 修正关键问题
predicted_time = float(rospy.get_param('~predicted_time', 1.2))  # 确保为浮点数类型
rospy.loginfo(f"Predicted time: {predicted_time} seconds")

# 全局变量
global first_measurement
first_measurement = True

# 初始化Matplotlib
plt.ion()  # 开启交互模式
fig, ax = plt.subplots()
predicted_positions = []
predicted_yaws = []

# 定时器回调函数
def timer_callback(event):
    """
    定时器回调函数, 执行EKF预测步骤
    """
    global x, P
    
    # EKF预测步骤
    F = Fjacobian(x, dt)              # 计算状态转移雅可比矩阵
    x_pred = fx(x, dt)                # 预测下一个状态
    P_pred = F @ P @ F.T + Q          # 预测协方差矩阵

    # 更新状态和协方差
    x = x_pred
    P = P_pred
    
    # 记录预测信息
    rospy.logdebug(f"Predicted state: x={x[0]:.3f}, y={x[1]:.3f}, z={x[2]:.3f}, phi={x[3]:.3f}")

# 创建TF监听器
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 创建定时器
rospy.Timer(rospy.Duration(dt), timer_callback)

# 主循环
rospy.loginfo("Starting main loop...")
while not rospy.is_shutdown():
    try:
        # 获取TF变换
        trans = tf_buffer.lookup_transform('map', 'target', rospy.Time(0), rospy.Duration(1.0))
        z_pos = trans.transform.translation.z  # 获取Z位置
        
        # 创建观测向量 [x, y, z]
        z = np.array([
            trans.transform.translation.x, 
            trans.transform.translation.y,
            z_pos
        ])
        
        # 获取yaw角（φ）
        rotation = trans.transform.rotation
        euler = tf_conversions.transformations.euler_from_quaternion([
            rotation.x, rotation.y, rotation.z, rotation.w
        ])
        phi_measure = euler[2]  # yaw角
        
        # 将φ加入观测向量
        z = np.append(z, phi_measure)  # 完整观测向量为 [x, y, z, φ]
        
        # 如果是第一次测量，初始化状态
        if first_measurement:
            rospy.loginfo("First measurement received, initializing state...")
            x[0] = z[0]  # 设置x位置
            x[1] = z[1]  # 设置y位置
            x[2] = z[2]  # 设置z位置
            x[3] = phi_measure  # 设置初始φ值
            x[4] = 0     # 初始速度
            x[5] = 0     # 初始加速度
            x[6] = 0     # 初始曲率
            x[7] = 0     # 初始曲率速度
            x[8] = 0     # 初始Z速度
            first_measurement = False
            rospy.loginfo(f"Initial state set: x={x[0]:.3f}, y={x[1]:.3f}, z={x[2]:.3f}, phi={x[3]:.3f}")
        else:
            # EKF更新步骤
            y = z - hx(x)                          # 观测残差
            H = Hjacobian(x)                       # 计算观测矩阵
            S = H @ P @ H.T + R                    # 计算观测协方差
            K = P @ H.T @ np.linalg.inv(S)         # 计算卡尔曼增益
            
            # 更新状态和协方差
            x = x + K @ y
            P = (np.eye(len(P)) - K @ H) @ P
            
            rospy.logdebug(f"Updated state: x={x[0]:.3f}, y={x[1]:.3f}, z={x[2]:.3f}, phi={x[3]:.3f}")
        
        # 计算预测位置（在predicted_time时间后的位置）
        predicted_x = x[0] + x[4] * np.cos(x[3]) * predicted_time
        predicted_y = x[1] + x[4] * np.sin(x[3]) * predicted_time
        predicted_z = x[2] + x[8] * predicted_time
        
        # 发布自定义消息
        msg.position.x = predicted_x
        msg.position.y = predicted_y
        msg.position.z = predicted_z
        
        # 转换为四元数
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, x[3])
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        
        # 设置其他消息字段
        msg.yaw = x[3] * 180 / np.pi  # 转换为度
        msg.v = x[4]
        msg.a = x[5]
        msg.K = x[6]
        msg.K_speed = x[7]
        msg.vz = x[8]

        pub.publish(msg)

        # 发布当前位置作为PoseStamped
        msg_estimate = PoseStamped()
        msg_estimate.header.frame_id = "map"
        msg_estimate.header.stamp = rospy.Time.now()
        msg_estimate.pose.position.x = x[0]  # 当前估计的x位置
        msg_estimate.pose.position.y = x[1]  # 当前估计的y位置
        msg_estimate.pose.position.z = x[2]  # 当前估计的z位置
        
        # 当前偏航角的四元数
        quat_current = tf_conversions.transformations.quaternion_from_euler(0, 0, x[3])
        msg_estimate.pose.orientation.x = quat_current[0]
        msg_estimate.pose.orientation.y = quat_current[1]
        msg_estimate.pose.orientation.z = quat_current[2]
        msg_estimate.pose.orientation.w = quat_current[3]
        
        pub_estimate.publish(msg_estimate)

        # 可视化预测点
        msg_vis.header.frame_id = "map"
        msg_vis.header.stamp = rospy.Time.now()  # 使用当前时间
        msg_vis.ns = "land_point"
        msg_vis.id = 0
        msg_vis.type = Marker.SPHERE  # 设置标记类型
        msg_vis.action = Marker.ADD   # 设置动作类型
        
        # 设置预测点位置
        msg_vis.pose.position.x = predicted_x
        msg_vis.pose.position.y = predicted_y
        msg_vis.pose.position.z = predicted_z
        
        # 设置标记大小
        msg_vis.scale.x = 0.1
        msg_vis.scale.y = 0.1
        msg_vis.scale.z = 0.1
        
        # 设置颜色（红色）
        msg_vis.color.a = 1.0
        msg_vis.color.r = 1.0
        msg_vis.color.g = 0.0
        msg_vis.color.b = 0.0

        pub_vis.publish(msg_vis)

        # 更新预测结果用于可视化
        predicted_positions.append((predicted_x, predicted_y, predicted_z))
        predicted_yaws.append(x[3])
        
        # 限制列表长度避免内存过多占用
        if len(predicted_positions) > 1000:
            predicted_positions.pop(0)
            predicted_yaws.pop(0)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"TF exception: {str(e)}")
        # TF异常时继续循环，不退出程序
        
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")
        # 其他异常也记录但不退出
        
    # 控制循环频率
    rospy.sleep(dt)

rospy.loginfo("EKF prediction node shutting down...")