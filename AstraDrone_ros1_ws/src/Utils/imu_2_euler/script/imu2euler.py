import rospy
import tf.transformations as tf_trans
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from collections import deque

# 存储数据的队列
timestamps = deque(maxlen=1000000)
pitch_data = deque(maxlen=1000000)
roll_data = deque(maxlen=1000000)
start_time = None  # 记录开始时间

def imu_callback(msg):
    global timestamps, pitch_data, roll_data, start_time
    
    # 读取四元数
    q = msg.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    
    # 转换为欧拉角 (roll, pitch, yaw)
    euler = tf_trans.euler_from_quaternion(quaternion)
    roll, pitch, _ = np.degrees(euler)  # 转换为度
    
    # 记录数据
    current_time = rospy.Time.now().to_sec()
    if start_time is None:
        start_time = current_time  # 记录第一个数据的时间
    relative_time = current_time - start_time
    timestamps.append(relative_time)  # 归一化时间
    pitch_data.append(pitch)
    roll_data.append(roll)
    
    # print(f"Time: {relative_time:.2f}, Roll: {roll:.4f}, Pitch: {pitch:.4f}")
    
    # 将数据写入文件
    with open("imu_data.txt", "a") as f:
        f.write(f"{relative_time:.2f} {pitch:.4f} {roll:.4f}\n")

def plot_data(event):
    if len(timestamps) > 10:  # 确保有足够数据
        plt.clf()
        
        # 计算整体均值和标准差
        pitch_mean, pitch_std = np.mean(pitch_data), np.std(pitch_data)
        roll_mean, roll_std = np.mean(roll_data), np.std(roll_data)
        
        # plt.subplot(2, 1, 1)
        plt.plot(timestamps, pitch_data, label=f'Pitch (Mean: {pitch_mean:.2f}°, Std: {pitch_std:.2f}°)', color='b')
        plt.plot(timestamps, roll_data, label=f'Roll (Mean: {roll_mean:.2f}°, Std: {roll_std:.2f}°)', color='r')
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch (degree)')
        plt.legend()
        
        plt.xlim(0, max(timestamps))  # x轴从0到最大时间
        plt.pause(0.1)  # 非阻塞式绘图

def main():
    rospy.init_node('imu_euler_plot', anonymous=True)
    rospy.Subscriber('/imu_car/data', Imu, imu_callback)
    
    # 清空旧的文件内容
    with open("imu_data.txt", "w") as f:
        f.write("Time(s) Pitch(deg) Roll(deg)\n")
    
    plt.ion()  # 开启交互模式
    rospy.Timer(rospy.Duration(0.5), plot_data)  # 每0.5秒更新一次图像
    
    print("Subscribed to /imu_car/data, collecting IMU data...")
    rospy.spin()

if __name__ == '__main__':
    main()
