#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import time
import yaml

# 默认视频设备列表（按优先级排序）
DEFAULT_VIDEO_DEVICES = ['/dev/video0', '/dev/video1', '/dev/video2']
# 重连参数
MAX_RETRIES = 5  # 最大重试次数
RETRY_DELAY = 0.5  # 重试间隔(秒) - 减少为0.5秒
CONNECTION_TIMEOUT = 3  # 连接超时(秒) - 减少为3秒

def load_camera_info(yaml_path):
    """从指定路径加载相机参数"""
    try:
        with open(yaml_path, 'r') as file:
            params = yaml.safe_load(file)
            
        # Create CameraInfo message
        camera_info = CameraInfo()
        
        # Set header (frame_id typically matches camera name)
        camera_info.header.frame_id = params['camera_name']
        
        # Set image dimensions
        camera_info.width = params['image_width']
        camera_info.height = params['image_height']
        
        # Set camera matrix (flatten 3x3 matrix)
        camera_info.K = params['camera_matrix']['data']
        
        # Set distortion model and coefficients
        camera_info.distortion_model = params['distortion_model']
        camera_info.D = params['distortion_coefficients']['data']
        
        # Set rectification matrix (flatten 3x3 matrix)
        camera_info.R = params['rectification_matrix']['data']
        
        # Set projection matrix (flatten 3x4 matrix)
        camera_info.P = params['projection_matrix']['data']
        
        return camera_info
        
    except Exception as e:
        rospy.logerr(f"Failed to load camera parameters: {e}")
        return None

def parse_video_devices(devices_param):
    """解析视频设备参数"""
    if isinstance(devices_param, str):
        # 如果是字符串，按逗号分割
        return [device.strip() for device in devices_param.split(',')]
    elif isinstance(devices_param, list):
        # 如果已经是列表，直接返回
        return devices_param
    else:
        rospy.logwarn(f"无效的视频设备参数格式: {devices_param}，使用默认值")
        return DEFAULT_VIDEO_DEVICES

def open_camera(video_devices, frame_width, frame_height, fourcc, device_index=0):
    """尝试打开摄像头设备"""
    for i in range(device_index, len(video_devices)):
        device = video_devices[i]
        rospy.loginfo(f"尝试打开摄像头设备: {device}")
        
        try:
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if cap.isOpened():
                # 设置摄像头参数
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
                rospy.loginfo(f"成功打开摄像头: {device} ({frame_width}x{frame_height}, {fourcc})")
                return cap, i  # 返回摄像头对象和当前设备索引
            else:
                rospy.logwarn(f"无法打开设备: {device}")
                if cap:
                    cap.release()
        except Exception as e:
            rospy.logerr(f"打开设备 {device} 时出错: {e}")
    
    return None, -1

def publish_camera_feed():
    # 初始化 ROS 节点
    rospy.init_node('mjpeg_camera_publisher', anonymous=True)
    
    # 从launch文件读取参数
    yaml_path = rospy.get_param('~camera_param_yaml', "../param/camera_param.yaml")
    video_devices_param = rospy.get_param('~video_devices', DEFAULT_VIDEO_DEVICES)
    
    # 相机配置参数
    frame_width = rospy.get_param('~frame_width', 1920)
    frame_height = rospy.get_param('~frame_height', 1080)
    fourcc = rospy.get_param('~fourcc', 'MJPG')
    publish_rate = rospy.get_param('~publish_rate', 100)
    
    # Topic名称参数
    image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
    compressed_topic = rospy.get_param('~compressed_topic', '/camera/image_raw/compressed')
    camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
    
    # 图像保存参数
    save_images = rospy.get_param('~save_images', False)
    save_frequency = rospy.get_param('~save_frequency', 3.0)
    save_dir = rospy.get_param('~save_dir', os.path.expanduser("~/camera_sdk_images"))
    
    # 压缩参数
    jpeg_quality = rospy.get_param('~jpeg_quality', 80)
    
    # 连接参数
    max_retries = rospy.get_param('~max_retries', MAX_RETRIES)
    retry_delay = rospy.get_param('~retry_delay', RETRY_DELAY)
    connection_timeout = rospy.get_param('~connection_timeout', CONNECTION_TIMEOUT)
    
    # 解析视频设备参数
    video_devices = parse_video_devices(video_devices_param)
    
    # 打印配置信息
    rospy.loginfo(f"相机配置参数:")
    rospy.loginfo(f"  相机参数文件: {yaml_path}")
    rospy.loginfo(f"  视频设备列表: {video_devices}")
    rospy.loginfo(f"  分辨率: {frame_width}x{frame_height}")
    rospy.loginfo(f"  编码格式: {fourcc}")
    rospy.loginfo(f"  发布频率: {publish_rate} Hz")
    rospy.loginfo(f"  图像保存: {'启用' if save_images else '禁用'}")
    if save_images:
        rospy.loginfo(f"  保存频率: {save_frequency} Hz")
        rospy.loginfo(f"  保存目录: {save_dir}")
    rospy.loginfo(f"  JPEG质量: {jpeg_quality}")
    
    # 创建图像发布者
    image_pub = rospy.Publisher(image_topic, Image, queue_size=10)
    # 创建压缩图像发布者
    compressed_pub = rospy.Publisher(compressed_topic, CompressedImage, queue_size=10)
    # 创建相机信息发布者
    camera_info_pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=10)
    
    # 加载相机内参
    camera_info = load_camera_info(yaml_path)
    if camera_info is None:
        rospy.logerr("无法加载相机参数，继续运行但不会发布相机信息")
    
    # 初始化 OpenCV 到 ROS 的转换桥
    bridge = CvBridge()

    # 图像保存参数计算
    save_interval = 1.0 / save_frequency if save_frequency > 0 else float('inf')
    last_save_time = time.time()
    
    # 创建保存目录
    if save_images and not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # 初始化摄像头
    cap = None
    current_device_index = 0
    retry_count = 0
    last_frame_time = time.time()
    
    # 初始尝试打开摄像头
    cap, current_device_index = open_camera(video_devices, frame_width, frame_height, fourcc)
    if cap is None:
        rospy.logerr("所有摄像头设备均无法打开，请检查连接")
        return

    rate = rospy.Rate(publish_rate)  # 设置发布频率
    while not rospy.is_shutdown():
        # 检查连接状态
        current_time = time.time()
        if cap is None or current_time - last_frame_time > connection_timeout:
            rospy.logwarn("摄像头连接超时，尝试重新连接...")
            
            # 安全释放当前摄像头资源
            if cap is not None:
                try:
                    cap.release()
                except Exception as e:
                    rospy.logerr(f"释放摄像头资源时出错: {e}")
                finally:
                    cap = None
            
            # 尝试重新打开摄像头
            cap, current_device_index = open_camera(video_devices, frame_width, frame_height, fourcc, current_device_index)
            if cap is None:
                if retry_count < max_retries:
                    rospy.logwarn(f"重新连接失败，将在 {retry_delay} 秒后重试 ({retry_count+1}/{max_retries})")
                    retry_count += 1
                    time.sleep(retry_delay)
                    continue
                else:
                    rospy.logerr("达到最大重试次数，退出程序")
                    break
            else:
                retry_count = 0
                last_frame_time = current_time
                rospy.loginfo("摄像头重新连接成功")
        
        # 读取帧
        ret = False
        frame = None
        if cap is not None:
            try:
                ret, frame = cap.read()
            except Exception as e:
                rospy.logerr(f"读取摄像头图像时出错: {e}")
                ret = False
        
        if not ret or frame is None:
            rospy.logwarn("无法读取摄像头图像，尝试重新获取...")
            time.sleep(0.1)
            continue
        
        # 重置超时计时器和重试计数器
        last_frame_time = time.time()
        retry_count = 0
        
        # 检查图像形状
        if frame is None:
            rospy.logwarn("获取的帧为空")
            continue

        try:
            # 获取当前时间
            current_rostime = rospy.Time.now()
            
            # 发布原始图像
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = current_rostime
            image_pub.publish(img_msg)
            
            # 发布压缩图像
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = current_rostime
            compressed_msg.format = "jpeg"
            # 使用cv2.imencode将图像压缩为JPEG格式
            ret, compressed_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if ret:
                compressed_msg.data = compressed_data.tobytes()
                compressed_pub.publish(compressed_msg)
            else:
                rospy.logwarn("图像压缩失败")
            
            # 发布相机信息
            if camera_info is not None:
                camera_info.header.stamp = current_rostime
                camera_info_pub.publish(camera_info)
            
            # 图像保存功能
            if save_images:
                current_save_time = time.time()
                if current_save_time - last_save_time >= save_interval:
                    # 生成带时间戳的文件名
                    timestamp = current_rostime.to_sec()
                    filename = os.path.join(save_dir, f"image_{timestamp:.3f}.jpg")
                    cv2.imwrite(filename, frame)
                    rospy.loginfo(f"已保存图像: {filename}")
                    last_save_time = current_save_time
                
        except Exception as e:
            rospy.logerr(f"图像处理出错: {e}")

        rate.sleep()
    
    # 清理资源
    if cap is not None:
        try:
            cap.release()
        except Exception as e:
            rospy.logerr(f"释放摄像头资源时出错: {e}")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publish_camera_feed()
    except rospy.ROSInterruptException:
        pass