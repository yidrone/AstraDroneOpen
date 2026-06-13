#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <camera_sdk/CameraConfigConfig.h>  // 注意：你的动态参数类型

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <chrono>
#include <dirent.h>
#include <algorithm>
#include <sys/stat.h>
#include <errno.h>
#include <cmath>
#include <cstring>

// JPEG压缩库
#include <turbojpeg.h>

class USBCameraDriver {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // 图像传输
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher compressed_pub_;
    ros::Publisher camera_info_pub_;

    // 相机信息管理器
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    // 动态参数服务器
    dynamic_reconfigure::Server<camera_sdk::CameraConfigConfig> dyn_server_;
    dynamic_reconfigure::Server<camera_sdk::CameraConfigConfig>::CallbackType dyn_cb_;

    // 相机参数
    std::string camera_param_yaml_;
    std::vector<std::string> video_devices_;
    int frame_width_;
    int frame_height_;
    std::string fourcc_;
    int publish_rate_;
    std::string image_topic_;
    std::string compressed_topic_;
    std::string camera_info_topic_;
    bool save_images_;
    double save_frequency_;
    std::string save_dir_;
    int jpeg_quality_;
    int max_retries_;
    double retry_delay_;
    int connection_timeout_;
    std::string camera_name_;

    // 新增：旋转 & 内参缩放
    double rotate_angle_;          // 度
    bool   scale_camera_info_;     // 是否缩放 YAML 内参

    // V4L2相关
    int camera_fd_;
    struct v4l2_buffer bufferinfo_;
    void* buffer_start_;
    size_t buffer_length_;
    std::string current_device_;

    // JPEG压缩器
    tjhandle tj_compressor_;
    tjhandle tj_decompressor_;

    // 线程控制
    std::atomic<bool> running_;
    std::thread capture_thread_;
    std::thread reconnect_thread_;
    std::mutex camera_mutex_;
    std::condition_variable camera_cv_;

    // 状态变量
    std::atomic<bool> camera_connected_;
    std::atomic<int> retry_count_;
    std::atomic<bool> need_reconnect_;

    // 图像保存
    ros::Timer save_timer_;
    int image_counter_;

public:
    USBCameraDriver()
        : private_nh_("~"),
          it_(nh_),
          camera_fd_(-1),
          buffer_start_(nullptr),
          buffer_length_(0),
          running_(false),
          camera_connected_(false),
          retry_count_(0),
          need_reconnect_(false),
          image_counter_(0),
          rotate_angle_(0.0),
          scale_camera_info_(false)
    {
        // 初始化TurboJPEG
        tj_compressor_ = tjInitCompress();
        tj_decompressor_ = tjInitDecompress();

        // 加载参数
        loadParameters();

        // 设置动态参数回调（如你的CameraConfigConfig里没有新字段，这里不改）
        dyn_cb_ = boost::bind(&USBCameraDriver::dynamicReconfigureCallback, this, _1, _2);
        dyn_server_.setCallback(dyn_cb_);

        // 初始化相机信息管理器
        camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_param_yaml_));

        // 设置发布器
        image_pub_ = it_.advertise(image_topic_, 1);
        compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(compressed_topic_, 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 1);

        // 设置图像保存定时器
        if (save_images_) {
            save_timer_ = nh_.createTimer(ros::Duration(1.0 / save_frequency_),
                                          &USBCameraDriver::saveImageTimerCallback, this);
        }

        // 启动相机连接线程
        running_ = true;
        reconnect_thread_ = std::thread(&USBCameraDriver::reconnectThread, this);
    }

    ~USBCameraDriver() {
        running_ = false;

        if (reconnect_thread_.joinable()) reconnect_thread_.join();
        if (capture_thread_.joinable())   capture_thread_.join();

        closeCamera();

        if (tj_compressor_)   tjDestroy(tj_compressor_);
        if (tj_decompressor_) tjDestroy(tj_decompressor_);
    }

    void loadParameters() {
        private_nh_.param("camera_param_yaml", camera_param_yaml_, std::string(""));
        private_nh_.param("camera_name", camera_name_, std::string("usb_cam"));

        std::string video_devices_str;
        private_nh_.param("video_devices", video_devices_str, std::string("/dev/video0"));
        video_devices_.clear();
        size_t pos = 0;
        while ((pos = video_devices_str.find(',')) != std::string::npos) {
            video_devices_.push_back(video_devices_str.substr(0, pos));
            video_devices_str.erase(0, pos + 1);
        }
        if (!video_devices_str.empty()) video_devices_.push_back(video_devices_str);

        private_nh_.param("frame_width", frame_width_, 640);
        private_nh_.param("frame_height", frame_height_, 480);
        private_nh_.param("fourcc", fourcc_, std::string("MJPG"));
        private_nh_.param("publish_rate", publish_rate_, 30);
        private_nh_.param("image_topic", image_topic_, std::string("/camera/image_raw"));
        private_nh_.param("compressed_topic", compressed_topic_, std::string("/camera/image_raw/compressed"));
        private_nh_.param("camera_info_topic", camera_info_topic_, std::string("/camera/camera_info"));
        private_nh_.param("save_images", save_images_, false);
        private_nh_.param("save_frequency", save_frequency_, 3.0);
        private_nh_.param("save_dir", save_dir_, std::string(std::string(getenv("HOME") ? getenv("HOME") : "/tmp") + "/camera_sdk_images"));
        private_nh_.param("jpeg_quality", jpeg_quality_, 80);
        private_nh_.param("max_retries", max_retries_, 5);
        private_nh_.param("retry_delay", retry_delay_, 0.5);
        private_nh_.param("connection_timeout", connection_timeout_, 3);

        // 新增
        private_nh_.param("rotate_angle", rotate_angle_, 0.0);
        private_nh_.param("scale_camera_info", scale_camera_info_, false);
    }

    void dynamicReconfigureCallback(camera_sdk::CameraConfigConfig &config, uint32_t level) {
        frame_width_   = config.frame_width;
        frame_height_  = config.frame_height;
        jpeg_quality_  = config.jpeg_quality;
        publish_rate_  = config.publish_rate;
        save_images_   = config.save_images;
        save_frequency_= config.save_frequency;

        // 如果相机已连接且参数有变化，需要重连
        if (camera_connected_ && level > 0) {
            need_reconnect_ = true;
        }
    }

    bool openCamera(const std::string& device) {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        camera_fd_ = open(device.c_str(), O_RDWR);
        if (camera_fd_ == -1) {
            ROS_ERROR("Failed to open device %s: %s", device.c_str(), strerror(errno));
            return false;
        }

        struct v4l2_capability cap;
        if (ioctl(camera_fd_, VIDIOC_QUERYCAP, &cap) == -1) {
            ROS_ERROR("Failed to query capabilities: %s", strerror(errno));
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            ROS_ERROR("Device %s is not a video capture device", device.c_str());
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            ROS_ERROR("Device %s does not support streaming", device.c_str());
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        struct v4l2_format format;
        std::memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width  = frame_width_;
        format.fmt.pix.height = frame_height_;

        if (fourcc_ == "MJPG")      format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        else if (fourcc_ == "YUYV") format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        else if (fourcc_ == "H264") format.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
        else {
            ROS_ERROR("Unsupported format: %s", fourcc_.c_str());
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        format.fmt.pix.field = V4L2_FIELD_ANY;

        if (ioctl(camera_fd_, VIDIOC_S_FMT, &format) == -1) {
            ROS_ERROR("Failed to set format: %s", strerror(errno));
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        struct v4l2_requestbuffers bufrequest;
        bufrequest.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufrequest.memory = V4L2_MEMORY_MMAP;
        bufrequest.count  = 1;

        if (ioctl(camera_fd_, VIDIOC_REQBUFS, &bufrequest) == -1) {
            ROS_ERROR("Failed to request buffers: %s", strerror(errno));
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        std::memset(&bufferinfo_, 0, sizeof(bufferinfo_));
        bufferinfo_.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo_.memory = V4L2_MEMORY_MMAP;
        bufferinfo_.index  = 0;

        if (ioctl(camera_fd_, VIDIOC_QUERYBUF, &bufferinfo_) == -1) {
            ROS_ERROR("Failed to query buffer: %s", strerror(errno));
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        buffer_start_ = mmap(NULL, bufferinfo_.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, camera_fd_, bufferinfo_.m.offset);

        if (buffer_start_ == MAP_FAILED) {
            ROS_ERROR("Failed to map buffer: %s", strerror(errno));
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        buffer_length_ = bufferinfo_.length;

        if (ioctl(camera_fd_, VIDIOC_QBUF, &bufferinfo_) == -1) {
            ROS_ERROR("Failed to queue buffer: %s", strerror(errno));
            munmap(buffer_start_, buffer_length_);
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        int type = bufferinfo_.type;
        if (ioctl(camera_fd_, VIDIOC_STREAMON, &type) == -1) {
            ROS_ERROR("Failed to start streaming: %s", strerror(errno));
            munmap(buffer_start_, buffer_length_);
            close(camera_fd_); camera_fd_ = -1;
            return false;
        }

        current_device_ = device;
        camera_connected_ = true;
        retry_count_ = 0;

        ROS_INFO("Camera %s opened successfully", device.c_str());
        return true;
    }

    void closeCamera() {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        if (camera_fd_ != -1) {
            int type = bufferinfo_.type;
            ioctl(camera_fd_, VIDIOC_STREAMOFF, &type);

            if (buffer_start_ != nullptr) {
                munmap(buffer_start_, buffer_length_);
                buffer_start_ = nullptr;
            }

            close(camera_fd_);
            camera_fd_ = -1;
        }

        camera_connected_ = false;
        ROS_INFO("Camera closed");
    }

    // 转换 YUYV -> RGB
    static inline uint8_t clamp_int(int v) {
        return (uint8_t)std::max(0, std::min(255, v));
    }

    void convertYUYVtoRGB(const unsigned char* yuyv, unsigned char* rgb, int width, int height) {
        for (int y = 0; y < height; y++) {
            const int row = y * width * 2;
            for (int x = 0; x < width; x += 2) {
                int index = row + x * 2;

                int y0 = yuyv[index + 0];
                int u  = yuyv[index + 1];
                int y1 = yuyv[index + 2];
                int v  = yuyv[index + 3];

                int r0 = (int)(y0 + 1.402 * (v - 128));
                int g0 = (int)(y0 - 0.344136 * (u - 128) - 0.714136 * (v - 128));
                int b0 = (int)(y0 + 1.772 * (u - 128));

                int r1 = (int)(y1 + 1.402 * (v - 128));
                int g1 = (int)(y1 - 0.344136 * (u - 128) - 0.714136 * (v - 128));
                int b1 = (int)(y1 + 1.772 * (u - 128));

                int rgb_index0 = (y * width + x) * 3;
                int rgb_index1 = rgb_index0 + 3;

                rgb[rgb_index0 + 0] = clamp_int(r0);
                rgb[rgb_index0 + 1] = clamp_int(g0);
                rgb[rgb_index0 + 2] = clamp_int(b0);

                rgb[rgb_index1 + 0] = clamp_int(r1);
                rgb[rgb_index1 + 1] = clamp_int(g1);
                rgb[rgb_index1 + 2] = clamp_int(b1);
            }
        }
    }

    // 双线性插值旋转（RGB8，输出尺寸与输入一致；边界外填充0）
    void rotateImageBilinear(const sensor_msgs::Image& src, sensor_msgs::Image& dst, double angle_deg) {
        if (angle_deg == 0.0) { dst = src; return; }

        const double rad = angle_deg * M_PI / 180.0;
        const double cosA = std::cos(rad);
        const double sinA = std::sin(rad);

        dst = src;
        std::fill(dst.data.begin(), dst.data.end(), 0);

        const int W = src.width;
        const int H = src.height;
        const int C = 3;  // RGB
        const double cx = (W - 1) * 0.5;
        const double cy = (H - 1) * 0.5;

        const uint8_t* S = src.data.data();
        uint8_t*       D = dst.data.data();

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                // 逆变换：目标(x,y) -> 源(xs,ys)
                const double dx = x - cx;
                const double dy = y - cy;
                const double xs =  cosA * dx + sinA * dy + cx;
                const double ys = -sinA * dx + cosA * dy + cy;

                if (xs < 0.0 || xs >= W - 1 || ys < 0.0 || ys >= H - 1) {
                    // 出界 => 黑色
                    continue;
                }

                const int x0 = (int)std::floor(xs);
                const int y0 = (int)std::floor(ys);
                const double ax = xs - x0;
                const double ay = ys - y0;

                const int x1 = x0 + 1;
                const int y1 = y0 + 1;

                const uint8_t* p00 = &S[(y0 * W + x0) * C];
                const uint8_t* p10 = &S[(y0 * W + x1) * C];
                const uint8_t* p01 = &S[(y1 * W + x0) * C];
                const uint8_t* p11 = &S[(y1 * W + x1) * C];

                uint8_t* pd = &D[(y * W + x) * C];

                for (int c = 0; c < C; ++c) {
                    double v = (1 - ax) * (1 - ay) * p00[c]
                             +      ax  * (1 - ay) * p10[c]
                             + (1 - ax) *      ay  * p01[c]
                             +      ax  *      ay  * p11[c];
                    if (v < 0) v = 0;
                    if (v > 255) v = 255;
                    pd[c] = static_cast<uint8_t>(v + 0.5);
                }
            }
        }
    }

    // 缩放 camera_info（K、P、roi）
    void maybeScaleCameraInfo(sensor_msgs::CameraInfo& ci) {
        if (!scale_camera_info_) return;

        if (ci.width == 0 || ci.height == 0) {
            // 如果 YAML 没填宽高，用当前帧尺寸
            ci.width  = frame_width_;
            ci.height = frame_height_;
            return;
        }

        const double sx = static_cast<double>(frame_width_)  / static_cast<double>(ci.width);
        const double sy = static_cast<double>(frame_height_) / static_cast<double>(ci.height);

        // 更新 width/height
        ci.width  = frame_width_;
        ci.height = frame_height_;

        // 内参 K： fx 0 cx 0 fy cy 0 0 1
        ci.K[0] *= sx; // fx
        ci.K[2] *= sx; // cx
        ci.K[4] *= sy; // fy
        ci.K[5] *= sy; // cy

        // 投影矩阵 P： fx' 0 cx' Tx 0 fy' cy' Ty 0 0 1 0
        ci.P[0] *= sx; // fx'
        ci.P[2] *= sx; // cx'
        ci.P[5] *= sy; // fy'
        ci.P[6] *= sy; // cy'
        // Tx, Ty 一般与基线有关，若存在也按各向缩放（通常为0）
        ci.P[3] *= sx;
        ci.P[7] *= sy;

        // ROI
        ci.roi.width  = static_cast<uint32_t>(std::round(ci.roi.width  * sx));
        ci.roi.height = static_cast<uint32_t>(std::round(ci.roi.height * sy));
        ci.roi.x_offset = static_cast<uint32_t>(std::round(ci.roi.x_offset * sx));
        ci.roi.y_offset = static_cast<uint32_t>(std::round(ci.roi.y_offset * sy));
    }

    // 注意：增加 force_decode_rgb 开关，旋转时需要
    bool captureFrame(sensor_msgs::Image& image_msg,
                      sensor_msgs::CompressedImage& compressed_msg,
                      bool force_decode_rgb = false)
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_fd_ == -1) return false;

        // 1) 等待可读（处理 EINTR）
        fd_set fds;
        struct timeval tv;
        int r = 0;
        do {
            FD_ZERO(&fds);
            FD_SET(camera_fd_, &fds);
            tv.tv_sec  = connection_timeout_;
            tv.tv_usec = 0;
            r = select(camera_fd_ + 1, &fds, NULL, NULL, &tv);
        } while (r == -1 && errno == EINTR);

        if (r == -1) {
            ROS_ERROR("select() error: %s", strerror(errno));
            return false;
        } else if (r == 0) {
            ROS_ERROR("select() timeout");
            return false;
        }

        // 2) 取出缓冲
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(camera_fd_, VIDIOC_DQBUF, &buf) == -1) {
            ROS_ERROR("VIDIOC_DQBUF failed: %s", strerror(errno));
            return false;
        }

        bool success = true;
        auto requeue = [&]() {
            if (ioctl(camera_fd_, VIDIOC_QBUF, &buf) == -1) {
                ROS_ERROR("VIDIOC_QBUF (requeue) failed: %s", strerror(errno));
                success = false;
            }
        };

        const ros::Time capture_time = ros::Time::now();
        const size_t bytes = buf.bytesused;

        // 3) 解析不同像素格式
        if (fourcc_ == "MJPG") {
            // 直接保存 compressed
            compressed_msg.format = "jpeg";
            compressed_msg.data.resize(bytes);
            std::memcpy(compressed_msg.data.data(), buffer_start_, bytes);

            // 当需要旋转 or 需要原始图像时，解码 JPEG -> RGB
            if (force_decode_rgb || image_pub_.getNumSubscribers() > 0) {
                int width = 0, height = 0, subsamp = 0;
                unsigned char* jpeg_ptr = static_cast<unsigned char*>(buffer_start_);
                if (tjDecompressHeader2(tj_decompressor_, jpeg_ptr, bytes, &width, &height, &subsamp) != 0) {
                    ROS_ERROR("tjDecompressHeader2 failed: %s", tjGetErrorStr());
                    success = false;
                } else {
                    image_msg.width  = width;
                    image_msg.height = height;
                    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    image_msg.step   = width * 3;
                    image_msg.data.resize(static_cast<size_t>(width) * height * 3);

                    if (tjDecompress2(tj_decompressor_,
                                      jpeg_ptr, bytes,
                                      image_msg.data.data(), width, 0, height,
                                      TJPF_RGB,
                                      TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0) {
                        ROS_ERROR("tjDecompress2 failed: %s", tjGetErrorStr());
                        success = false;
                    }
                }
            }
        } else if (fourcc_ == "YUYV") {
            // 生成 RGB 原图
            image_msg.width  = frame_width_;
            image_msg.height = frame_height_;
            image_msg.encoding = sensor_msgs::image_encodings::RGB8;
            image_msg.step   = frame_width_ * 3;
            image_msg.data.resize(static_cast<size_t>(frame_width_) * frame_height_ * 3);
            convertYUYVtoRGB(static_cast<unsigned char*>(buffer_start_),
                             image_msg.data.data(), frame_width_, frame_height_);

            // 如需要压缩，则压 JPEG
            if (compressed_pub_.getNumSubscribers() > 0) {
                unsigned long jpeg_size = 0;
                unsigned char* jpeg_buffer = nullptr;
                if (tjCompress2(tj_compressor_,
                                image_msg.data.data(), frame_width_, 0, frame_height_,
                                TJPF_RGB,
                                &jpeg_buffer, &jpeg_size,
                                TJSAMP_444, jpeg_quality_, TJFLAG_FASTDCT) != 0) {
                    ROS_ERROR("tjCompress2 failed: %s", tjGetErrorStr());
                    success = false;
                } else {
                    compressed_msg.format = "jpeg";
                    compressed_msg.data.resize(jpeg_size);
                    std::memcpy(compressed_msg.data.data(), jpeg_buffer, jpeg_size);
                    tjFree(jpeg_buffer);
                }
            }
        } else if (fourcc_ == "H264") {
            // 不建议直接用 H264 原始字节，这里与 MJPG 逻辑保持一致（需要旋转就必须解码到 RGB —— 但本例未实现H264软解码）
            // 简化：仅把字节作为“压缩图像”发布（format 标注 h264），如需旋转请改用 MJPG/YUYV。
            compressed_msg.format = "h264";
            compressed_msg.data.resize(bytes);
            std::memcpy(compressed_msg.data.data(), buffer_start_, bytes);
            // 无法旋转（未实现H264->RGB解码），若需要请切换编码格式
        } else {
            ROS_ERROR("Unsupported fourcc: %s", fourcc_.c_str());
            success = false;
        }

        // 4) 填充时间戳与 frame_id
        image_msg.header.stamp = capture_time;
        compressed_msg.header.stamp = capture_time;
        image_msg.header.frame_id = camera_name_;
        compressed_msg.header.frame_id = camera_name_;

        // 5) 归还缓冲
        requeue();
        return success;
    }

    // 在 captureThread 中统一应用旋转和发布逻辑
    void captureThread() {
        ros::Rate rate(publish_rate_);

        while (running_ && camera_connected_) {
            sensor_msgs::Image image_msg;
            sensor_msgs::CompressedImage compressed_msg;

            // 只要需要旋转，就强制解码成 RGB
            const bool force_rgb = (std::fabs(rotate_angle_) > 1e-6);

            if (captureFrame(image_msg, compressed_msg, force_rgb)) {
                // 如果需要旋转（只支持 RGB8）
                if (std::fabs(rotate_angle_) > 1e-6 && !image_msg.data.empty()) {
                    sensor_msgs::Image rotated;
                    rotateImageBilinear(image_msg, rotated, rotate_angle_);
                    image_msg = rotated;

                    // 压缩话题需要发布旋转后版本：重新压 JPEG
                    if (compressed_pub_.getNumSubscribers() > 0) {
                        unsigned long jpeg_size = 0;
                        unsigned char* jpeg_buffer = nullptr;
                        if (tjCompress2(tj_compressor_,
                                        image_msg.data.data(), image_msg.width, 0, image_msg.height,
                                        TJPF_RGB,
                                        &jpeg_buffer, &jpeg_size,
                                        TJSAMP_444, jpeg_quality_, TJFLAG_FASTDCT) != 0) {
                            ROS_ERROR("tjCompress2 (after rotate) failed: %s", tjGetErrorStr());
                        } else {
                            compressed_msg.header = image_msg.header;
                            compressed_msg.format = "jpeg";
                            compressed_msg.data.resize(jpeg_size);
                            std::memcpy(compressed_msg.data.data(), jpeg_buffer, jpeg_size);
                            tjFree(jpeg_buffer);
                        }
                    }
                }

                // 发布原始图像
                if (image_pub_.getNumSubscribers() > 0 && !image_msg.data.empty()) {
                    image_pub_.publish(image_msg);
                }

                // 发布压缩图
                if (compressed_pub_.getNumSubscribers() > 0 && !compressed_msg.data.empty()) {
                    compressed_pub_.publish(compressed_msg);
                }

                // 发布相机信息（必要时缩放）
                sensor_msgs::CameraInfo camera_info = camera_info_manager_->getCameraInfo();
                maybeScaleCameraInfo(camera_info);
                camera_info.header.stamp = (image_msg.data.empty() ? compressed_msg.header.stamp : image_msg.header.stamp);
                camera_info.header.frame_id = (image_msg.data.empty() ? compressed_msg.header.frame_id : image_msg.header.frame_id);
                camera_info_pub_.publish(camera_info);
            } else {
                ROS_WARN("Frame capture failed, triggering reconnect");
                need_reconnect_ = true;
                break;
            }

            rate.sleep();
        }
    }

    void reconnectThread() {
        while (running_) {
            if (need_reconnect_ || !camera_connected_) {
                closeCamera();

                bool connected = false;
                for (const auto& device : video_devices_) {
                    ROS_INFO("Trying to connect to %s (attempt %d/%d)",
                             device.c_str(), retry_count_ + 1, max_retries_);

                    if (openCamera(device)) {
                        connected = true;
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(retry_delay_ * 1000)));
                    if (!running_) break;
                }

                if (connected) {
                    if (capture_thread_.joinable()) capture_thread_.join();
                    capture_thread_ = std::thread(&USBCameraDriver::captureThread, this);
                    need_reconnect_ = false;
                } else {
                    retry_count_++;
                    if (retry_count_ >= max_retries_) {
                        ROS_ERROR("Failed to connect to any camera after %d attempts", max_retries_);
                        running_ = false;
                        break;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void saveImageTimerCallback(const ros::TimerEvent& /*event*/) {
        if (!camera_connected_) return;

        sensor_msgs::Image image_msg;
        sensor_msgs::CompressedImage compressed_msg;

        const bool force_rgb = (std::fabs(rotate_angle_) > 1e-6);  // 保存时也要和发布一致
        if (captureFrame(image_msg, compressed_msg, force_rgb)) {
            if (std::fabs(rotate_angle_) > 1e-6 && !image_msg.data.empty()) {
                sensor_msgs::Image rotated;
                rotateImageBilinear(image_msg, rotated, rotate_angle_);
                image_msg = rotated;

                // 保存压缩图：若 compressed 原本不是旋转版，则重压
                unsigned long jpeg_size = 0;
                unsigned char* jpeg_buffer = nullptr;
                if (tjCompress2(tj_compressor_,
                                image_msg.data.data(), image_msg.width, 0, image_msg.height,
                                TJPF_RGB,
                                &jpeg_buffer, &jpeg_size,
                                TJSAMP_444, jpeg_quality_, TJFLAG_FASTDCT) == 0) {
                    compressed_msg.format = "jpeg";
                    compressed_msg.data.resize(jpeg_size);
                    std::memcpy(compressed_msg.data.data(), jpeg_buffer, jpeg_size);
                    tjFree(jpeg_buffer);
                }
            }
            saveImage(compressed_msg);
        }
    }

    void saveImage(const sensor_msgs::CompressedImage& compressed_msg) {
        // 创建保存目录
        struct stat st;
        if (stat(save_dir_.c_str(), &st) == -1) {
            if (mkdir(save_dir_.c_str(), 0755) == -1) {
                ROS_ERROR("Failed to create directory %s: %s", save_dir_.c_str(), strerror(errno));
                return;
            }
        }

        char filename[512];
        std::snprintf(filename, sizeof(filename), "%s/image_%06d.jpg",
                      save_dir_.c_str(), image_counter_++);

        std::ofstream file(filename, std::ios::binary);
        if (file.is_open()) {
            if (!compressed_msg.data.empty()) {
                file.write(reinterpret_cast<const char*>(compressed_msg.data.data()),
                           static_cast<std::streamsize>(compressed_msg.data.size()));
            }
            file.close();
            ROS_DEBUG("Saved image to %s", filename);
        } else {
            ROS_ERROR("Failed to save image to %s: %s", filename, strerror(errno));
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_sdk");

    try {
        USBCameraDriver driver;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in camera driver: %s", e.what());
        return 1;
    }

    return 0;
}
