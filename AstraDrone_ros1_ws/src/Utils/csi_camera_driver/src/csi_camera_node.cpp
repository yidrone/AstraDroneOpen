// csi_camera_node_gst.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cstring>
#include <thread>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdexcept>

class CSICameraNodeGST {
public:
    CSICameraNodeGST()
        : nh_(),
          pnh_("~"),
          it_(nh_)
    {
        // 参数读取（保持与你原来一致）
        pnh_.param("frame_id", frame_id_, std::string("camera_frame"));
        pnh_.param("capture_width", capture_width_, 640);
        pnh_.param("capture_height", capture_height_, 480);
        pnh_.param("display_width", display_width_, 640);
        pnh_.param("display_height", display_height_, 480);
        pnh_.param("framerate", framerate_, 30);
        pnh_.param("flip_method", flip_method_, 0);
        pnh_.param("sensor_id", sensor_id_, 0);
        pnh_.param("use_gstreamer_pipeline", use_gst_, true);
        pnh_.param("pipeline_override", pipeline_override_, std::string(""));
        pnh_.param("rotate_angle", rotate_angle_, 0);
        pnh_.param("enable_rtsp", enable_rtsp_, false);
        pnh_.param("rtsp_port", rtsp_port_, 8554);
        pnh_.param("rtsp_mount", rtsp_mount_, std::string("/csi"));
        pnh_.param("rtsp_bitrate", rtsp_bitrate_, 2000);
        pnh_.param("rtsp_use_hardware_encoder", rtsp_use_hardware_encoder_, true);

        pnh_.param("camera_name", camera_name_, std::string("csi_cam"));
        pnh_.param("camera_info_url", camera_info_url_, std::string(""));
        pnh_.param("autoscale_camera_info", autoscale_ci_, true);

        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
        cam_pub_ = it_.advertiseCamera("csi_camera/image_raw", 1);

        int argc = 0;
        char** argv = nullptr;
        gst_init(&argc, &argv);

        std::string pipeline_str = createPipeline(true); // 优先 nvargus
        ROS_INFO_STREAM("GStreamer pipeline (first attempt)_: " << pipeline_str);

        // 尝试启动 pipeline（内部会处理错误并在必要时尝试回退）
        if (!tryStartPipeline(pipeline_str)) {
            ROS_FATAL("Failed to start any GStreamer pipeline. Node will throw.");
            throw std::runtime_error("Failed to start GStreamer pipeline");
        }

        // 配置 appsink 回调（在 pipeline 成功启动并有 appsink 时）
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
        if (!appsink_) {
            ROS_FATAL("Failed to get appsink after successful pipeline start!");
            throw std::runtime_error("appsink not found");
        }

        gst_app_sink_set_emit_signals(GST_APP_SINK(appsink_), true);
        gst_app_sink_set_drop(GST_APP_SINK(appsink_), true);
        gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 1);

        GstAppSinkCallbacks cbs = { nullptr, nullptr, &CSICameraNodeGST::onNewSampleStatic };
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &cbs, this, nullptr);

        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1, framerate_)),
                                 &CSICameraNodeGST::publishTimer, this);

        if (enable_rtsp_) {
            setupRtspServer();
        }

        ROS_INFO("CSI Camera node initialized.");
    }

    ~CSICameraNodeGST() {
        ROS_INFO("Shutting down CSI camera node...");

        if (appsink_) {
            gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), nullptr, nullptr, nullptr);
        }

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            GstBus* bus = gst_element_get_bus(pipeline_);
            if (bus) gst_bus_poll(bus, GST_MESSAGE_ELEMENT, 0);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }

        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }

        if (rtsp_appsrc_) {
            gst_object_unref(rtsp_appsrc_);
            rtsp_appsrc_ = nullptr;
        }
        if (rtsp_main_loop_) {
            g_main_loop_quit(rtsp_main_loop_);
        }
        if (rtsp_thread_.joinable()) {
            rtsp_thread_.join();
        }
        if (rtsp_main_loop_) {
            g_main_loop_unref(rtsp_main_loop_);
            rtsp_main_loop_ = nullptr;
        }
        if (rtsp_factory_) {
            g_object_unref(rtsp_factory_);
            rtsp_factory_ = nullptr;
        }
        if (rtsp_server_) {
            g_object_unref(rtsp_server_);
            rtsp_server_ = nullptr;
        }

        ROS_INFO("CSI camera node shutdown complete.");
    }

private:
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher cam_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    ros::Timer timer_;

    std::string frame_id_, camera_name_, camera_info_url_;
    bool autoscale_ci_{true};
    int capture_width_, capture_height_;
    int display_width_, display_height_;
    int framerate_, flip_method_, sensor_id_;
    bool use_gst_{true};
    std::string pipeline_override_;
    int rotate_angle_;
    bool enable_rtsp_{false};
    int rtsp_port_{8554};
    std::string rtsp_mount_{"/csi"};
    int rtsp_bitrate_{2000};
    bool rtsp_use_hardware_encoder_{true};

    GstElement* pipeline_{nullptr};
    GstElement* appsink_{nullptr};
    GstRTSPServer* rtsp_server_{nullptr};
    GstRTSPMediaFactory* rtsp_factory_{nullptr};
    GstAppSrc* rtsp_appsrc_{nullptr};
    GMainLoop* rtsp_main_loop_{nullptr};
    std::thread rtsp_thread_;

    std::mutex mtx_;
    std::vector<uint8_t> last_frame_;
    int last_width_{0}, last_height_{0};
    ros::Time last_stamp_;
    std::mutex rtsp_mtx_;
    uint64_t rtsp_frame_count_{0};
    bool rtsp_media_playing_{false};
    std::atomic_bool rtsp_need_data_{false};

private:
    std::string createPipeline(bool prefer_nvargus) {
        if (!pipeline_override_.empty()) {
            ROS_WARN("Using pipeline_override provided via parameter.");
            return pipeline_override_;
        }

        char pipeline[1024];

        if (prefer_nvargus) {
            // nvarguscamerasrc pipeline (Jetson Argus)
            snprintf(pipeline, sizeof(pipeline),
                     "nvarguscamerasrc sensor-id=%d ! "
                     "video/x-raw(memory:NVMM), width=%d, height=%d, format=NV12, framerate=%d/1 ! "
                     "nvvidconv flip-method=%d ! "
                     "video/x-raw, width=%d, height=%d, format=BGRx ! "
                     "videoconvert ! "
                     "video/x-raw, format=BGR ! "
                     "appsink name=appsink emit-signals=true drop=true max-buffers=1 sync=false",
                     sensor_id_, capture_width_, capture_height_,
                     framerate_, flip_method_,
                     display_width_, display_height_);
        } else {
            // 回退：v4l2src pipeline（适用于非 Jetson 或 nvargus 不可用时）
            // 注意: /dev/video0 可能需要调整，或使用 v4l2loopback
            snprintf(pipeline, sizeof(pipeline),
                     "v4l2src device=/dev/video%d ! "
                     "video/x-raw, width=%d, height=%d, framerate=%d/1 ! "
                     "videoconvert ! video/x-raw, format=BGR ! "
                     "appsink name=appsink emit-signals=true drop=true max-buffers=1 sync=false",
                     sensor_id_, display_width_, display_height_, framerate_);
        }

        return std::string(pipeline);
    }

    void setupRtspServer() {
        std::string launch_pipeline = createRtspPipeline();
        if (launch_pipeline.empty()) {
            ROS_ERROR("RTSP pipeline string is empty. RTSP will be disabled.");
            enable_rtsp_ = false;
            return;
        }

        printLocalIPv4Addresses();
        rtsp_main_loop_ = g_main_loop_new(nullptr, FALSE);
        if (!rtsp_main_loop_) {
            ROS_ERROR("Failed to create RTSP main loop. RTSP will be disabled.");
            enable_rtsp_ = false;
            return;
        }

        rtsp_server_ = gst_rtsp_server_new();
        if (!rtsp_server_) {
            ROS_ERROR("Failed to create RTSP server. RTSP will be disabled.");
            enable_rtsp_ = false;
            return;
        }

        std::string port_str = std::to_string(rtsp_port_);
        gst_rtsp_server_set_service(rtsp_server_, port_str.c_str());

        GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(rtsp_server_);
        rtsp_factory_ = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(rtsp_factory_, launch_pipeline.c_str());
        gst_rtsp_media_factory_set_shared(rtsp_factory_, true);
        g_signal_connect(rtsp_factory_, "media-configure", G_CALLBACK(&CSICameraNodeGST::onRtspMediaConfigureStatic), this);
        gst_rtsp_mount_points_add_factory(mounts, rtsp_mount_.c_str(), rtsp_factory_);
        g_object_unref(mounts);

        if (gst_rtsp_server_attach(rtsp_server_, nullptr) == 0) {
            ROS_ERROR("Failed to attach RTSP server to default context. RTSP will be disabled.");
            enable_rtsp_ = false;
            return;
        }

        rtsp_thread_ = std::thread([this]() {
            g_main_loop_run(rtsp_main_loop_);
        });

        ROS_INFO_STREAM("RTSP server started at rtsp://<ip>:" << rtsp_port_ << rtsp_mount_);
    }

    void printLocalIPv4Addresses() const {
        struct ifaddrs* ifaddr = nullptr;
        if (getifaddrs(&ifaddr) == -1) {
            ROS_WARN("Failed to query local network interfaces for RTSP.");
            return;
        }
        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
                continue;
            }
            auto* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
            char buf[INET_ADDRSTRLEN] = {0};
            if (inet_ntop(AF_INET, &addr->sin_addr, buf, sizeof(buf))) {
                ROS_INFO_STREAM("Local IPv4: " << buf);
            }
        }
        freeifaddrs(ifaddr);
    }

    std::string createRtspPipeline() const {
        char pipeline[1024];
        if (rtsp_use_hardware_encoder_) {
            const int iframe_interval = std::max(1, framerate_ / 2);
            const int bitrate_bps = std::min(rtsp_bitrate_ * 1000, 1000000);
            snprintf(pipeline, sizeof(pipeline),
                     "appsrc name=src is-live=true format=time block=true do-timestamp=true "
                     "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
                     "videoconvert ! video/x-raw,format=NV12 ! "
                     "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
                     "nvv4l2h264enc profile=baseline bitrate=%d insert-sps-pps=1 iframeinterval=%d ! "
                     "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 mtu=1200",
                     display_width_, display_height_, framerate_,
                     bitrate_bps, iframe_interval);
        } else {
            snprintf(pipeline, sizeof(pipeline),
                     "appsrc name=src is-live=true format=time block=true do-timestamp=true "
                     "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
                     "videoconvert ! video/x-raw,format=I420 ! "
                     "x264enc bitrate=%d speed-preset=ultrafast tune=zerolatency key-int-max=%d ! "
                     "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1",
                     display_width_, display_height_, framerate_,
                     rtsp_bitrate_, framerate_);
        }
        return std::string(pipeline);
    }

    static void onRtspMediaConfigureStatic(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data) {
        reinterpret_cast<CSICameraNodeGST*>(user_data)->onRtspMediaConfigure(factory, media);
    }

    static void onRtspMediaStateStatic(GstRTSPMedia* media, GstState state, gpointer user_data) {
        reinterpret_cast<CSICameraNodeGST*>(user_data)->onRtspMediaState(media, state);
    }

    static void onRtspNeedData(GstAppSrc*, guint, gpointer user_data) {
        auto* self = reinterpret_cast<CSICameraNodeGST*>(user_data);
        self->rtsp_need_data_.store(true, std::memory_order_relaxed);
    }

    static void onRtspEnoughData(GstAppSrc*, gpointer user_data) {
        auto* self = reinterpret_cast<CSICameraNodeGST*>(user_data);
        self->rtsp_need_data_.store(false, std::memory_order_relaxed);
    }

    void onRtspMediaConfigure(GstRTSPMediaFactory*, GstRTSPMedia* media) {
        GstElement* element = gst_rtsp_media_get_element(media);
        if (!element) {
            ROS_WARN("RTSP media element is null; cannot find appsrc.");
            return;
        }

        GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "src");
        gst_object_unref(element);
        if (!appsrc) {
            ROS_WARN("Failed to get RTSP appsrc element.");
            return;
        }

        GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                            "format", G_TYPE_STRING, "BGR",
                                            "width", G_TYPE_INT, display_width_,
                                            "height", G_TYPE_INT, display_height_,
                                            "framerate", GST_TYPE_FRACTION, framerate_, 1,
                                            nullptr);
        gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
        gst_caps_unref(caps);

        g_object_set(G_OBJECT(appsrc),
                     "is-live", TRUE,
                     "format", GST_FORMAT_TIME,
                     "block", TRUE,
                     "do-timestamp", TRUE,
                     nullptr);

        GstAppSrcCallbacks callbacks = { &CSICameraNodeGST::onRtspNeedData,
                                         &CSICameraNodeGST::onRtspEnoughData,
                                         nullptr };
        rtsp_need_data_.store(false, std::memory_order_relaxed);
        gst_app_src_set_callbacks(GST_APP_SRC(appsrc), &callbacks, this, nullptr);

        {
            std::lock_guard<std::mutex> lk(rtsp_mtx_);
            if (rtsp_appsrc_) {
                gst_object_unref(rtsp_appsrc_);
            }
            rtsp_appsrc_ = GST_APP_SRC(appsrc);
            rtsp_frame_count_ = 0;
            rtsp_media_playing_ = false;
        }

        g_signal_connect(media, "new-state", G_CALLBACK(&CSICameraNodeGST::onRtspMediaStateStatic), this);

        ROS_INFO("RTSP appsrc configured.");
    }

    void onRtspMediaState(GstRTSPMedia*, GstState state) {
        std::lock_guard<std::mutex> lk(rtsp_mtx_);
        rtsp_media_playing_ = (state == GST_STATE_PLAYING);
        if (!rtsp_media_playing_) {
            rtsp_frame_count_ = 0;
        }
    }

    // 启动 pipeline，并监听 Bus 错误；失败时（nvargus）尝试 v4l2 回退。
    bool tryStartPipeline(const std::string& initial_pipeline) {
        GError* err = nullptr;
        pipeline_ = gst_parse_launch(initial_pipeline.c_str(), &err);
        if (!pipeline_) {
            ROS_FATAL_STREAM("gst_parse_launch failed: " << (err ? err->message : "unknown"));
            if (err) g_error_free(err);
            return false;
        }
        if (err) { g_error_free(err); err = nullptr; }

        ROS_INFO("Pipeline parsed, trying to set PLAYING...");

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            ROS_ERROR("gst_element_set_state returned GST_STATE_CHANGE_FAILURE immediately.");
            printBusErrors(pipeline_);
            cleanupPipeline();
            // 如果这是 nvargus pipeline，尝试回退到 v4l2 pipeline
            ROS_WARN("Attempting v4l2src fallback pipeline...");
            std::string fallback = createPipeline(false);
            ROS_INFO_STREAM("Fallback pipeline: " << fallback);
            err = nullptr;
            pipeline_ = gst_parse_launch(fallback.c_str(), &err);
            if (!pipeline_) {
                ROS_FATAL_STREAM("Fallback gst_parse_launch failed: " << (err ? err->message : "unknown"));
                if (err) g_error_free(err);
                return false;
            }
            if (err) { g_error_free(err); err = nullptr; }

            GstStateChangeReturn ret2 = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
            if (ret2 == GST_STATE_CHANGE_FAILURE) {
                ROS_FATAL("Fallback pipeline failed to PLAY as well.");
                printBusErrors(pipeline_);
                cleanupPipeline();
                return false;
            } else {
                // 等待总线消息短时间以捕获可能的 runtime error
                if (!waitForBusOkOrError(pipeline_, 3000)) {
                    ROS_ERROR("Fallback pipeline reported error during startup.");
                    printBusErrors(pipeline_);
                    cleanupPipeline();
                    return false;
                }
            }
        } else {
            // 如果不是 immediate failure：仍监听几秒钟，看看是否有 Argus runtime 错误（例如 OutputStream 创建失败会在 set_state 后发出错误）
            if (!waitForBusOkOrError(pipeline_, 3000)) {
                ROS_ERROR("Pipeline reported error during startup.");
                printBusErrors(pipeline_);
                cleanupPipeline();
                return false;
            }
        }

        ROS_INFO("GStreamer pipeline set to PLAYING successfully.");
        return true;
    }

    void printBusErrors(GstElement* pipeline) {
        if (!pipeline) return;
        GstBus* bus = gst_element_get_bus(pipeline);
        if (!bus) {
            ROS_WARN("No GST bus available to query errors.");
            return;
        }

        // 非阻塞地拉取错误/消息并打印
        while (true) {
            GstMessage* msg = gst_bus_pop(bus);
            if (!msg) break;

            switch (GST_MESSAGE_TYPE (msg)) {
                case GST_MESSAGE_ERROR: {
                    GError *err = nullptr;
                    gchar *dbg = nullptr;
                    gst_message_parse_error (msg, &err, &dbg);
                    ROS_ERROR_STREAM("GStreamer ERROR: " << (err ? err->message : "unknown"));
                    if (dbg) ROS_ERROR_STREAM("Debug info: " << dbg);
                    if (err) g_error_free(err);
                    if (dbg) g_free(dbg);
                    break;
                }
                case GST_MESSAGE_WARNING: {
                    GError *err = nullptr;
                    gchar *dbg = nullptr;
                    gst_message_parse_warning (msg, &err, &dbg);
                    ROS_WARN_STREAM("GStreamer WARNING: " << (err ? err->message : "unknown"));
                    if (dbg) ROS_WARN_STREAM("Debug info: " << dbg);
                    if (err) g_error_free(err);
                    if (dbg) g_free(dbg);
                    break;
                }
                default:
                    // 其它消息忽略或记录
                    // gchar *str = gst_message_type_get_name(GST_MESSAGE_TYPE(msg));
                    // ROS_INFO_STREAM("GStreamer message: " << str);
                    break;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }

    // 等待最多 timeout_ms 毫秒以捕获 ERROR 或 EOS；无错误则返回 true
    bool waitForBusOkOrError(GstElement* pipeline, int timeout_ms) {
        if (!pipeline) return false;
        GstBus* bus = gst_element_get_bus(pipeline);
        if (!bus) return true; // 无 bus 可以认为 OK（非常罕见）
        GstMessage* msg = gst_bus_timed_pop_filtered(bus, (GstClockTime)(timeout_ms * 1000000LL),
                                                    (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_WARNING));
        if (!msg) {
            gst_object_unref(bus);
            // 超时，没有显式错误，认为启动成功
            return true;
        }

        bool ok = true;
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError *err = nullptr;
                gchar *dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                ROS_ERROR_STREAM("GStreamer ERROR during startup: " << (err ? err->message : "unknown"));
                if (dbg) ROS_ERROR_STREAM("Debug info: " << dbg);
                if (err) g_error_free(err);
                if (dbg) g_free(dbg);
                ok = false;
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError *err = nullptr;
                gchar *dbg = nullptr;
                gst_message_parse_warning(msg, &err, &dbg);
                ROS_WARN_STREAM("GStreamer WARNING during startup: " << (err ? err->message : "unknown"));
                if (dbg) ROS_WARN_STREAM("Debug info: " << dbg);
                if (err) g_error_free(err);
                if (dbg) g_free(dbg);
                break;
            }
            case GST_MESSAGE_EOS:
                ROS_WARN("GStreamer EOS received during startup.");
                break;
            case GST_MESSAGE_STATE_CHANGED:
                // 可用于调试 state 转换
                break;
            default:
                break;
        }
        gst_message_unref(msg);
        gst_object_unref(bus);
        return ok;
    }

    void cleanupPipeline() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

    static GstFlowReturn onNewSampleStatic(GstAppSink* sink, gpointer user_data) {
        return reinterpret_cast<CSICameraNodeGST*>(user_data)->onNewSample(sink);
    }

    void rotateFrame(std::vector<uint8_t>& frame, int& width, int& height) {
        int ang = rotate_angle_ % 360;
        if (ang < 0) ang += 360;
        if (ang == 0) return;

        const int C = 3;
        int W = width, H = height;
        int newW = (ang == 90 || ang == 270) ? H : W;
        int newH = (ang == 90 || ang == 270) ? W : H;

        std::vector<uint8_t> out(newW * newH * C);

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int src = (y * W + x) * C;
                int dx, dy;
                if (ang == 180) {
                    dx = W - 1 - x;
                    dy = H - 1 - y;
                } else if (ang == 90) {
                    dx = H - 1 - y;
                    dy = x;
                } else { // 270
                    dx = y;
                    dy = W - 1 - x;
                }
                int dst = (dy * newW + dx) * C;
                out[dst] = frame[src];
                out[dst + 1] = frame[src + 1];
                out[dst + 2] = frame[src + 2];
            }
        }
        frame.swap(out);
        width = newW;
        height = newH;
    }

    GstFlowReturn onNewSample(GstAppSink* sink) {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            ROS_DEBUG("onNewSample: gst_app_sink_pull_sample returned null");
            return GST_FLOW_OK;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);

        if (!buffer || !caps) {
            ROS_WARN("onNewSample: missing buffer or caps");
            if (sample) gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        GstVideoInfo info;
        if (!gst_video_info_from_caps(&info, caps)) {
            ROS_WARN("onNewSample: failed to get video info from caps");
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        int width = info.width;
        int height = info.height;
        int stride = info.stride[0];

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            ROS_WARN("onNewSample: gst_buffer_map failed");
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        // 基本数据长度检查
        size_t expected_line = (size_t)width * 3;
        if ((size_t)stride < expected_line) {
            ROS_WARN_STREAM("Stride (" << stride << ") < expected line bytes (" << expected_line << "). Caps/format mismatch?");
            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        // map.size 可能大于也可能小于 width*height*3，检查避免越界
        size_t needed_size = (size_t)height * stride;
        if (map.size < needed_size) {
            ROS_WARN_STREAM("Mapped buffer size (" << map.size << ") smaller than expected (" << needed_size << ").");
            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        {
            std::lock_guard<std::mutex> lk(mtx_);
            last_width_ = width;
            last_height_ = height;
            last_stamp_ = ros::Time::now();
            last_frame_.resize(width * height * 3);

            // 逐行拷贝（从 map.data 中复制 width*3 字节到 contiguous BGR buffer）
            for (int y = 0; y < height; y++) {
                uint8_t* srcrow = map.data + (size_t)y * stride;
                uint8_t* dstrow = &last_frame_[(size_t)y * width * 3];
                std::memcpy(dstrow, srcrow, (size_t)width * 3);
            }

            // 旋转（如果配置了 rotate_angle）
            rotateFrame(last_frame_, last_width_, last_height_);
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    sensor_msgs::CameraInfo makeCameraInfo(const ros::Time& stamp, int w, int h) {
        sensor_msgs::CameraInfo ci = cinfo_->getCameraInfo();
        ci.header.stamp = stamp;
        ci.header.frame_id = frame_id_;
        ci.width = w;
        ci.height = h;
        return ci;
    }

    void publishTimer(const ros::TimerEvent&) {
        std::vector<uint8_t> frame;
        int w, h;
        ros::Time t;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (last_frame_.empty()) return;
            frame = last_frame_;
            w = last_width_;
            h = last_height_;
            t = last_stamp_;
        }

        sensor_msgs::Image img;
        img.header.stamp = t;
        img.header.frame_id = frame_id_;
        img.width = w;
        img.height = h;
        img.encoding = "bgr8";
        img.step = w * 3;
        img.data = frame;

        cam_pub_.publish(img, makeCameraInfo(t, w, h));
        publishRtspFrame(frame, w, h);
    }

    void publishRtspFrame(const std::vector<uint8_t>& frame, int w, int h) {
        if (!enable_rtsp_) return;
        GstAppSrc* appsrc = nullptr;
        bool media_playing = false;
        {
            std::lock_guard<std::mutex> lk(rtsp_mtx_);
            appsrc = rtsp_appsrc_;
            media_playing = rtsp_media_playing_;
            if (appsrc) {
                gst_object_ref(appsrc);
            }
        }
        if (!appsrc) return;
        if (!rtsp_need_data_.load(std::memory_order_relaxed)) {
            gst_object_unref(appsrc);
            return;
        }

        if (w != display_width_ || h != display_height_) {
            ROS_WARN_THROTTLE(5.0, "RTSP frame size does not match display size; skipping RTSP frame.");
            gst_object_unref(appsrc);
            return;
        }

        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, frame.size(), nullptr);
        if (!buffer) {
            ROS_WARN_THROTTLE(5.0, "Failed to allocate GStreamer buffer for RTSP.");
            gst_object_unref(appsrc);
            return;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            ROS_WARN_THROTTLE(5.0, "Failed to map RTSP buffer.");
            gst_buffer_unref(buffer);
            gst_object_unref(appsrc);
            return;
        }
        std::memcpy(map.data, frame.data(), frame.size());
        gst_buffer_unmap(buffer, &map);

        GstClockTime duration = gst_util_uint64_scale_int(1, GST_SECOND, std::max(1, framerate_));
        {
            std::lock_guard<std::mutex> lk(rtsp_mtx_);
            GST_BUFFER_PTS(buffer) = rtsp_frame_count_ * duration;
            GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
            GST_BUFFER_DURATION(buffer) = duration;
            rtsp_frame_count_++;
        }

        GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
        if (ret != GST_FLOW_OK) {
            ROS_WARN_THROTTLE(5.0, "RTSP appsrc push_buffer failed: %s (%d).", gst_flow_get_name(ret), ret);
        }

        gst_object_unref(appsrc);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "csi_camera_node_gst");

    while (ros::ok()) {
        try {
            CSICameraNodeGST node;

            ros::AsyncSpinner spinner(1);
            spinner.start();
            ros::waitForShutdown();

            break;

        } catch (const std::exception& e) {
            ROS_FATAL_STREAM("Exception starting node: " << e.what());
            ros::Duration(2.0).sleep();
        }
    }
    return 0;
}
