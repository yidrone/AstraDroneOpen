代码修改摘要 (Code Modifications)

文件: include/orbbec_camera/ob_camera_node.h

修改 onNewFrameCallback 函数声明，增加 forced_time 参数，用于接收强制同步的时间戳。
C++

```C++
// void onNewFrameCallback(std::shared_ptr<ob::Frame> frame, const stream_index_pair& stream_index);

// 修改后
void onNewFrameCallback(std::shared_ptr<ob::Frame> frame,
const stream_index_pair& stream_index,
ros::Time forced_time = ros::Time(0));
```

文件: src/ob_camera_node.cpp
A. 修改 onNewFrameCallback 实现
使其优先使用传入的 forced_time。
```C++

void OBCameraNode::onNewFrameCallback(std::shared_ptr<ob::Frame> frame,
const stream_index_pair& stream_index,
ros::Time forced_time) {
// ... (检查 frame 是否为空)

// --- 修改开始 ---
ros::Time timestamp;
auto frame_timestamp = getFrameTimestampUs(frame); // 仅保留用于FPS计算

if (!forced_time.isZero()) {
timestamp = forced_time; // 使用强制同步的时间 (来自 Depth)
} else {
timestamp = fromUsToROSTime(frame_timestamp); // 使用默认时间
}
// --- 修改结束 ---

// ... (后续使用 timestamp 填充 header.stamp)
}
```
B. 修改 RGB 处理线程 onNewColorFrameCallback

这是核心逻辑：提取 Depth 时间 -> 传给 RGB -> 丢弃孤立帧。
```C++
void OBCameraNode::onNewColorFrameCallback() {
while (enable_stream_[COLOR] && ...) {
// ... (获取 frameSet)

    auto depth_frame = frameSet->depthFrame();
    auto color_frame = frameSet->colorFrame();

    // 仅当两者都存在时才发布 (严格同频)
    if (depth_frame && color_frame) {
        rgb_is_decoded_ = decodeColorFrameToBuffer(color_frame, rgb_buffer_);
        publishPointCloud(frameSet);

        // 1. 获取基准时间 (Depth)
        uint64_t depth_ts_us = getFrameTimestampUs(depth_frame);
        auto depth_ros_time = fromUsToROSTime(depth_ts_us);

        // 2. 调试日志：打印物理时间差
        uint64_t color_ts_us = getFrameTimestampUs(color_frame);
        double diff_ms = (static_cast<long long>(depth_ts_us) - static_cast<long long>(color_ts_us)) / 1000.0;
        ROS_DEBUG_STREAM("[Sync] Depth vs Color | Diff: " << diff_ms << " ms");

        // 3. 发布 RGB (传入 Depth 时间)
        onNewFrameCallback(color_frame, IMAGE_STREAMS.at(0), depth_ros_time);
    } 
    // else { 直接丢弃，不进入 else 分支 }
}
}
```
(注：同样的逻辑也应用到了 onNewLeftColorFrameCallback 和 onNewRightColorFrameCallback 中)

整体上进行了rgb图像的软同步。Gemini335只支持深度图的硬件触发，因此修改SDK，修改距离深度图时间戳最近的彩色帧。
启动方式依旧不变，但是会影响Standalone等模式的正常使用。

对于启动文件的参数，需要把触发捕获帧数（frames_per_trigger）设置为1（默认为2）。