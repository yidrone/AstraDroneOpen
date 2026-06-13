//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "pub_handler.h"
#include "livox_lidar_api.h"
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>
#include <utility>

namespace livox_ros {

PubHandler &pub_handler() {
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() {
}

void PubHandler::Uninit() {
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();

  if (point_process_thread_ &&
    point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

void PubHandler::RequestExit() {
  is_quit_.store(true);
}

void PubHandler::SetPointCloudConfig(const double publish_freq) {
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10;
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetNoSyncConfig(const std::string& time_mode,
                                 const std::string& publish_mode,
                                 uint32_t min_points_per_frame) {
  const NoSyncTimeMode old_time_mode = nosync_time_mode_;
  const NoSyncPublishMode old_publish_mode = nosync_publish_mode_;

  // NoSync 默认不再依赖系统时间；legacy_system 只用于和官方行为 A/B 对比。
  if (time_mode == "legacy_system") {
    nosync_time_mode_ = NoSyncTimeMode::kLegacySystem;
  } else if (time_mode == "steady_raw") {
    nosync_time_mode_ = NoSyncTimeMode::kSteadyRaw;
  } else {
    WarnEvery("bad_nosync_time_mode",
              "Unknown nosync_time_mode '" + time_mode + "', using steady_raw.");
    nosync_time_mode_ = NoSyncTimeMode::kSteadyRaw;
  }

  if (publish_mode == "legacy_timer") {
    nosync_publish_mode_ = NoSyncPublishMode::kLegacyTimer;
  } else if (publish_mode == "sensor_time") {
    nosync_publish_mode_ = NoSyncPublishMode::kSensorTime;
  } else {
    WarnEvery("bad_nosync_publish_mode",
              "Unknown nosync_publish_mode '" + publish_mode + "', using sensor_time.");
    nosync_publish_mode_ = NoSyncPublishMode::kSensorTime;
  }

  min_points_per_frame_ = min_points_per_frame;
  std::cout << "Livox NoSync config: time_mode=" << time_mode
            << ", publish_mode=" << publish_mode
            << ", min_points_per_frame=" << min_points_per_frame_ << std::endl;

  // 运行中切换 NoSync 策略时，丢弃已累积的旧时间域点，避免混帧。
  if (old_time_mode != nosync_time_mode_ || old_publish_mode != nosync_publish_mode_) {
    std::vector<uint32_t> handles;
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      for (const auto& state : timestamp_states_) {
        handles.push_back(state.first);
      }
    }
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      for (const auto& handler : lidar_process_handlers_) {
        if (std::find(handles.begin(), handles.end(), handler.first) == handles.end()) {
          handles.push_back(handler.first);
        }
      }
    }
    for (uint32_t handle : handles) {
      ResetLidarState(handle, "NoSync config changed");
    }
  }
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
}

void PubHandler::AddLidarsExtParam(LidarExtParameter& lidar_param) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t id = 0;
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id);
  lidar_extrinsics_[id] = lidar_param;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::ResetLidarState(uint32_t handle, const std::string& reason) {
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    HandleTimestampState& state = timestamp_states_[handle];
    // 保留 last_*_published，只重置 raw/domain 状态；下一次 re-anchor 会从更大的 ROS 时间开始。
    state.domain = TimestampDomain::kUnknown;
    state.nosync_converter.Reset();
    state.has_last_lidar_raw = false;
    state.last_lidar_raw = 0;
    state.has_last_imu_raw = false;
    state.last_imu_raw = 0;
  }

  ResetPendingPointData(handle);
  WarnEvery("reset_handle_" + std::to_string(handle),
            "Reset Livox timestamp state for handle " + std::to_string(handle) +
            ": " + reason);
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self || !data) {
    return;
  }

  if (data->dot_num == 0) {
    WarnEvery("drop_zero_dot_" + std::to_string(handle),
              "Drop Livox packet with dot_num == 0, handle " + std::to_string(handle));
    return;
  }

  // data->length 是完整 UDP payload 长度，减去 flexible-array 前的头部得到点数据长度。
  const uint32_t packet_header_len = sizeof(LivoxLidarEthernetPacket) - 1;
  if (data->length < packet_header_len) {
    WarnEvery("drop_bad_len_header_" + std::to_string(handle),
              "Drop Livox packet with invalid length header, handle " + std::to_string(handle));
    return;
  }
  const uint32_t payload_len = data->length - packet_header_len;
  const uint32_t expected_point_size = GetPointPayloadSize(data->data_type);
  if (expected_point_size > 0 &&
      payload_len < static_cast<uint64_t>(data->dot_num) * expected_point_size) {
    WarnEvery("drop_bad_len_payload_" + std::to_string(handle),
              "Drop Livox packet with short payload, handle " + std::to_string(handle));
    return;
  }

  const uint64_t raw_stamp = GetEthPacketRawTimestamp(data->timestamp, sizeof(data->timestamp));

  if (data->data_type == kLivoxLidarImuData) {
    if (payload_len < sizeof(RawImuPoint)) {
      WarnEvery("drop_bad_imu_len_" + std::to_string(handle),
                "Drop Livox IMU packet with short payload, handle " + std::to_string(handle));
      return;
    }
    TimestampResult timestamp = self->ConvertPacketTimestamp(
        handle, data->time_type, raw_stamp, PacketSource::kImu);
    if (timestamp.reset_frame) {
      // IMU 先发现 reset/切换时，也要清掉 LiDAR 已累积的过渡帧。
      self->ResetPendingPointData(handle);
    }
    if (!timestamp.valid || !self->ShouldPublishImu(handle, timestamp.stamp, data->time_type)) {
      return;
    }

    if (self->imu_callback_) {
      RawImuPoint* imu = (RawImuPoint*) data->data;
      ImuData imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = timestamp.stamp;
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
      self->MarkImuPublished(handle, timestamp.stamp);
    }
    return;
  }

  TimestampResult timestamp = self->ConvertPacketTimestamp(
      handle, data->time_type, raw_stamp, PacketSource::kLidar);
  if (timestamp.reset_frame) {
    // sync/nosync 切换或 raw rollback 后，旧点云不能和新时间域拼成一帧。
    self->ResetPendingPointData(handle);
  }
  if (!timestamp.valid) {
    return;
  }

  RawPacket packet = {};
  packet.handle = handle;
  packet.lidar_type = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false;
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360 ||
             dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360s) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.timestamp_type = data->time_type;
  packet.point_num = data->dot_num;
  packet.raw_time_stamp = raw_stamp;
  // time_interval 单位是 0.1us，因此乘 100 转 ns；使用 uint64 防溢出。
  packet.point_interval = (static_cast<uint64_t>(data->time_interval) * 100ULL) / data->dot_num;  // ns
  packet.time_stamp = timestamp.stamp;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + payload_len);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
  }
  self->packet_condition_.notify_one();

  return;
}

void PubHandler::PublishPointCloud() {
  //publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

void PubHandler::CheckTimer(uint32_t id, uint8_t timestamp_type) {
  const bool synced = IsSyncedTimestamp(timestamp_type);
  if (!synced && nosync_publish_mode_ == NoSyncPublishMode::kLegacyTimer) {
    // 回滚路径：复用官方基于主机 timer 的切帧行为，便于现场对比。
    PublishLegacyTimerFrames();
    return;
  }

  TryPublishFrame(id, timestamp_type, synced);
}

bool PubHandler::TryPublishFrame(uint32_t id, uint8_t timestamp_type, bool require_sync_modulo) {
  LidarPubHandler* process_handler = nullptr;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    auto iter = lidar_process_handlers_.find(id);
    if (iter == lidar_process_handlers_.end()) {
      return false;
    }
    process_handler = iter->second.get();
  }

  const uint64_t base_time = process_handler->GetLidarBaseTime();
  const uint64_t recent_time = process_handler->GetRecentTimeStamp();
  if (base_time == 0 || recent_time == 0) {
    return false;
  }
  if (recent_time < base_time) {
    ResetPendingPointData(id);
    WarnEvery("drop_frame_time_reverse_" + std::to_string(id),
              "Drop Livox frame with reversed point timestamps, handle " + std::to_string(id));
    return false;
  }

  if (require_sync_modulo) {
    // 已同步时间戳仍沿用官方按同步时间对齐的发布节拍。
    const uint64_t recent_time_ms = recent_time / kRatioOfMsToNs;
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return false;
    }
  }

  const uint64_t diff = recent_time - base_time;
  if (diff < publish_interval_tolerance_) {
    return false;
  }

  // 默认 NoSync 路径按 sensor timestamp 累计跨度切帧，不受系统时间跳变影响。
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    points_[id].clear();
    process_handler->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      return false;
    }

    if (min_points_per_frame_ > 0 && points_[id].size() < min_points_per_frame_) {
      WarnEvery("drop_tiny_frame_" + std::to_string(id),
                "Drop Livox tiny frame, handle " + std::to_string(id) +
                ", points " + std::to_string(points_[id].size()));
      points_[id].clear();
      return false;
    }

    if (!ShouldPublishLidarFrame(id, base_time, recent_time, timestamp_type)) {
      points_[id].clear();
      return false;
    }

    frame_.lidar_num = 0;
    frame_.base_time[frame_.lidar_num] = base_time;
    PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;
    lidar_point.handle = id;
    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    frame_.lidar_num++;

    PublishPointCloud();
    frame_.lidar_num = 0;
  }
  MarkLidarPublished(id, base_time, recent_time);
  return true;
}

void PubHandler::PublishLegacyTimerFrames() {
  auto now_time = std::chrono::high_resolution_clock::now();
  if (!legacy_timer_initialized_) {
    last_pub_time_ = now_time;
    legacy_timer_initialized_ = true;
    return;
  }
  if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
    return;
  }
  last_pub_time_ += std::chrono::nanoseconds(publish_interval_);

  std::vector<std::pair<uint32_t, LidarPubHandler*>> handlers;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    for (const auto& process_handler : lidar_process_handlers_) {
      handlers.push_back(std::make_pair(process_handler.first, process_handler.second.get()));
    }
  }

  std::vector<uint32_t> published_handles;
  std::vector<uint64_t> published_base_times;
  std::vector<uint64_t> published_end_times;
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    frame_.lidar_num = 0;
    for (const auto& handler : handlers) {
      const uint32_t handle = handler.first;
      LidarPubHandler* process_handler = handler.second;
      if (process_handler == nullptr) {
        continue;
      }

      const uint64_t base_time = process_handler->GetLidarBaseTime();
      const uint64_t recent_time = process_handler->GetRecentTimeStamp();
      if (base_time == 0 || recent_time < base_time) {
        continue;
      }
      points_[handle].clear();
      process_handler->GetLidarPointClouds(points_[handle]);
      if (points_[handle].empty()) {
        continue;
      }
      if (min_points_per_frame_ > 0 && points_[handle].size() < min_points_per_frame_) {
        points_[handle].clear();
        continue;
      }
      if (!ShouldPublishLidarFrame(handle, base_time, recent_time, kTimestampTypeNoSync)) {
        points_[handle].clear();
        continue;
      }

      frame_.base_time[frame_.lidar_num] = base_time;
      PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;
      lidar_point.handle = handle;
      lidar_point.points_num = points_[handle].size();
      lidar_point.points = points_[handle].data();
      frame_.lidar_num++;
      published_handles.push_back(handle);
      published_base_times.push_back(base_time);
      published_end_times.push_back(recent_time);
    }

    if (frame_.lidar_num != 0) {
      PublishPointCloud();
    }
    frame_.lidar_num = 0;
  }
  for (size_t i = 0; i < published_handles.size(); ++i) {
    MarkLidarPublished(published_handles[i], published_base_times[i], published_end_times[i]);
  }
}

void PubHandler::RawDataProcess() {
  RawPacket raw_data;
  while (!is_quit_.load()) {
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
        if (raw_packet_queue_.empty()) {
          continue;
        }
      }
      raw_data = raw_packet_queue_.front();
      raw_packet_queue_.pop_front();
    }
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    LidarPubHandler* process_handler = nullptr;
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
        lidar_process_handlers_[id].reset(new LidarPubHandler());
      }
      process_handler = lidar_process_handlers_[id].get();
      if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
        process_handler->SetLidarsExtParam(lidar_extrinsics_[id]);
      }
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer(id, raw_data.timestamp_type);
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

uint64_t PubHandler::GetEthPacketRawTimestamp(uint8_t* time_stamp, uint8_t size) {
  uint64_t stamp = 0;
  if (time_stamp == nullptr) {
    return stamp;
  }
  const uint8_t copy_size = std::min<uint8_t>(size, sizeof(stamp));
  memcpy(&stamp, time_stamp, copy_size);
  return stamp;
}

bool PubHandler::IsSyncedTimestamp(uint8_t timestamp_type) {
  return timestamp_type == kTimestampTypeGptpOrPtp ||
         timestamp_type == kTimestampTypeGps;
}

uint32_t PubHandler::GetPointPayloadSize(uint8_t data_type) {
  switch (data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      return sizeof(LivoxLidarCartesianHighRawPoint);
    case kLivoxLidarCartesianCoordinateLowData:
      return sizeof(LivoxLidarCartesianLowRawPoint);
    case kLivoxLidarSphericalCoordinateData:
      return sizeof(LivoxLidarSpherPoint);
    default:
      return 0;
  }
}

uint64_t PubHandler::GetSteadyWallTimeNs() {
  using SteadyClock = std::chrono::steady_clock;
  using SystemClock = std::chrono::system_clock;
  static const auto steady_start = SteadyClock::now();
  static const auto system_start = SystemClock::now();
  const auto steady_elapsed = SteadyClock::now() - steady_start;
  const auto monotonic_wall = system_start +
      std::chrono::duration_cast<SystemClock::duration>(steady_elapsed);
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          monotonic_wall.time_since_epoch()).count());
}

void PubHandler::WarnEvery(const std::string& key, const std::string& msg) {
  using SteadyClock = std::chrono::steady_clock;
  static std::mutex warn_mutex;
  static std::map<std::string, SteadyClock::time_point> last_warn_time;
  const auto now = SteadyClock::now();
  std::lock_guard<std::mutex> lock(warn_mutex);
  auto iter = last_warn_time.find(key);
  if (iter == last_warn_time.end() || now - iter->second > std::chrono::seconds(5)) {
    std::cout << msg << std::endl;
    last_warn_time[key] = now;
  }
}

PubHandler::TimestampResult PubHandler::ConvertPacketTimestamp(uint32_t handle,
                                                               uint8_t timestamp_type,
                                                               uint64_t raw_stamp,
                                                               PacketSource source) {
  TimestampResult result;
  if (timestamp_type != kTimestampTypeNoSync && !IsSyncedTimestamp(timestamp_type)) {
    WarnEvery("drop_unknown_time_type_" + std::to_string(handle),
              "Drop Livox packet with unknown timestamp type " +
              std::to_string(static_cast<int>(timestamp_type)) +
              ", handle " + std::to_string(handle));
    return result;
  }

  if (timestamp_type == kTimestampTypeNoSync &&
      nosync_time_mode_ == NoSyncTimeMode::kLegacySystem) {
    // legacy_system 故意使用系统时间，保留官方原始行为用于 A/B 测试。
    result.valid = true;
    result.stamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    return result;
  }

  std::lock_guard<std::mutex> lock(time_mutex_);
  HandleTimestampState& state = timestamp_states_[handle];
  const TimestampDomain new_domain = IsSyncedTimestamp(timestamp_type) ?
      TimestampDomain::kSynced : TimestampDomain::kNoSync;

  if (state.domain != TimestampDomain::kUnknown && state.domain != new_domain) {
    result.reset_frame = true;
    // 时间域切换后不能继续沿用旧 anchor；同步 raw timestamp 本身不做转换。
    state.nosync_converter.Reset();
    state.has_last_lidar_raw = false;
    state.last_lidar_raw = 0;
    state.has_last_imu_raw = false;
    state.last_imu_raw = 0;
    WarnEvery("timestamp_domain_switch_" + std::to_string(handle),
              "Livox timestamp domain switch, handle " + std::to_string(handle));
  }
  state.domain = new_domain;

  if (new_domain == TimestampDomain::kSynced) {
    // PTP/GPS/gPTP 分支保持 raw packet timestamp，不转换、不 clamp。
    result.valid = true;
    result.stamp = raw_stamp;
    return result;
  }

  bool& has_last_raw = (source == PacketSource::kLidar) ?
      state.has_last_lidar_raw : state.has_last_imu_raw;
  uint64_t& last_raw = (source == PacketSource::kLidar) ?
      state.last_lidar_raw : state.last_imu_raw;
  if (has_last_raw && raw_stamp < last_raw) {
    result.reset_frame = true;
    // MID360 重启或采样重启会让 power-on raw ns 回退，需要重新建 anchor。
    state.nosync_converter.Reset();
    state.has_last_lidar_raw = false;
    state.last_lidar_raw = 0;
    state.has_last_imu_raw = false;
    state.last_imu_raw = 0;
    WarnEvery("nosync_raw_rollback_" + std::to_string(handle),
              "Livox NoSync raw timestamp rollback/reset, handle " +
              std::to_string(handle));
  }

  if (!state.nosync_converter.initialized) {
    const uint64_t monotonic_wall = GetSteadyWallTimeNs();
    uint64_t min_new_domain_start = 0;
    if (state.has_last_any_published) {
      min_new_domain_start = state.last_any_published == std::numeric_limits<uint64_t>::max() ?
          state.last_any_published : state.last_any_published + 1;
    }
    state.nosync_converter.raw_anchor = raw_stamp;
    // re-anchor 时保证后续整个 NoSync 时间域大于该 handle 已发布的 LiDAR/IMU stamp。
    state.nosync_converter.ros_anchor = std::max(monotonic_wall, min_new_domain_start);
    state.nosync_converter.initialized = true;
  }

  if (raw_stamp < state.nosync_converter.raw_anchor) {
    WarnEvery("drop_pre_anchor_" + std::to_string(handle),
              "Drop Livox NoSync packet older than converter anchor, handle " +
              std::to_string(handle));
    return result;
  }

  const uint64_t delta = raw_stamp - state.nosync_converter.raw_anchor;
  const uint64_t max_stamp = std::numeric_limits<uint64_t>::max();
  result.valid = true;
  result.stamp = delta > max_stamp - state.nosync_converter.ros_anchor ?
      max_stamp : state.nosync_converter.ros_anchor + delta;

  has_last_raw = true;
  last_raw = raw_stamp;
  return result;
}

void PubHandler::ResetPendingPointData(uint32_t handle) {
  {
    std::lock_guard<std::mutex> lock(packet_mutex_);
    // 只清目标 handle，避免多雷达场景下误删其它 handle 的数据。
    std::deque<RawPacket> kept_packets;
    while (!raw_packet_queue_.empty()) {
      RawPacket packet = raw_packet_queue_.front();
      raw_packet_queue_.pop_front();
      if (packet.handle != handle) {
        kept_packets.push_back(std::move(packet));
      }
    }
    raw_packet_queue_.swap(kept_packets);
  }

  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    auto iter = lidar_process_handlers_.find(handle);
    if (iter != lidar_process_handlers_.end() && iter->second) {
      iter->second->ClearPointClouds();
    }
  }
  {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    points_[handle].clear();
    frame_.lidar_num = 0;
  }
}

bool PubHandler::ShouldPublishLidarFrame(uint32_t handle,
                                         uint64_t base_time,
                                         uint64_t end_time,
                                         uint8_t timestamp_type) {
  std::lock_guard<std::mutex> lock(time_mutex_);
  HandleTimestampState& state = timestamp_states_[handle];
  if (state.has_last_lidar_published && base_time <= state.last_lidar_published) {
    // 同步分支也不静默发布 loopback，避免 FAST-LIO 清 buffer 后停住。
    const std::string sync_text = IsSyncedTimestamp(timestamp_type) ? "synced" : "NoSync";
    WarnEvery("drop_lidar_loopback_" + std::to_string(handle),
              "Drop Livox " + sync_text + " lidar frame timestamp loopback, handle " +
              std::to_string(handle));
    return false;
  }
  if (end_time < base_time) {
    WarnEvery("drop_lidar_bad_end_" + std::to_string(handle),
              "Drop Livox lidar frame with invalid end time, handle " +
              std::to_string(handle));
    return false;
  }
  return true;
}

bool PubHandler::ShouldPublishImu(uint32_t handle, uint64_t timestamp, uint8_t timestamp_type) {
  std::lock_guard<std::mutex> lock(time_mutex_);
  HandleTimestampState& state = timestamp_states_[handle];
  if (state.has_last_imu_published && timestamp <= state.last_imu_published) {
    // IMU loopback 会导致 FAST-LIO 同步失败，这里直接丢弃并节流报警。
    const std::string sync_text = IsSyncedTimestamp(timestamp_type) ? "synced" : "NoSync";
    WarnEvery("drop_imu_loopback_" + std::to_string(handle),
              "Drop Livox " + sync_text + " IMU timestamp loopback, handle " +
              std::to_string(handle));
    return false;
  }
  return true;
}

void PubHandler::MarkLidarPublished(uint32_t handle, uint64_t base_time, uint64_t end_time) {
  std::lock_guard<std::mutex> lock(time_mutex_);
  HandleTimestampState& state = timestamp_states_[handle];
  state.has_last_lidar_published = true;
  state.last_lidar_published = base_time;
  state.last_lidar_frame_end = end_time;
  const uint64_t visible_end = std::max(base_time, end_time);
  if (!state.has_last_any_published || visible_end > state.last_any_published) {
    state.last_any_published = visible_end;
    state.has_last_any_published = true;
  }
}

void PubHandler::MarkImuPublished(uint32_t handle, uint64_t timestamp) {
  std::lock_guard<std::mutex> lock(time_mutex_);
  HandleTimestampState& state = timestamp_states_[handle];
  state.has_last_imu_published = true;
  state.last_imu_published = timestamp;
  if (!state.has_last_any_published || timestamp > state.last_any_published) {
    state.last_any_published = timestamp;
    state.has_last_any_published = true;
  }
}

/*******************************/
/*  LidarPubHandler Definitions*/
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds.swap(points_clouds_);
}

void LidarPubHandler::ClearPointClouds() {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds_.clear();
}

uint64_t LidarPubHandler::GetRecentTimeStamp() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_clouds_.size();
}

//convert to standard format and extrinsic compensate
void LidarPubHandler::PointCloudProcess(RawPacket & pkt) {
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt) {
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt) {
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) {
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

} // namespace livox_ros
