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

#ifndef LIVOX_DRIVER_PUB_HANDLER_H_
#define LIVOX_DRIVER_PUB_HANDLER_H_

#include <atomic>
#include <chrono>
#include <cstring>
#include <condition_variable> // std::condition_variable
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>              // std::mutex
#include <string>
#include <thread>

#include "livox_lidar_api.h"
#include "comm/comm.h"

namespace livox_ros {

class LidarPubHandler {
 public:
  LidarPubHandler();
  ~ LidarPubHandler() {}

  void PointCloudProcess(RawPacket& pkt);
  void SetLidarsExtParam(LidarExtParameter param);
  void GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds);
  void ClearPointClouds();

  uint64_t GetRecentTimeStamp();
  uint32_t GetLidarPointCloudsSize();
  uint64_t GetLidarBaseTime();

 private:
  void LivoxLidarPointCloudProcess(RawPacket & pkt);
  void ProcessCartesianHighPoint(RawPacket & pkt);
  void ProcessCartesianLowPoint(RawPacket & pkt);
  void ProcessSphericalPoint(RawPacket & pkt);
  std::vector<PointXyzlt> points_clouds_;
  ExtParameterDetailed extrinsic_ = {
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 1},
      {0, 0, 1}
    }
  };
  std::mutex mutex_;
  std::atomic_bool is_set_extrinsic_params_;
};
  
class PubHandler {
 public:
  using PointCloudsCallback = std::function<void(PointFrame*, void *)>;
  using ImuDataCallback = std::function<void(ImuData*, void*)>;
  using TimePoint = std::chrono::high_resolution_clock::time_point;

  PubHandler() {}

  ~ PubHandler() { Uninit(); }

  void Uninit();
  void RequestExit();
  void Init();
  void SetPointCloudConfig(const double publish_freq);
  // NoSync 修复开关：默认走 raw sensor time，保留 legacy 路径便于现场 A/B。
  void SetNoSyncConfig(const std::string& time_mode,
                       const std::string& publish_mode,
                       uint32_t min_points_per_frame);
  // 断连重连、采样重启、时间域切换时清掉中间帧，避免 FAST-LIO 看到混合时间域。
  void ResetLidarState(uint32_t handle, const std::string& reason);
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data);
  void AddLidarsExtParam(LidarExtParameter& extrinsic_params);
  void ClearAllLidarsExtrinsicParams();
  void SetImuDataCallback(ImuDataCallback cb, void* client_data);

 private:
  //thread to process raw data
  void RawDataProcess();
  std::atomic<bool> is_quit_{false};
  std::shared_ptr<std::thread> point_process_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;
  std::mutex handler_mutex_;
  std::mutex time_mutex_;
  std::mutex publish_mutex_;

  //publish callback
  void CheckTimer(uint32_t id, uint8_t timestamp_type);
  void PublishPointCloud();
  void PublishLegacyTimerFrames();
  bool TryPublishFrame(uint32_t id, uint8_t timestamp_type, bool require_sync_modulo);
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data);
  
  static bool GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id);
  static uint64_t GetEthPacketRawTimestamp(uint8_t* time_stamp, uint8_t size);
  static bool IsSyncedTimestamp(uint8_t timestamp_type);
  static uint32_t GetPointPayloadSize(uint8_t data_type);
  static uint64_t GetSteadyWallTimeNs();

  enum class NoSyncTimeMode {
    kSteadyRaw,
    kLegacySystem,
  };

  enum class NoSyncPublishMode {
    kSensorTime,
    kLegacyTimer,
  };

  enum class TimestampDomain {
    kUnknown,
    kNoSync,
    kSynced,
  };

  enum class PacketSource {
    kLidar,
    kImu,
  };

  // timestamp.valid=false 表示该包不能发布；reset_frame=true 表示要丢弃过渡帧。
  struct TimestampResult {
    bool valid = false;
    bool reset_frame = false;
    uint64_t stamp = 0;
  };

  // MID360 NoSync 的 raw timestamp 是 LiDAR 上电后的 uint64 ns。
  // 每个 handle 用一个 anchor 映射到 ROS epoch，LiDAR 和 Livox IMU 共用。
  struct NoSyncConverter {
    bool initialized = false;
    uint64_t raw_anchor = 0;
    uint64_t ros_anchor = 0;

    void Reset() {
      initialized = false;
      raw_anchor = 0;
      ros_anchor = 0;
    }
  };

  struct HandleTimestampState {
    TimestampDomain domain = TimestampDomain::kUnknown;
    NoSyncConverter nosync_converter;

    // raw rollback 需要按数据源判断，避免 LiDAR/IMU 交错包互相误判。
    bool has_last_lidar_raw = false;
    uint64_t last_lidar_raw = 0;
    bool has_last_imu_raw = false;
    uint64_t last_imu_raw = 0;

    // 发布侧单调保护：NoSync re-anchor 要保证整个新时间域都大于这里。
    bool has_last_lidar_published = false;
    uint64_t last_lidar_published = 0;
    uint64_t last_lidar_frame_end = 0;
    bool has_last_imu_published = false;
    uint64_t last_imu_published = 0;
    bool has_last_any_published = false;
    uint64_t last_any_published = 0;
  };

  TimestampResult ConvertPacketTimestamp(uint32_t handle,
                                         uint8_t timestamp_type,
                                         uint64_t raw_stamp,
                                         PacketSource source);
  void ResetPendingPointData(uint32_t handle);
  bool ShouldPublishLidarFrame(uint32_t handle,
                               uint64_t base_time,
                               uint64_t end_time,
                               uint8_t timestamp_type);
  bool ShouldPublishImu(uint32_t handle, uint64_t timestamp, uint8_t timestamp_type);
  void MarkLidarPublished(uint32_t handle, uint64_t base_time, uint64_t end_time);
  void MarkImuPublished(uint32_t handle, uint64_t timestamp);
  static void WarnEvery(const std::string& key, const std::string& msg);

  PointCloudsCallback points_callback_;
  void* pub_client_data_ = nullptr;

  ImuDataCallback imu_callback_;
  void* imu_client_data_ = nullptr;

  PointFrame frame_;

  std::deque<RawPacket> raw_packet_queue_;

  //pub config
  uint64_t publish_interval_ = 100000000; //100 ms
  uint64_t publish_interval_tolerance_ = 100000000; //100 ms
  uint64_t publish_interval_ms_ = 100; //100 ms
  TimePoint last_pub_time_;
  bool legacy_timer_initialized_ = false;
  NoSyncTimeMode nosync_time_mode_ = NoSyncTimeMode::kSteadyRaw;
  NoSyncPublishMode nosync_publish_mode_ = NoSyncPublishMode::kSensorTime;
  uint32_t min_points_per_frame_ = 0;

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_;
  std::map<uint32_t, std::vector<PointXyzlt>> points_;
  std::map<uint32_t, LidarExtParameter> lidar_extrinsics_;
  std::map<uint32_t, HandleTimestampState> timestamp_states_;
  uint16_t lidar_listen_id_ = 0;
};

PubHandler &pub_handler();

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_PUB_HANDLER_H_
