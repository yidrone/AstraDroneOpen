#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 (Ubuntu 20) 将 rosbag 中的 sensor_msgs/Image 转为视频 + 等时采样图片

使用方法：
1. 先在下方【配置区】按需修改参数（bag 路径、话题名、是否按张数采样或按时间间隔采样等）。
2. 在已 source /opt/ros/noetic/setup.bash 的环境中直接运行：
   python3 bag_to_video_and_images.py

依赖（Ubuntu 20 / ROS Noetic）：
  sudo apt-get install -y ros-noetic-cv-bridge python3-rosbag python3-rospkg
  python3 -m pip install --upgrade pip opencv-python
"""
import os
import sys
from pathlib import Path
import math

import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2


# =========================
# ======【配置区】=========
# =========================

# 1) 输入 rosbag 及图像话题
BAG_PATH      = "/media/luli/SSD(TiPlus7100)/yolo_data_car.bag"                 # 你的 bag 路径
IMAGE_TOPIC   = "/camera/color/image_raw"           # 图像话题名称

# 2) 导出视频设置
VIDEO_OUT     = "output.mp4"                        # 导出视频文件路径
VIDEO_FOURCC  = "mp4v"                              # 视频编码 fourcc（常见：mp4v / XVID / avc1）
VIDEO_FPS     = 0.0                                  # 视频帧率；<=0 表示自动根据时间戳估计

# 3) 导出图片设置（等时采样）
USE_COUNT     = True                                 # 采样方式二选一：True=按张数；False=按间隔
NUM_IMAGES    = 120                                  # 当 USE_COUNT=True 时，等时采样的图片总张数
INTERVAL_SEC  = 0.5                                  # 当 USE_COUNT=False 时，每隔多少秒导出一张

IMAGES_DIR    = "frames_out"                         # 图片输出文件夹
IMAGE_EXT     = "png"                                # 图片扩展名：png / jpg / jpeg / bmp
JPEG_QUALITY  = 95                                   # 当输出 jpg/jpeg 时的质量 (1-100)

# 4) 可选：只处理中间一段（从开头跳过 / 从结尾提前结束），单位：秒
START_OFFSET  = 0.0                                  # 从 bag 起始时间偏移多少秒开始处理
END_OFFSET    = 0.0                                  # 在 bag 结束前提前多少秒停止处理

# 5) 可选：调整输出分辨率（留空字符串保持原始尺寸；格式 "宽x高" 如 "1280x720"）
RESIZE        = ""


# =========================
# ======【实现区】=========
# =========================

def human_fourcc(s: str) -> int:
    """将 'mp4v'/'XVID'/... 转为 cv2.VideoWriter fourcc。"""
    s = (s or "mp4v").strip()
    if len(s) != 4:
        raise ValueError("fourcc 必须是 4 个字符，例如 mp4v / XVID / avc1")
    return cv2.VideoWriter_fourcc(*s)


def ensure_dir(p: Path):
    """确保目录存在。"""
    p.mkdir(parents=True, exist_ok=True)


def infer_fps_from_timestamps(stamps):
    """
    用中位帧间隔估计 FPS，避免抖动影响。
    如果数据太少或异常，则给保底值。
    """
    if len(stamps) < 3:
        return 30.0
    dts = [stamps[i+1] - stamps[i] for i in range(len(stamps)-1)]
    dts_pos = [dt for dt in dts if dt > 0]
    if not dts_pos:
        return 30.0
    dts_pos.sort()
    median_dt = dts_pos[len(dts_pos)//2]
    if median_dt <= 0:
        return 30.0
    fps = 1.0 / median_dt
    return max(1.0, round(fps, 1))


def build_equal_time_samples_by_count(t0, t1, n):
    """在 [t0, t1] 区间内平均采样 n 个时间点（含首尾）。"""
    if n <= 1:
        return [t0]
    step = (t1 - t0) / (n - 1)
    return [t0 + i * step for i in range(n)]


def build_equal_time_samples_by_interval(t0, t1, interval):
    """从 t0 起每隔 interval 秒取样，直到接近 t1（包含末尾附近最后一个点）。"""
    if interval <= 0:
        raise ValueError("INTERVAL_SEC 必须 > 0")
    times = []
    t = t0
    while t <= t1 + 1e-6:
        times.append(t)
        t += interval
    return times


def parse_resize(resize_str: str):
    """解析 RESIZE 字符串 'WxH' -> (w, h)，空字符串返回 None。"""
    if not resize_str:
        return None
    try:
        w, h = resize_str.lower().split("x")
        return (int(w), int(h))
    except Exception:
        raise ValueError('RESIZE 格式应为 "宽x高"，例如 "1280x720"')


def main():
    bag_path = Path(BAG_PATH)
    if not bag_path.exists():
        print(f"[ERR] bag 文件不存在: {bag_path}", file=sys.stderr)
        sys.exit(1)

    resize_wh = parse_resize(RESIZE)  # None 或 (w,h)
    bridge = CvBridge()

    # ---------- 预扫描：收集时间戳 + 第一帧尺寸 ----------
    print("[INFO] 预扫描 bag，收集时间戳与图像尺寸 ...")
    timestamps = []
    img_size = None  # (w, h)

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_t0 = bag.get_start_time()
        bag_t1 = bag.get_end_time()

        t_begin = bag_t0 + max(0.0, float(START_OFFSET))
        t_end   = bag_t1 - max(0.0, float(END_OFFSET))
        if t_end <= t_begin:
            print("[ERR] 有效时间范围为空，请检查 START_OFFSET / END_OFFSET", file=sys.stderr)
            sys.exit(1)

        for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
            ts = float(t.to_sec())
            if ts < t_begin or ts > t_end:
                continue
            timestamps.append(ts)

            if img_size is None:
                # 抢第一帧，确定尺寸
                try:
                    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except CvBridgeError:
                    cv_img = bridge.imgmsg_to_cv2(msg)
                h0, w0 = cv_img.shape[:2]
                img_size = (w0, h0)

    if not timestamps:
        print("[ERR] 在指定时间范围/话题内未找到图像消息。", file=sys.stderr)
        sys.exit(1)

    # ---------- 确定 FPS ----------
    fps = VIDEO_FPS if VIDEO_FPS and VIDEO_FPS > 0 else infer_fps_from_timestamps(timestamps)
    print(f"[INFO] 视频 FPS = {fps:.2f}（<=0 时自动估计，可在配置区 VIDEO_FPS 指定）")

    # ---------- 构建采样时间点（等时采样） ----------
    t0 = timestamps[0]
    t1 = timestamps[-1]
    if USE_COUNT:
        n = max(1, int(NUM_IMAGES))
        sample_times = build_equal_time_samples_by_count(t0, t1, n)
        print(f"[INFO] 采样方式：按张数，共 {n} 张")
    else:
        interval = float(INTERVAL_SEC)
        sample_times = build_equal_time_samples_by_interval(t0, t1, interval)
        print(f"[INFO] 采样方式：按间隔，每 {interval:.3f} s 一张，共 {len(sample_times)} 张")

    # ---------- 输出准备 ----------
    images_dir = Path(IMAGES_DIR)
    ensure_dir(images_dir)

    if resize_wh is not None:
        vw_size = resize_wh
    else:
        vw_size = img_size  # (w, h)

    fourcc = human_fourcc(VIDEO_FOURCC)
    video_out_path = Path(VIDEO_OUT)
    ensure_dir(video_out_path.parent)

    writer = cv2.VideoWriter(str(video_out_path), fourcc, fps, vw_size)
    if not writer.isOpened():
        print("[ERR] 无法打开 VideoWriter，请检查 fourcc 与输出路径/后缀。", file=sys.stderr)
        sys.exit(1)

    # 图片编码参数
    is_jpeg = IMAGE_EXT.lower() in ("jpg", "jpeg")
    img_params = []
    if is_jpeg:
        q = max(1, min(100, int(JPEG_QUALITY)))
        img_params = [int(cv2.IMWRITE_JPEG_QUALITY), q]

    # ---------- 主循环：写视频 + 遇到采样时间就导出图片 ----------
    print("[INFO] 开始写视频并导出等时采样图片 ...")
    bridge = CvBridge()
    next_idx = 0
    saved = 0
    total = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
            ts = float(t.to_sec())
            # 仅在有效时间窗口内处理
            bag_t0 = bag.get_start_time()
            bag_t1 = bag.get_end_time()
            if ts < (bag_t0 + START_OFFSET) or ts > (bag_t1 - END_OFFSET):
                continue

            try:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError:
                cv_img = bridge.imgmsg_to_cv2(msg)

            # 调整尺寸（如配置）
            if resize_wh is not None:
                frame = cv2.resize(cv_img, resize_wh, interpolation=cv2.INTER_AREA)
            else:
                frame = cv_img

            # 写入视频
            writer.write(frame)
            total += 1

            # 等时采样导图：当前帧时间戳 >= 下一个目标采样时间 -> 保存当前帧
            while next_idx < len(sample_times) and ts + 1e-9 >= sample_times[next_idx]:
                fname = f"frame_{next_idx:06d}_{ts:.6f}.{IMAGE_EXT}"
                out_path = images_dir / fname
                if is_jpeg:
                    cv2.imwrite(str(out_path), frame, img_params)
                else:
                    cv2.imwrite(str(out_path), frame)
                saved += 1
                next_idx += 1

    writer.release()

    print("\n[OK] 处理完成")
    print(f"    视频输出 : {video_out_path}  （FPS={fps:.2f}，尺寸={vw_size[0]}x{vw_size[1]}，总帧数≈{total}）")
    print(f"    图片输出 : {images_dir}  （导出 {saved}/{len(sample_times)} 张）")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[FATAL] 运行失败：{e}", file=sys.stderr)
        sys.exit(1)
