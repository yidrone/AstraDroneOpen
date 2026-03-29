#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ground_api.py (完整可运行版)
- FastAPI 静态网页：/  /explore  /map
- 状态：优先 MAVLink TCP (tcp:192.168.144.11:5760)
- 一键开始探索：MAVLink 版 /api/mission/start（预热setpoint -> OFFBOARD -> ARM -> 持续setpoint）
- 启动/关闭 8 个命令：subprocess.Popen 按顺序每 3 秒启动，stop 用 killpg 一键关闭
- 探索范围保存/读取：/api/exploration/box  -> 写入 ~/.astra/box.yaml
"""

import os
import json
import time
import math
import signal
import socket
import uuid
import threading
import http.client
import xmlrpc.client
import subprocess
from dataclasses import dataclass
from typing import Dict, Any, Optional, List

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ===================== 配置（按你实际路径） =====================
WWW_DIR = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/Exploration/FUEL/dmz/www"
PCD_DIR = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/SLAM/FAST_LIO/PCD"

# MAVLink TCP（直连数据源）
MAVLINK_TCP = "tcp:192.168.144.11:5760"
ROS_SETUP = "source /opt/ros/noetic/setup.bash"
WS_SETUP = "source /home/yidrone/AstraDrone/AstraDrone_ros1_ws/devel/setup.bash"
ENV_PREFIX = f"{ROS_SETUP} && {WS_SETUP} && "
DEMO_SCRIPT = "/home/yidrone/AstraDrone/scripts/run_sh/demonstration.sh"

# 8 个命令里涉及的 python 脚本
PCD_PUBLISHER = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/Exploration/FUEL/dmz/bin/pcd_publisher.py"
RC_TOGGLE = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/Exploration/FUEL/dmz/bin/rc_toggle_to_reconfigure.py"

# 探索范围保存路径（网页保存/读取用）
BOX_CFG_PATH = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/Exploration/FUEL/dmz/bin/exploration_box.json"
# ROS 使用的 box 参数文件（自动生成）
ROS_BOX_CFG_PATH = "/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/Exploration/FUEL/dmz/bin/box.yaml"

DEFAULT_BOX = {
    "box_min_x": -10.0, "box_min_y": -10.0, "box_min_z": 0.0,
    "box_max_x": 10.0,  "box_max_y": 10.0,  "box_max_z": 2.0,
}

# 日志目录（用于 diagnose）
LOG_DIR = "/home/yidrone/.astra/exploration_logs"

# ===================== FastAPI =====================
app = FastAPI()
app.mount("/vendor", StaticFiles(directory=os.path.join(WWW_DIR, "vendor")), name="vendor")

# ===================== 页面路由（三页面） =====================
@app.get("/")
def page_status():
    return FileResponse(os.path.join(WWW_DIR, "status.html"))

@app.get("/explore")
def page_explore():
    return FileResponse(os.path.join(WWW_DIR, "explore.html"))

@app.get("/map")
def page_map():
    return FileResponse(os.path.join(WWW_DIR, "map_viewer.html"))

# ===================== 工具函数 =====================
def sh(cmd_list: List[str]):
    p = subprocess.run(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.returncode, p.stdout, p.stderr

def bash(cmd: str):
    return sh(["bash", "-lc", cmd])

def _tail_file(path: str, max_lines: int = 60) -> str:
    if not path or not os.path.exists(path):
        return ""
    try:
        read_bytes = 300_000
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(size - read_bytes, 0), os.SEEK_SET)
            data = f.read()

        text = data.decode("utf-8", errors="replace")
        return "\n".join(text.splitlines()[-max_lines:])
    except Exception as e:
        return f"[tail error] {repr(e)}"

# ===================== 探索范围保存/读取（给网页） =====================
_box_lock = threading.Lock()

def _save_box(d: dict):
    # 使用 with 语句获取锁，保护整个文件写入过程
    with _box_lock:
        os.makedirs(os.path.dirname(BOX_CFG_PATH), exist_ok=True)
        tmp = BOX_CFG_PATH + ".tmp"

        with open(tmp, "w") as f:
            json.dump(d, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, BOX_CFG_PATH)
        _write_ros_box_yaml(d)

def _write_ros_box_yaml(d: dict):
    """
    根据网页 box（JSON）生成 ROS 可直接 rosparam load 的 yaml
    使用绝对参数路径，避免命名空间问题
    """
    os.makedirs(os.path.dirname(ROS_BOX_CFG_PATH), exist_ok=True)

    lines = [
        f"/exploration_node/sdf_map/box_min_x: {d['box_min_x']}",
        f"/exploration_node/sdf_map/box_min_y: {d['box_min_y']}",
        f"/exploration_node/sdf_map/box_min_z: {d['box_min_z']}",
        f"/exploration_node/sdf_map/box_max_x: {d['box_max_x']}",
        f"/exploration_node/sdf_map/box_max_y: {d['box_max_y']}",
        f"/exploration_node/sdf_map/box_max_z: {d['box_max_z']}",
    ]

    tmp = ROS_BOX_CFG_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(lines) + "\n")
        f.flush()
        os.fsync(f.fileno())

    os.replace(tmp, ROS_BOX_CFG_PATH)

def _load_box() -> dict:
    os.makedirs(os.path.dirname(BOX_CFG_PATH), exist_ok=True)
    if not os.path.exists(BOX_CFG_PATH):
        _save_box(dict(DEFAULT_BOX))
        return dict(DEFAULT_BOX)
    try:
        if os.path.getsize(BOX_CFG_PATH) == 0:
            _save_box(dict(DEFAULT_BOX))
            return dict(DEFAULT_BOX)
        with open(BOX_CFG_PATH, "r") as f:
            d = json.load(f)
        for k, v in DEFAULT_BOX.items():
            d.setdefault(k, v)
        return d
    except Exception:
        _save_box(dict(DEFAULT_BOX))
        return dict(DEFAULT_BOX)

class BoxReq(BaseModel):
    box_min_x: float
    box_min_y: float
    box_min_z: float
    box_max_x: float
    box_max_y: float
    box_max_z: float

@app.get("/api/exploration/box")
def get_box():
    return _load_box()

@app.post("/api/exploration/box")
def set_box(req: BoxReq):
    d = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    
    if not (d["box_min_x"] < d["box_max_x"] and d["box_min_y"] < d["box_max_y"] and d["box_min_z"] < d["box_max_z"]):
        raise HTTPException(400, "min must be < max")
    _save_box(d)
    return {"ok": True, **d}

# ===================== 进程管理：按顺序每 3 秒启动 8 个命令 =====================
@dataclass
class ProcInfo:
    name: str
    cmd: str
    popen: subprocess.Popen
    log_path: str
    start_ts: float

_proc_lock = threading.Lock()
_procs: List[ProcInfo] = []

def exploration_running() -> bool:
    with _proc_lock:
        for p in _procs:
            if p.popen.poll() is None:
                return True
        return False

def _kill_process_group(p: subprocess.Popen, timeout_sec: float = 3.0):
    """SIGINT -> SIGTERM -> SIGKILL"""
    if p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        if p.poll() is not None:
            return
        time.sleep(0.1)

    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < timeout_sec:
        if p.poll() is not None:
            return
        time.sleep(0.1)

    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass

def stop_all_processes():
    with _proc_lock:
        procs = list(_procs)
        _procs.clear()
    for info in reversed(procs):
        try:
            _kill_process_group(info.popen)
        except Exception:
            pass

def start_demo_script() -> Dict[str, Any]:
    """
    systemd(systemctl) 运行时没有 GUI 环境变量，需手动注入 DISPLAY / XAUTHORITY / DBUS 等，
    才能在 GNOME 桌面弹出 gnome-terminal 跑 demonstration.sh

    目标行为：
    - 启动时：正常运行一直显示；如果脚本异常退出，窗口停留便于看报错
    - 停止时：能通过 stop_all_processes() 杀掉 gnome-terminal，从而自动关闭窗口
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, "demonstration.log")

    run_cmd = f"{ENV_PREFIX}{DEMO_SCRIPT}"

    # ✅ 只在失败(rc!=0)时停留窗口；成功则直接退出，终端自动关闭
    # ✅ stop 时会 kill gnome-terminal（因为用了 --wait，p.pid 一直有效），窗口必关
    bash_cmd = (
        f"{run_cmd}; rc=$?; "
        "echo; "
        "if [ $rc -ne 0 ]; then "
        "  echo \"[demo failed] exit_code=$rc\"; "
        "  echo \"Press Enter to close...\"; "
        "  read -r _; "
        "fi; "
        "exit $rc"
    )

    uid = 1000
    env = os.environ.copy()
    env["DISPLAY"] = ":0"
    env["XAUTHORITY"] = "/home/yidrone/.Xauthority"
    env["XDG_RUNTIME_DIR"] = f"/run/user/{uid}"
    env["DBUS_SESSION_BUS_ADDRESS"] = f"unix:path=/run/user/{uid}/bus"

    try:
        p = subprocess.Popen(
            # ⭐ 关键：--wait 让 gnome-terminal 进程不立即退出
            ["gnome-terminal", "--wait", "--title=demonstration", "--", "bash", "-lc", bash_cmd],
            preexec_fn=os.setsid,
            close_fds=True,
            env=env,
        )
        with _proc_lock:
            _procs[:] = [
                ProcInfo(
                    name="demonstration(gnome-terminal)",
                    cmd=DEMO_SCRIPT,
                    popen=p,
                    log_path=log_path,
                    start_ts=time.time(),
                )
            ]
        return {"ok": True, "pid": p.pid, "terminal": "gnome-terminal", "log": log_path}
    except Exception as e:
        return {"ok": False, "error": repr(e), "log": log_path}

# ===================== MAVLink 状态线程（后台读取，供 /api/status） =====================
_mav_ok = False
_mav_error: Optional[str] = None
_mav_last_ts = 0.0

_mav_state: Dict[str, Any] = {
    "connected": False,
    "ts": 0.0,
    "mode": None,
    "armed": None,
    "battery_voltage": None,   # V
    "battery_current": None,   # A
    "battery_remaining": None, # %
    "gps_fix_type": None,
    "gps_satellites": None,
    "gps_lat": None,
    "gps_lon": None,
    "gps_alt_m": None,
    "roll_deg": None,
    "pitch_deg": None,
    "yaw_deg": None,
    "rel_alt_m": None,
    "vx": None, "vy": None, "vz": None,
    "status_text": None,
}
_mav_lock = threading.Lock()

def _mav_set(**kwargs):
    global _mav_last_ts
    with _mav_lock:
        for k, v in kwargs.items():
            _mav_state[k] = v
        _mav_state["ts"] = time.time()
        _mav_last_ts = _mav_state["ts"]

def _mav_get_snapshot():
    with _mav_lock:
        return dict(_mav_state)

def _start_mavlink_thread():
    global _mav_ok, _mav_error
    try:
        from pymavlink import mavutil
    except Exception as e:
        _mav_ok = False
        _mav_error = f"pymavlink import failed: {repr(e)}"
        _mav_set(connected=False)
        return

    backoff = 1.0
    while True:
        try:
            _mav_error = None
            _mav_ok = False
            _mav_set(connected=False)

            mav = mavutil.mavlink_connection(MAVLINK_TCP, autoreconnect=True, source_system=255)
            mav.wait_heartbeat(timeout=6)

            _mav_ok = True
            _mav_set(connected=True)
            backoff = 1.0

            while True:
                msg = mav.recv_match(blocking=True, timeout=1)
                if msg is None:
                    if time.time() - _mav_last_ts > 5.0:
                        _mav_set(connected=False)
                    time.sleep(0.1)
                    continue

                mtype = msg.get_type()
                _mav_set(connected=True)

                if mtype == "HEARTBEAT":
                    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    try:
                        mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        mode = None
                    _mav_set(armed=armed, mode=mode)

                elif mtype == "SYS_STATUS":
                    voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != -1 else None
                    current = msg.current_battery / 100.0 if msg.current_battery != -1 else None
                    remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
                    _mav_set(battery_voltage=voltage, battery_current=current, battery_remaining=remaining)

                elif mtype == "GPS_RAW_INT":
                    lat = msg.lat / 1e7 if msg.lat != 0 else None
                    lon = msg.lon / 1e7 if msg.lon != 0 else None
                    alt_m = msg.alt / 1000.0 if msg.alt != 0 else None
                    _mav_set(
                        gps_fix_type=int(msg.fix_type),
                        gps_satellites=int(msg.satellites_visible),
                        gps_lat=lat, gps_lon=lon, gps_alt_m=alt_m
                    )

                elif mtype == "GLOBAL_POSITION_INT":
                    _mav_set(
                        rel_alt_m=msg.relative_alt / 1000.0,
                        vx=msg.vx / 100.0, vy=msg.vy / 100.0, vz=msg.vz / 100.0
                    )

                elif mtype == "ATTITUDE":
                    roll = msg.roll * 180.0 / math.pi
                    pitch = msg.pitch * 180.0 / math.pi
                    yaw = msg.yaw * 180.0 / math.pi
                    _mav_set(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw)

                elif mtype == "STATUSTEXT":
                    _mav_set(status_text=str(msg.text))

        except Exception as e:
            _mav_ok = False
            _mav_error = f"mavlink loop error: {repr(e)}"
            _mav_set(connected=False)
            time.sleep(backoff)
            backoff = min(backoff * 1.5, 10.0)

def ensure_mavlink_thread():
    if not hasattr(ensure_mavlink_thread, "_started"):
        ensure_mavlink_thread._started = True
        th = threading.Thread(target=_start_mavlink_thread, daemon=True)
        th.start()

# ===================== ROS 节点状态判定（全节点 + OFFBOARD） =====================

# 你提供的“探索脚本启动后会启动的全部节点”
ALL_EXPLORATION_NODES = {
    "/csi_camera_node",
    "/livox_lidar_publisher2",
    "/exploration_node",
    "/freedom",
    "/laserMapping_color",
    "/mavros",
    "/rosout",
    "/rosbridge_websocket",
    "/rqt_reconfigure",
    "/tf123",
    "/tf_base2camera",
    "/tf_to_odometry_node",
    "/tf_world_map",
    "/traj_server",
    "/waypoint_generator",
}

# rosnode list 缓存（避免 status 被频繁调用时每次都起 bash）
_ROSNODE_CACHE = {"ts": 0.0, "nodes": [], "err": None}
_ROSNODE_CACHE_TTL_SEC = 1.0

# ===================== ROS 节点状态判定 =====================

# 1. 在函数外面定义这个安全的传输层
class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout=0.5, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.timeout = timeout

    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self.timeout)


def _get_ros_nodes_cached() -> list:
    now = time.time()
    if (now - float(_ROSNODE_CACHE["ts"])) < _ROSNODE_CACHE_TTL_SEC:
        return list(_ROSNODE_CACHE["nodes"])

    master_uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    
    try:
        master = xmlrpc.client.ServerProxy(
            master_uri, 
            transport=TimeoutTransport(timeout=0.5)
        )
        
        code, msg, state = master.getSystemState("/fastapi_monitor")
        
        if code == 1:
            pubs, subs, srvs = state
            nodes = set()
            for _, publishers_list in pubs: nodes.update(publishers_list)
            for _, subscribers_list in subs: nodes.update(subscribers_list)
            for _, providers_list in srvs: nodes.update(providers_list)
                
            _ROSNODE_CACHE["nodes"] = list(nodes)
            _ROSNODE_CACHE["err"] = None
        else:
            _ROSNODE_CACHE["nodes"] = []
            _ROSNODE_CACHE["err"] = f"ROS Master error: {msg}"
            
    except Exception as e:
        _ROSNODE_CACHE["nodes"] = []
        _ROSNODE_CACHE["err"] = f"XML-RPC failed: {repr(e)}"

    _ROSNODE_CACHE["ts"] = now
    return list(_ROSNODE_CACHE["nodes"])

def _is_offboard(mode_val: Any) -> bool:
    """
    mode 可能是 'OFFBOARD' / 'OFFBOARD ' / 'Offboard' / None
    """
    if not mode_val:
        return False
    try:
        return str(mode_val).strip().upper() == "OFFBOARD"
    except Exception:
        return False


def _calc_exploration_state(mav: Dict[str, Any], mav_recent: bool) -> Dict[str, Any]:
    """
    四状态：
      - 未启动：ALL_EXPLORATION_NODES 一个都没出现
      - 启动中：出现了部分，但没齐
      - 已启动：全部节点齐，但未满足探索条件（OFFBOARD+ARM+mav_recent）
      - 探索中：全部节点齐 + mav_recent + armed + OFFBOARD
    """
    nodes = set(_get_ros_nodes_cached())

    present = sorted([n for n in ALL_EXPLORATION_NODES if n in nodes])
    missing = sorted([n for n in ALL_EXPLORATION_NODES if n not in nodes])

    if len(present) == 0:
        state = "未启动"
    elif len(missing) > 0:
        state = "启动中"
    else:
        armed_ok = bool(mav.get("armed"))
        offboard_ok = _is_offboard(mav.get("mode"))
        if mav_recent and armed_ok and offboard_ok:
            state = "探索中"
        else:
            state = "已启动"

    return {
        "exploration_state": state,
        "required_nodes": sorted(list(ALL_EXPLORATION_NODES)),
        "present_nodes": present,
        "missing_nodes": missing,
        "rosnode_err": _ROSNODE_CACHE.get("err"),

        # 额外给前端/排查用：为什么没到“探索中”
        "mav_recent": mav_recent,
        "armed_ok": bool(mav.get("armed")),
        "offboard_ok": _is_offboard(mav.get("mode")),
        "mode_raw": mav.get("mode"),
    }

# ===================== 基础API =====================
@app.get("/api/health")
def health():
    return {"ok": True}

# ===================== 状态API（只用 MAVLink + 本地进程是否运行） =====================
@app.get("/api/status")
def api_status():
    ensure_mavlink_thread()
    mav = _mav_get_snapshot()

    mav_recent = bool(mav.get("connected")) and (time.time() - float(mav.get("ts") or 0.0) < 3.0)
    source_used = "mavlink" if mav_recent else "none"

    # ===== 4 状态判定：未启动 / 启动中 / 已启动 / 探索中（全节点 + OFFBOARD）=====
    expl = _calc_exploration_state(mav, mav_recent)
    expl_state = expl["exploration_state"]

    # “探索节点是否在跑”：至少不是“未启动”就算跑起来了
    exploration_nodes_running = expl_state != "未启动"

    gps = {
        "fix_type": mav.get("gps_fix_type"),
        "satellites": mav.get("gps_satellites"),
        "lat": mav.get("gps_lat"),
        "lon": mav.get("gps_lon"),
        "alt_m": mav.get("gps_alt_m"),
    }

    return {
        "source_used": source_used,
        "mavlink_ok": _mav_ok,
        "mavlink_error": _mav_error,
        "mavlink_connected": bool(mav.get("connected")),
        "mavlink_ts": mav.get("ts"),

        "armed": mav.get("armed"),
        "mode": mav.get("mode"),
        "battery_percent": mav.get("battery_remaining"),
        "battery_voltage": mav.get("battery_voltage"),
        "battery_current": mav.get("battery_current"),

        "attitude_rpy_deg": [mav.get("roll_deg"), mav.get("pitch_deg"), mav.get("yaw_deg")],
        "gps": gps,
        "rel_alt_m": mav.get("rel_alt_m"),
        "vel_mps": [mav.get("vx"), mav.get("vy"), mav.get("vz")],
        "status_text": mav.get("status_text"),

        "exploration_nodes_running": exploration_nodes_running,
        "exploration_state": expl_state,

        # 前端可展示：还缺哪些节点 / 是否满足 offboard+arm
        "exploration_required_nodes": expl["required_nodes"],
        "exploration_present_nodes": expl["present_nodes"],
        "exploration_missing_nodes": expl["missing_nodes"],
        "rosnode_err": expl["rosnode_err"],

        "exploration_mav_recent": expl["mav_recent"],
        "exploration_armed_ok": expl["armed_ok"],
        "exploration_offboard_ok": expl["offboard_ok"],
        "exploration_mode_raw": expl["mode_raw"],

        "pcd_dir": PCD_DIR,
    }

@app.get("/api/status_mavlink")
def api_status_mavlink():
    ensure_mavlink_thread()
    snap = _mav_get_snapshot()
    return {"mavlink_ok": _mav_ok, "mavlink_error": _mav_error, **snap}

# ===================== 启停探索（8个命令，间隔3秒） =====================
@app.post("/api/exploration/start_nodes")
def start_nodes():
    stop_all_processes()

    info = start_demo_script()

    return {
        "ok": True,
        **info,
        "hint": f"排查：GET /api/exploration/diagnose（日志目录 {LOG_DIR}）",
    }

@app.post("/api/exploration/stop_nodes")
def stop_nodes():
    # 1) 先停 tmux（杀掉 roscore/roslaunch 等）
    subprocess.run(
        ["bash", "-lc", "tmux kill-session -t demonstration || true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # 2) 再杀我们记录的进程（现在 gnome-terminal --wait 会在这里被杀掉，窗口会关闭）
    stop_all_processes()

    return {"ok": True}

@app.get("/api/exploration/running")
def api_running():
    with _proc_lock:
        lst = []
        for p in _procs:
            rc = p.popen.poll()
            lst.append({
                "name": p.name,
                "pid": p.popen.pid,
                "running": (rc is None),
                "returncode": rc,
                "cmd": p.cmd,
                "log": p.log_path,
                "start_ts": p.start_ts,
            })
    return {"running": exploration_running(), "procs": lst}

@app.get("/api/exploration/diagnose")
def api_diagnose():
    with _proc_lock:
        items = []
        for p in _procs:
            rc = p.popen.poll()
            items.append({
                "name": p.name,
                "pid": p.popen.pid,
                "running": (rc is None),
                "returncode": rc,
                "cmd": p.cmd,
                "log": p.log_path,
                "tail": _tail_file(p.log_path, max_lines=80),
            })
    return {"running": exploration_running(), "log_dir": LOG_DIR, "items": items}

class StartMissionReq(BaseModel):
    mode: str = "OFFBOARD"        # ✅ 加回 mode，供 /api/mission/start 使用
    arm: bool = True
    target_alt_m: Optional[float] = None  # 保留你原来的字段

@app.post("/api/mission/start")
def mission_start(req: StartMissionReq):
    """
    MAVROS 版一键开始探索：
    - 仅通过 MAVROS services 切模式 + ARM
    依赖：
      /mavros/set_mode   (mavros_msgs/SetMode)
      /mavros/cmd/arming (mavros_msgs/CommandBool)
    """
    logs: List[str] = []

    # 1) 切模式（默认 OFFBOARD）
    # 用 $'...' 传多行 YAML，避免引号地狱
    mode_str = (req.mode or "OFFBOARD").strip()
    cmd_set_mode = (
        f"{ROS_SETUP} && "
        f"rosservice call /mavros/set_mode "
        f"$'base_mode: 0\\ncustom_mode: \"{mode_str}\"\\n'"
    )
    code_m, out_m, err_m = bash(cmd_set_mode)
    logs.append(f"SetMode cmd: {cmd_set_mode}")
    logs.append(f"SetMode rc={code_m}")
    if out_m:
        logs.append(f"SetMode out: {out_m.strip()}")
    if err_m:
        logs.append(f"SetMode err: {err_m.strip()}")

    if code_m != 0:
        raise HTTPException(500, f"mavros set_mode failed: {err_m or out_m or 'unknown error'}")

    # 2) ARM（可选，req.arm=False 时则跳过）
    if bool(req.arm):
        cmd_arm = (
            f"{ROS_SETUP} && "
            f"rosservice call /mavros/cmd/arming "
            f"$'value: true\\n'"
        )
        code_a, out_a, err_a = bash(cmd_arm)
        logs.append(f"Arming cmd: {cmd_arm}")
        logs.append(f"Arming rc={code_a}")
        if out_a:
            logs.append(f"Arming out: {out_a.strip()}")
        if err_a:
            logs.append(f"Arming err: {err_a.strip()}")

        if code_a != 0:
            raise HTTPException(500, f"mavros arming failed: {err_a or out_a or 'unknown error'}")

    return {
        "ok": True,
        "mode_requested": mode_str,
        "armed_requested": bool(req.arm),
        "logs": logs[-20:],
        "hint": "MAVROS版：仅 set_mode + arming；未发送任何 setpoint。若 OFFBOARD 失败，请确认飞控/参数允许无需外部 setpoint 进入 OFFBOARD（通常 PX4 仍需要持续 setpoint）。",
    }

# ===================== maps：列表/下载/加载/预览（保留你之前的接口） =====================
@app.get("/api/maps")
def list_maps():
    if not os.path.isdir(PCD_DIR):
        return {"maps": []}
    maps = [f for f in os.listdir(PCD_DIR) if f.lower().endswith(".pcd")]
    maps.sort()
    return {"maps": maps}

@app.get("/api/maps/download/{filename}")
def download_map(filename: str):
    path = os.path.join(PCD_DIR, filename)
    if not os.path.exists(path):
        raise HTTPException(404, "pcd not found")
    return FileResponse(path, filename=filename)

class LoadMapReq(BaseModel):
    filename: str
    voxel: float = 0.15

@app.post("/api/maps/load")
def load_map(req: LoadMapReq):
    path = os.path.join(PCD_DIR, req.filename)
    if not os.path.exists(path):
        raise HTTPException(404, "pcd not found")
    cmd = (
        f"{ROS_SETUP} && "
        f"rosparam set /pcd_publisher/pcd_path '{path}' && "
        f"rosparam set /pcd_publisher/voxel {req.voxel} && "
        "rosservice call /pcd_publisher/load '{}'"
    )
    code, out, err = bash(cmd)
    return {"ok": code == 0, "out": out, "err": err}

# ===================== maps：LAS 导出（PCD -> LAS，供 App 下载） =====================
LAS_CACHE_DIR = os.path.join(PCD_DIR, ".cache_las")
LAS_SCALE = 0.01  # 1mm 精度（工程坐标足够用）

def _safe_basename(name: str) -> str:
    # 防止 ../../ 路径穿越
    base = os.path.basename(name)
    if base != name:
        raise HTTPException(400, "invalid filename")
    return base

def _pcd_to_las_cached(pcd_path: str, las_path: str, force: bool = False) -> None:
    """
    使用 PDAL 将 PCD 转为 LAS，并做简单缓存：源文件更新才重新生成。

    ⭐ 只修改这一处：
    1) Open3D 读取 PCD（正确解析 packed rgb）写临时 PLY
    2) PDAL 从 PLY 写 LAS，优先用 filters.assign 的 value 语法把 8-bit RGB 放大到 16-bit
       若 PDAL 解析失败则自动降级不放大（仍然是彩色，不会全黑）
    """
    os.makedirs(os.path.dirname(las_path), exist_ok=True)

    if (not force) and os.path.exists(las_path):
        try:
            if os.path.getmtime(las_path) >= os.path.getmtime(pcd_path):
                return
        except Exception:
            pass

    # ---- 1) PCD -> 临时 PLY（Open3D 负责正确解析 packed rgb）----
    unique_id = uuid.uuid4().hex[:8]
    tmp_ply = os.path.splitext(las_path)[0] + f".__tmp_color_{unique_id}.ply"
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(pcd_path)
        if not pcd.has_colors():
            raise RuntimeError("Open3D did not detect colors in PCD (pcd.has_colors()==False)")
        o3d.io.write_point_cloud(tmp_ply, pcd, write_ascii=False)
    except Exception as e:
        raise HTTPException(500, f"open3d pcd->ply failed: {repr(e)}")

    import subprocess

    def _run_pdal_pipeline(pipeline_obj: dict) -> None:
        try:
            subprocess.run(
                ["pdal", "pipeline", "--stdin"],
                input=json.dumps(pipeline_obj),
                text=True,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except FileNotFoundError:
            raise HTTPException(500, "pdal not installed or not in PATH")
        except subprocess.CalledProcessError as e:
            msg = (e.stderr or e.stdout or "").strip()
            raise RuntimeError(msg or "pdal pipeline failed")

    try:
        # ---- 2A) 优先：用 filters.assign 的 value 语法放大到 16-bit（正确写法是 value，不是 assignment）----
        pipeline_scale = {
            "pipeline": [
                {"type": "readers.ply", "filename": tmp_ply},
                {
                    "type": "filters.assign",
                    "value": [
                        "Red = Red * 256",
                        "Green = Green * 256",
                        "Blue = Blue * 256",
                    ],
                },
                {
                    "type": "writers.las",
                    "filename": las_path,
                    "scale_x": LAS_SCALE,
                    "scale_y": LAS_SCALE,
                    "scale_z": LAS_SCALE,
                },
            ]
        }
        _run_pdal_pipeline(pipeline_scale)

    except RuntimeError as e:
        # ---- 2B) 兜底：如果 assign 解析失败，直接写 LAS（仍然是彩色，只是 0~255 不放大）----
        pipeline_noscale = {
            "pipeline": [
                {"type": "readers.ply", "filename": tmp_ply},
                {
                    "type": "writers.las",
                    "filename": las_path,
                    "scale_x": LAS_SCALE,
                    "scale_y": LAS_SCALE,
                    "scale_z": LAS_SCALE,
                },
            ]
        }
        try:
            _run_pdal_pipeline(pipeline_noscale)
        except RuntimeError as e2:
            raise HTTPException(500, f"pdal pipeline failed: {str(e2)}")

    finally:
        try:
            os.remove(tmp_ply)
        except Exception:
            pass

@app.get("/api/maps/download_las/{filename}")
def download_map_las(filename: str, force: bool = True):
    """
    前端：先 GET /api/maps 拿到 .pcd 列表；
         用户选择某个 pcd 后，调用本接口下载同名 .las。

    - filename：pcd 文件名（必须以 .pcd 结尾，且只能是文件名不允许带路径）
    - force：true 时强制重新转换（默认 false，会做 mtime 缓存）
    """
    fname = _safe_basename(filename)

    if not fname.lower().endswith(".pcd"):
        raise HTTPException(400, "filename must end with .pcd")

    pcd_path = os.path.join(PCD_DIR, fname)
    if not os.path.exists(pcd_path):
        raise HTTPException(404, "pcd not found")

    base = os.path.splitext(fname)[0]
    las_name = base + ".las"
    las_path = os.path.join(LAS_CACHE_DIR, las_name)

    _pcd_to_las_cached(pcd_path, las_path, force=bool(force))

    return FileResponse(las_path, filename=las_name)

@app.get("/api/maps/preview/{filename}")
def preview_map_ply(filename: str):
    src = os.path.join(PCD_DIR, filename)
    if not os.path.exists(src):
        raise HTTPException(404, "pcd not found")

    cache_dir = os.path.join(PCD_DIR, ".cache_preview")
    os.makedirs(cache_dir, exist_ok=True)

    base = os.path.splitext(os.path.basename(filename))[0]
    dst = os.path.join(cache_dir, base + ".ply")

    need = True
    if os.path.exists(dst):
        need = os.path.getmtime(dst) < os.path.getmtime(src)

    if need:
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(src)
            pcd = pcd.voxel_down_sample(0.01)
            o3d.io.write_point_cloud(dst, pcd, write_ascii=False)
        except Exception as e:
            raise HTTPException(500, f"convert to ply failed: {repr(e)}")

    return FileResponse(dst, filename=os.path.basename(dst))

# ===================== 直接运行（可选） =====================
if __name__ == "__main__":
    # 你也可以用 uvicorn 启动：
    #   uvicorn ground_api:app --host 0.0.0.0 --port 8080
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080, access_log=False)

