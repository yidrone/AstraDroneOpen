#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, os, socket, time, multiprocessing as mp
from typing import Optional, Tuple, List

def human_bps(num_bytes: int, seconds: float) -> Tuple[float, float, float]:
    if seconds <= 0: return 0.0, 0.0, 0.0
    bps = (num_bytes * 8) / seconds
    return bps/1e6, bps/1e9, num_bytes/seconds/(1024*1024)

def worker_sender(dst_ip: str, dst_port: int, payload: bytes, start_ts: float,
                  duration: float, source_ip: Optional[str], sndbuf: int,
                  stop_evt: mp.Event, result_q: mp.Queue) -> None:
    total_sent = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if source_ip:
            sock.bind((source_ip, 0))
        if sndbuf:
            try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sndbuf)
            except Exception: pass

        # 同步起跑
        while True:
            now = time.perf_counter()
            if now >= start_ts or stop_evt.is_set():
                break
            time.sleep(0.001)

        end_ts = start_ts + duration
        dest = (dst_ip, dst_port)

        while not stop_evt.is_set():
            if time.perf_counter() >= end_ts:
                break
            try:
                sent = sock.sendto(payload, dest)
                if sent: total_sent += sent
            except Exception:
                # 可能的暂时性错误（缓冲满等），忽略重试
                pass
    finally:
        try: sock.close()
        except Exception: pass
        result_q.put(total_sent)

def run_one(args):
    print(f"[信息] 目标 {args.dst_ip}:{args.port} | 时长 {args.duration:.1f}s | 并发 {args.parallel} | 载荷 {args.payload}B")
    payload = os.urandom(args.payload) if args.random_payload else (b"x" * args.payload)
    start_ts = time.perf_counter() + 1.0  # 1秒后统一起跑
    stop_evt = mp.Event()
    result_q = mp.Queue()

    procs: List[mp.Process] = []
    for _ in range(args.parallel):
        p = mp.Process(target=worker_sender,
                       args=(args.dst_ip, args.port, payload, start_ts, args.duration,
                             args.source_ip, args.sndbuf, stop_evt, result_q),
                       daemon=True)
        p.start()
        procs.append(p)

    try:
        time.sleep(max(0.0, start_ts - time.perf_counter()) + args.duration)
    finally:
        stop_evt.set()
        for p in procs:
            p.join(timeout=2.0)

    total_bytes = 0
    while not result_q.empty():
        total_bytes += result_q.get()

    mbps, gbps, MBps = human_bps(total_bytes, args.duration)
    print(f"[结果] 发送总量: {total_bytes/1e6:.2f} MB | 吞吐(发送侧): {mbps:.2f} Mb/s ({gbps:.3f} Gb/s) ≈ {MBps:.2f} MB/s")

def parse_args():
    ap = argparse.ArgumentParser(description="UDP 上行发送压力测试（尽力洪泛）")
    ap.add_argument("--dst-ip", required=True, help="接收端 PC 的 IP 地址")
    ap.add_argument("--port", type=int, default=5600, help="目标 UDP 端口（默认 5600）")
    ap.add_argument("--duration", type=float, default=15.0, help="测试时长秒（默认 15）")
    ap.add_argument("--parallel", type=int, default=4, help="并发发送进程数（默认 4）")
    ap.add_argument("--payload", type=int, default=1400, help="每包字节数（默认 1400，接近 MTU）")
    ap.add_argument("--random-payload", action="store_true", help="使用随机载荷（默认禁用以省 CPU）")
    ap.add_argument("--source-ip", default=None, help="绑定本机源 IP（多网卡时选定出口）")
    ap.add_argument("--sndbuf", type=int, default=16*1024*1024, help="SO_SNDBUF（默认 16MiB）")
    return ap.parse_args()

if __name__ == "__main__":
    mp.set_start_method("spawn")  # 兼容性更好
    args = parse_args()
    run_one(args)
