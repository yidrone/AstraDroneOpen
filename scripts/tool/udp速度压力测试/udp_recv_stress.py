#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, socket, time, os
from typing import Optional

def human_bps(nbytes, secs):
    if secs <= 0: return 0.0, 0.0, 0.0
    bps = nbytes * 8 / secs
    return bps / 1e6, bps / 1e9, nbytes / secs / (1024*1024)

def read_rx_bytes(iface: str) -> Optional[int]:
    try:
        with open(f"/sys/class/net/{iface}/statistics/rx_bytes","r") as f:
            return int(f.read().strip())
    except Exception:
        return None

def main():
    ap = argparse.ArgumentParser(description="UDP 下行接收压力测试（统计端到端可达吞吐）")
    ap.add_argument("--bind-ip", default="0.0.0.0", help="绑定本机IP（默认 0.0.0.0）")
    ap.add_argument("--port", type=int, default=5600, help="监听端口（默认 5600）")
    ap.add_argument("--duration", type=float, default=15.0, help="测试时长秒（默认 15）")
    ap.add_argument("--iface", default=None, help="本机网卡名，用于读取 rx_bytes 校验（如 enp3s0）")
    ap.add_argument("--rcvbuf", type=int, default=64*1024*1024, help="SO_RCVBUF（默认 64MiB）")
    args = ap.parse_args()

    print(f"[信息] 监听 {args.bind_ip}:{args.port}，持续 {args.duration:.1f}s")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)
        except Exception:
            pass
        sock.bind((args.bind_ip, args.port))
        sock.settimeout(0.2)

        buf = bytearray(65536)
        mv = memoryview(buf)

        t0 = time.perf_counter()
        t_end = t0 + args.duration
        total = 0
        pkt = 0
        rx0 = read_rx_bytes(args.iface) if args.iface else None

        while True:
            now = time.perf_counter()
            if now >= t_end:
                break
            try:
                n, _ = sock.recvfrom_into(mv)
                if n > 0:
                    total += n
                    pkt += 1
            except socket.timeout:
                pass
            except Exception:
                pass

        rx1 = read_rx_bytes(args.iface) if args.iface else None
    finally:
        sock.close()

    mbps, gbps, MBps = human_bps(total, args.duration)
    print(f"[结果] 接收总量: {total/1e6:.2f} MB | 包数: {pkt} | 吞吐: {mbps:.2f} Mb/s ({gbps:.3f} Gb/s) ≈ {MBps:.2f} MB/s")
    print(f"FINAL_THROUGHPUT_Mbps={mbps:.2f}")  # 便于脚本化解析
    if args.iface and rx0 is not None and rx1 is not None and rx1 >= rx0:
        diff = rx1 - rx0
        m2, g2, M2 = human_bps(diff, args.duration)
        print(f"[网卡校验 {args.iface}] RX差值: {diff/1e6:.2f} MB | 吞吐: {m2:.2f} Mb/s ({g2:.3f} Gb/s) ≈ {M2:.2f} MB/s")
        if abs(m2-mbps)/max(mbps,1e-6) > 0.15:
            print("[提示] 应用层统计与网卡计数差异较大，可能存在其它流量/丢包或内核缓冲影响，可延长时长/调大缓冲后再测。")

if __name__ == "__main__":
    main()
