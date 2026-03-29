# test_move.py
import time
import socket

IP = "192.168.144.108"   # 你的云台 IP
PORT = 9002              # 先用 9002，如无动作再换 5000

def send(cmd):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("SEND:", cmd)
    sock.sendto(cmd.encode("ascii"), (IP, PORT))
    sock.close()

# ========== 向右转 ==========
# PTZ 04 = 向右
send("#TPUG2wPTZ046E")
time.sleep(3)

# ========== 停止 ==========
send("#TPUG2wPTZ006A")
time.sleep(2)

# ========== 向左转 ==========
# PTZ 03 = 向左
send("#TPUG2wPTZ036D")
time.sleep(3)

# ========== 再停止 ==========
send("#TPUG2wPTZ006A")

print("测试完成，请观察云台是否左右转动")

