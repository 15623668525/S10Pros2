#!/usr/bin/env python3
# SNTP 客户端：测本机与 NTP 服务器的时钟偏差。无需 root、无需安装任何包。
# 用法: python3 ntp_offset_check.py [server]     # 默认 <SERVER_IP>
import socket, struct, sys, time

server = sys.argv[1] if len(sys.argv) > 1 else '<SERVER_IP>'
NTP_DELTA = 2208988800  # 1900->1970

def query(host, timeout=3):
    pkt = b'\x1b' + 47 * b'\0'
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    t1 = time.time()
    s.sendto(pkt, (host, 123))
    data, _ = s.recvfrom(1024)
    t4 = time.time()
    s.close()
    words = struct.unpack('!12I', data)
    # 标准 NTPv4 布局：receive = words[8]/[9]，transmit = words[10]/[11]
    t2 = words[8] + words[9] / 2**32 - NTP_DELTA
    t3 = words[10] + words[11] / 2**32 - NTP_DELTA
    # NTP offset 公式
    offset = ((t2 - t1) + (t3 - t4)) / 2
    delay = (t4 - t1) - (t3 - t2)
    return offset, delay

try:
    off, dly = query(server)
    print(f'server={server}  offset={off:+.3f}s  delay={dly*1000:.1f}ms')
    if abs(off) > 1:
        print(f'>>> 本机时钟{"慢" if off > 0 else "快"} {abs(off):.1f} 秒（offset>0 = 服务器时间超前 = 本机慢），需要同步')
    else:
        print('>>> 时钟偏差在 1 秒内，正常')
except Exception as e:
    print(f'FAIL server={server}: {type(e).__name__}: {e}')
    print('>>> 查不到 NTP（网络不通或 123/udp 被拦）')
