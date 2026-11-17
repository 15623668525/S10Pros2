#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sayd 250ms 缓冲抗抖测试：推 2s 单音，中途制造 0.5s 空洞（远超 250ms 缓冲）。

判据：aplay 不应退出；补上数据后继续出声；STOP 后释放设备。
用法（205）：python3 sayd_jittertest.py
"""
import math
import socket
import struct
import subprocess
import time


def fr(ftype, payload):
    return struct.pack(">BI", ftype, len(payload)) + payload


def sh(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return (r.stdout + r.stderr).strip()


N = 32000
pcm = bytearray()
for i in range(N):
    pcm += struct.pack("<h", int(12000 * math.sin(2 * math.pi * 440 * i / N)))

s = socket.create_connection(("127.0.0.1", 18081), 5)
s.settimeout(5)
s.sendall(fr(1, b'{"rate":16000,"channels":1}'))
print("START:", s.recv(64).decode().strip(), flush=True)

for idx, off in enumerate(range(0, len(pcm), 3200)):
    if idx == 10:                       # 第 1s 处制造 0.5s 空洞
        print("gap 0.5s ...", flush=True)
        time.sleep(0.5)
        print("APLAY alive:", sh("pgrep -c -f '[a]play'"), flush=True)
    s.sendall(fr(2, bytes(pcm[off:off + 3200])))
    time.sleep(0.1)

s.sendall(fr(4, b""))
print("STATE:", s.recv(64).decode().strip(), flush=True)
s.sendall(fr(3, b""))
print("STOP:", s.recv(64).decode().strip(), flush=True)
s.close()
time.sleep(0.5)
print("APLAY after stop:", sh("pgrep -c -f '[a]play'") or "0", flush=True)
