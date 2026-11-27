#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sayd 缓冲验证：推 1s 440Hz 单音，打印 aplay 实际命令与 ALSA 协商出的 buffer。

用法（205 板上）：python3 sayd_buftest.py
判据：buffer_size ≈ 4000 frames（16000/4）= 250ms；period_size ≈ 800 frames = 50ms
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


N = 16000
pcm = bytearray()
for i in range(N):
    pcm += struct.pack("<h", int(12000 * math.sin(2 * math.pi * 440 * i / N)))

s = socket.create_connection(("127.0.0.1", 18081), 5)
s.settimeout(5)
s.sendall(fr(1, b'{"rate":16000,"channels":1}'))
print("START:", s.recv(64).decode().strip(), flush=True)
time.sleep(0.15)

print("CMD:", sh("ps -eo args | grep -m1 '[a]play'"), flush=True)
print("HW :", sh("cat /proc/asound/card0/pcm0p/sub0/hw_params 2>&1").replace("\n", " | "), flush=True)

for off in range(0, len(pcm), 3200):
    s.sendall(fr(2, bytes(pcm[off:off + 3200])))
    time.sleep(0.1)
s.sendall(fr(3, b""))
print("STOP:", s.recv(64).decode().strip(), flush=True)
s.close()
