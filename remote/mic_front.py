#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MIC 三通道前端：选能量最大路（L2）+ speex 降噪（L3），输出单声道 raw 到 stdout。

背景（OPS-91 / 2026-11-26）：
  本体 USB 麦阵列是 6ch/16k，实测有效独立通道只有 ch0/ch1/ch2（ch2==ch3，ch4/ch5 近静音），
  ch1(FL→FR) 比 ch0 高 5.7dB。原对讲下行 arecord -c 1 固定取 ch0，浪费了更强拾音点。
  本脚本采前 3 路，按帧选能量最大路（带滞回防抖），可选 speex 降噪，仍按 16k/mono/S16_LE
  输出，后端与前端零改动。

用法（由后端 _talk_cmd() 经 SSH 调用）：
  stdbuf -o0 python3 mic_front.py [--no-denoise] [--fixed 1]

参数：
  --no-denoise   关闭 speex 降噪（A/B 对比用）
  --fixed N      固定取第 N 路（0/1/2），不选路（A/B 对比用）；默认自动选路
环境变量：
  MIC_PERIOD_MS  帧长 ms，默认 40（须与后端 TALK_PERIOD_US/1000 一致）
  MIC_HYST_DB    选路滞回 dB，默认 3.0（新路需高当前路 3dB 才切，防抖）
  MIC_HOLD       连续多少帧满足才切，默认 3
  MIC_DEV        arecord 设备，默认 plughw:1,0
输出：stdout 二进制 S16_LE 16k 单声道；诊断写 stderr。
"""
import argparse
import array
import ctypes
import math
import os
import select
import subprocess
import sys
import time

RATE = 16000
PERIOD_MS = int(os.environ.get("MIC_PERIOD_MS", "40"))
FRAME_SMP = RATE * PERIOD_MS // 1000          # 640 samples @40ms
NCH = 3                                        # 采前 3 路
HYST_DB = float(os.environ.get("MIC_HYST_DB", "3.0"))
HOLD = int(os.environ.get("MIC_HOLD", "3"))
DEV = os.environ.get("MIC_DEV", "plughw:1,0")

# 写 stderr 的诊断（不污染 stdout 的 PCM）
def diag(msg):
    sys.stderr.write("[mic_front] %s\n" % msg)
    sys.stderr.flush()


# ---------- speex 降噪（ctypes 直调 libspeexdsp，零 pip 依赖）----------
class SpeexNS:
    """speex_preprocess：denoise on，agc off（硬件增益已 50dB 满格）。"""
    def __init__(self, frame_smp=RATE // 50):  # 20ms 帧 = 320 smp
        self.frame = 320
        self.lib = ctypes.CDLL("libspeexdsp.so.1")
        self.lib.speex_preprocess_state_init.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.speex_preprocess_state_init.restype = ctypes.c_void_p
        self.lib.speex_preprocess_ctl.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        self.lib.speex_preprocess_run.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.speex_preprocess_state_destroy.argtypes = [ctypes.c_void_p]
        self.st = self.lib.speex_preprocess_state_init(self.frame, RATE)
        # SpeexPreprocess ctl 常量
        self.SET_DENOISE = 2
        self.SET_AGC = 3
        self.SET_VAD = 4
        self.SET_NOISE_SUPPRESS = 5      # 噪声抑制强度 dB（负值）
        self.SET_DEREVERB = 12
        one = ctypes.c_int(1)
        zero = ctypes.c_int(0)
        supp = ctypes.c_int(18)          # −18 dB：温和，避免人声失真（−25 偏激进）
        self.lib.speex_preprocess_ctl(self.st, self.SET_DENOISE, ctypes.byref(one))
        self.lib.speex_preprocess_ctl(self.st, self.SET_AGC, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_VAD, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_DEREVERB, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_NOISE_SUPPRESS, ctypes.byref(supp))
        diag("speex NS init: frame=%d rate=%d denoise=on supp=-18dB agc=off" % (self.frame, RATE))

    def process(self, samples):
        """samples: array('h') 长度 FRAME_SMP（须为 320 整数倍）；整块 in-place 处理。"""
        out = array.array("h", samples)
        for i in range(0, len(out), self.frame):
            chunk = out[i:i + self.frame]
            if len(chunk) < self.frame:
                continue
            self.lib.speex_preprocess_run(self.st, chunk.buffer_info()[0])
            out[i:i + self.frame] = chunk
        return out

    def close(self):
        try:
            self.lib.speex_preprocess_state_destroy(self.st)
        except Exception:
            pass


def energy(samples):
    """S16 短整型的平方和（避免浮点开销）。"""
    s = 0
    for v in samples:
        s += v * v
    return s


def pick_channel(frame, cur, above, hyst_lin, fixed=None):
    """frame: NCH 路交织 array；返回 (选路, 新above, 该路单声道 array)。"""
    chans = [array.array("h") for _ in range(NCH)]
    for i in range(FRAME_SMP):
        for c in range(NCH):
            chans[c].append(frame[i * NCH + c])
    if fixed is not None:
        return fixed, 0, chans[fixed]
    e = [energy(x) for x in chans]
    best = max(range(NCH), key=lambda c: e[c])
    cur_e = e[cur]
    if best != cur and e[best] > cur_e * hyst_lin:
        above += 1
        if above >= HOLD:
            return best, 0, chans[best]
        return cur, above, chans[cur]
    return cur, 0, chans[cur]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-denoise", action="store_true")
    ap.add_argument("--fixed", type=int, default=None, choices=[0, 1, 2])
    args = ap.parse_args()

    ns = None if args.no_denoise else SpeexNS()
    hyst_lin = 10 ** (HYST_DB / 10.0)
    cmd = ["arecord", "-q", "-D", DEV, "-f", "S16_LE", "-r", str(RATE),
           "-c", str(NCH), "-t", "raw", "-B", "200000", "-F",
           str(PERIOD_MS * 1000), "-"]
    diag("spawn: %s | nch=%d frame=%dsmp hyst=%.1fdB hold=%d fixed=%s denoise=%s"
         % (" ".join(cmd), NCH, FRAME_SMP, HYST_DB, HOLD, args.fixed, not args.no_denoise))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    outstdout = sys.stdout.buffer
    buf = bytearray()
    cur, above = 1, 0        # 默认从 ch1（实测最强）起
    raw_frame_bytes = FRAME_SMP * NCH * 2
    t0 = time.time()
    frames = 0
    switches = 0
    try:
        while True:
            r, _, _ = select.select([p.stdout], [], [], 0.5)
            if not r:
                if p.poll() is not None:
                    break
                continue
            data = os.read(p.stdout.fileno(), 8192)
            if not data:
                break
            buf.extend(data)
            while len(buf) >= raw_frame_bytes:
                raw = bytes(buf[:raw_frame_bytes])
                del buf[:raw_frame_bytes]
                a = array.array("h")
                a.frombytes(raw)
                if sys.byteorder != "little":
                    a.byteswap()
                newcur, above, mono = pick_channel(a, cur, above, hyst_lin, args.fixed)
                if newcur != cur:
                    switches += 1
                    diag("switch ch%d -> ch%d (t=%.1fs)" % (cur, newcur, time.time() - t0))
                cur = newcur
                if ns is not None:
                    mono = ns.process(mono)
                if sys.byteorder != "little":
                    mono.byteswap()
                outstdout.write(mono.tobytes())
                outstdout.flush()
                frames += 1
    except BrokenPipeError:
        diag("downstream closed")
    finally:
        try:
            p.terminate()
        except Exception:
            pass
        diag("exit frames=%d dur=%.1fs switches=%d final_ch=%d" %
             (frames, time.time() - t0, switches, cur))
        if ns is not None:
            ns.close()


if __name__ == "__main__":
    main()
