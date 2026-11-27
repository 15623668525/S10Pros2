#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MIC 三通道前端：选能量最大路（L2）+ 陷波高通（L3a）+ speex 降噪（L3b，默认关），
输出单声道 raw 到 stdout。

背景（OPS-91 / 2026-11-26）：
  本体 USB 麦阵列是 6ch/16k，实测有效独立通道只有 ch0/ch1/ch2（ch2==ch3，ch4/ch5 近静音），
  ch1(FL→FR) 比 ch0 高 5.7dB。原对讲下行 arecord -c 1 固定取 ch0，浪费了更强拾音点。
  本脚本采前 3 路，按帧选能量最大路（带滞回防抖），再经「二阶高通 + 224/244Hz 双陷波」
  滤掉电机线谱/结构低频（语音带 500-3kHz 零损），可选 speex 降噪，仍按 16k/mono/S16_LE
  输出，后端与前端零改动。

设计依据（录音样本无效性诊断_20261126.md）：
  电机噪声是 224/244Hz 基频 + 离散谐波线，基频邻带能量是语音带的 15-29 倍；
  speex/谱减/Wiener/MMSE 在低输入 SNR 下均劣化语音，唯一无损处理是陷波(基频)+高通。
  L3a（本版新增）即该无损链路；L3b（speex）经真人 A/B 判定削语音，维持默认关。

用法（由后端 _talk_cmd() 经 SSH 调用）：
  stdbuf -o0 python3 mic_front.py [--no-denoise] [--no-filter] [--fixed 1]

参数：
  --no-denoise   关闭 speex 降噪（默认即关，A/B 对比用）
  --no-filter    关闭高通+陷波（A/B 对照组 / 一键回退）
  --fixed N      固定取第 N 路（0/1/2），不选路（A/B 对比用）；默认自动选路
环境变量：
  MIC_PERIOD_MS  帧长 ms，默认 40（须与后端 TALK_PERIOD_US/1000 一致）
  MIC_HYST_DB    选路滞回 dB，默认 3.0（新路需高当前路 3dB 才切，防抖）
  MIC_HOLD       连续多少帧满足才切，默认 3
  MIC_DEV        arecord 设备，默认 plughw:1,0
  MIC_FILTER     设 0 关闭滤波（等价 --no-filter），默认 1 开
  MIC_HP_HZ      高通截止 Hz，默认 120（0=关闭高通）
  MIC_NOTCH_HZ   陷波频率列表（逗号分隔），默认 224,244（空=关闭陷波）
  MIC_NOTCH_Q    陷波 Q 值，默认 25
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

# 注意：故意不 import numpy/scipy —— 业务板上二者冷启动 import 约 2.84s，
# 会把首帧推过后端 talk_start() 的 3s 等待窗口。滤波器系数已离线算好固化在
# 下方 BIQUAD_COEFFS，运行时纯 Python DF-II 递推，零第三方依赖、启动 <0.1s。

RATE = 16000
PERIOD_MS = int(os.environ.get("MIC_PERIOD_MS", "40"))
FRAME_SMP = RATE * PERIOD_MS // 1000          # 640 samples @40ms
NCH = 3                                        # 采前 3 路
HYST_DB = float(os.environ.get("MIC_HYST_DB", "3.0"))
HOLD = int(os.environ.get("MIC_HOLD", "3"))
DEV = os.environ.get("MIC_DEV", "plughw:1,0")

# 滤波链配置：默认开（无损），--no-filter / MIC_FILTER=0 一键回退
FILTER_ON = os.environ.get("MIC_FILTER", "1") != "0"
HP_HZ = float(os.environ.get("MIC_HP_HZ", "120"))
_notch_raw = os.environ.get("MIC_NOTCH_HZ", "224,244").strip()
NOTCH_HZ = [float(x) for x in _notch_raw.split(",") if x.strip()]
NOTCH_Q = float(os.environ.get("MIC_NOTCH_Q", "25"))

# 写 stderr 的诊断（不污染 stdout 的 PCM）
def diag(msg):
    sys.stderr.write("[mic_front] %s\n" % msg)
    sys.stderr.flush()


# ---------- 二阶高通 + 双陷波（纯 Python DF-II，系数固化，跨帧状态）----------
# 系数由 scipy 离线算出后固化（fs=16000）：避免运行时 import scipy 拖慢首帧。
#   HP120 : butter(2,120,highpass,fs=16000)
#   N224  : iirnotch(224,Q=25,fs=16000)
#   N244  : iirnotch(244,Q=25,fs=16000)
# a0 归一化为 1。改 MIC_HP_HZ / MIC_NOTCH_HZ / MIC_NOTCH_Q 时若非默认组合，
# 惰性 import scipy 现算系数（仅该情形付出 import 成本）。
_BIQUAD_PRESET = {
    ("hp", 120.0): (
        [0.967227282715, -1.93445456543, 0.967227282715],
        [1.0, -1.93338022588, 0.935528904979]),
    ("notch", 224.0, 25.0): (
        [0.998243795978, -1.98876839058, 0.998243795978],
        [1.0, -1.98876839058, 0.996487591955]),
    ("notch", 244.0, 25.0): (
        [0.998087291600, -1.98701795362, 0.998087291600],
        [1.0, -1.98701795362, 0.996174583200]),
}


def _coeffs(kind, freq, q=25.0):
    """取二阶节 (b0,b1,b2,a1,a2)；默认组合查表，否则惰性用 scipy 算。"""
    key = ("hp", freq) if kind == "hp" else ("notch", freq, q)
    if key in _BIQUAD_PRESET:
        b, a = _BIQUAD_PRESET[key]
    else:
        from scipy.signal import butter, iirnotch   # 非默认参数才触发
        if kind == "hp":
            b, a = butter(2, freq, btype="highpass", fs=RATE)
        else:
            b, a = iirnotch(freq, q, fs=RATE)
    a0 = a[0]
    return (b[0] / a0, b[1] / a0, b[2] / a0, a[1] / a0, a[2] / a0)


class _Biquad:
    """转置直接 II 型二阶节，状态 s1/s2 跨帧保持。"""
    __slots__ = ("b0", "b1", "b2", "a1", "a2", "s1", "s2")

    def __init__(self, coeffs):
        self.b0, self.b1, self.b2, self.a1, self.a2 = coeffs
        self.s1 = self.s2 = 0.0

    def process_into(self, src, dst):
        """src/dst: array('h')；逐样本递推，结果写 dst（in-place 语义）。"""
        b0, b1, b2, a1, a2 = self.b0, self.b1, self.b2, self.a1, self.a2
        s1, s2 = self.s1, self.s2
        for i, x in enumerate(src):
            y = b0 * x + s1
            s1 = b1 * x - a1 * y + s2
            s2 = b2 * x - a2 * y
            if y > 32767:
                y = 32767
            elif y < -32768:
                y = -32768
            dst[i] = int(y)
        self.s1, self.s2 = s1, s2


class FilterChain:
    """对选路后的 mono 施加：二阶 Butterworth 高通 + 若干二阶 IIR 陷波。

    - 高通默认 120Hz：压制结构传导低频/风噪，语音基频（男 85-155Hz）有少量损失，
      但对可懂度无碍；设 MIC_HP_HZ=0 可关闭。
    - 陷波默认 224/244Hz（Q=25）：电机基频线谱，邻带能量是语音带 15-29 倍；
      陷波带宽约 f0/Q≈9Hz，语音带 500-3kHz 完全不动（无损）。
    - 纯 Python DF-II，状态跨帧保持，无块边界效应，流式延迟 0，启动 <0.1s。
    """
    def __init__(self, hp_hz=HP_HZ, notch_hz=NOTCH_HZ, q=NOTCH_Q):
        self.sections = []       # [_Biquad, ...]
        if hp_hz and hp_hz > 0:
            self.sections.append(_Biquad(_coeffs("hp", hp_hz)))
        for f in notch_hz:
            if f and f > 0:
                self.sections.append(_Biquad(_coeffs("notch", f, q)))
        desc = []
        if hp_hz and hp_hz > 0:
            desc.append("HP%.0f" % hp_hz)
        desc += ["N%.0f" % f for f in notch_hz if f and f > 0]
        diag("filter chain init: %s q=%.0f sections=%d (pure-python, voice band 500-3k untouched)"
             % ("+".join(desc) if desc else "NONE", q, len(self.sections)))

    def process(self, samples):
        """samples: array('h') 长度 FRAME_SMP；返回滤波后等长 array('h')。"""
        cur = samples
        for sec in self.sections:
            out = array.array("h", [0]) * len(cur)
            sec.process_into(cur, out)
            cur = out
        return cur


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
        # SpeexPreprocess ctl 常量（标准 speex_preprocess.h，2026-11-26 修正）
        # 旧代码用非标编号（DENOISE=2/AGC=3/NS=5）实际命中 SET_AGC/GET_AGC/GET_VAD，
        # 导致 AGC 误开、抑制强度从未设置。真人 A/B 判定 L3 削语音，默认关闭；
        # 此处仍修正为标准值，避免日后误用。
        self.SET_DENOISE = 0
        self.SET_AGC = 2
        self.SET_VAD = 4
        self.SET_NOISE_SUPPRESS = 18     # 噪声抑制强度 dB（负值）
        self.SET_DEREVERB = 8
        one = ctypes.c_int(1)
        zero = ctypes.c_int(0)
        supp = ctypes.c_int(-18)         # −18 dB（speex 要求负值）；真人 A/B 判定仍削语音，默认关闭 L3
        self.lib.speex_preprocess_ctl(self.st, self.SET_DENOISE, ctypes.byref(one))
        self.lib.speex_preprocess_ctl(self.st, self.SET_AGC, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_VAD, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_DEREVERB, ctypes.byref(zero))
        self.lib.speex_preprocess_ctl(self.st, self.SET_NOISE_SUPPRESS, ctypes.byref(supp))
        diag("speex NS init: frame=%d rate=%d denoise=on supp=-18dB agc=off "
             "(NOTE: human-voice A/B 2026-11-26 showed speech attenuated; keep L3 off)" % (self.frame, RATE))

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
    ap.add_argument("--no-filter", action="store_true")
    ap.add_argument("--fixed", type=int, default=None, choices=[0, 1, 2])
    args = ap.parse_args()

    ns = None if args.no_denoise else SpeexNS()
    flt = None if (args.no_filter or not FILTER_ON) else FilterChain()
    hyst_lin = 10 ** (HYST_DB / 10.0)
    cmd = ["arecord", "-q", "-D", DEV, "-f", "S16_LE", "-r", str(RATE),
           "-c", str(NCH), "-t", "raw", "-B", "200000", "-F",
           str(PERIOD_MS * 1000), "-"]
    diag("spawn: %s | nch=%d frame=%dsmp hyst=%.1fdB hold=%d fixed=%s "
         "denoise=%s filter=%s"
         % (" ".join(cmd), NCH, FRAME_SMP, HYST_DB, HOLD, args.fixed,
            not args.no_denoise, flt is not None))
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
                if flt is not None:
                    mono = flt.process(mono)
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
        diag("exit frames=%d dur=%.1fs switches=%d final_ch=%d filter=%s" %
             (frames, time.time() - t0, switches, cur, flt is not None))
        if ns is not None:
            ns.close()


if __name__ == "__main__":
    main()
