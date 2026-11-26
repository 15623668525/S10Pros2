#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S10 遥控应用后端 v2.9（纯标准库 HTTP + rclpy，零 pip 依赖）
功能：
  - 运动状态转换 /MOTION_STATE（站立1 / 软急停2 / 趴下4）
  - 步态切换 /GAIT（平地0x3002 / 楼梯0x3003 / 高台0x1002）
  - 速度控制 /STEER（20Hz 下发，三档量化，松手持续发零 ≥1.5s，须先 ARM）
    2026-11-16 实测：/NAV_CMD 对机器人无效，有效通路是 /STEER（drdds/msg/Steer）
    vel_x ≈ 0.115 + 1.19·x；data.y 无左右通道；转向走 data.yaw
  - 状态反馈 /MOTION_INFO（20Hz 订阅缓存）
  - 电池状态 /BATTERY_DATA（v2.1 电量/电压/电流/温度；v2.2 充电中判定）
  - 语音播报：预置音效走 /AUDIO/COMMAND（DDS 直发）；自定义语音经 103 常驻播放器
    sayd（TCP 推流，按需开设备/播完即释放，v2.4 取代 SSH+aplay）
  - 摄像头预览（v2.6）：RTSP→MJPEG 低时延流，帧总线共享，空闲 20s 自动停
落盘约束：本应用所有数据仅写 /S10ykRemove/
"""
import json
import os
import queue
import select
import socket
import ssl
from collections import deque
import struct
import subprocess
import hmac
import io
import threading
import time
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from drdds.msg import MotionState, Gait, NavCmd, MotionInfo, BatteryData

try:
    from drdds.msg import Steer  # 2026-11-16：实测有效的运动控制通道
except Exception:
    Steer = None

try:
    from drdds.msg import StdStatus  # /AUDIO/STATUS
except Exception:
    StdStatus = None

AUDIO_YAML = "/opt/robot/share/robot_drivers/audio_node_assets/audio.yaml"
FRONTEND = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend", "index.html"))
PRESET_DUR = {}   # {audio_id: 时长 ms}，前端据此在单次播放结束后自动取消框选

# ---- 自定义语音通路（v2.4）----
# 厂商音频节点只认它自己容器内的 9 条预置音效，新增 ID 无效（audio.yaml 不热加载）；
# 103 业务板有声卡，由常驻播放器 sayd 统一代理：按需 open、播完即 close，
# 杜绝"遗留 aplay 长期独占 plughw:0,0 导致厂商提示音全部丢失"的问题。
ASSET_DIR = os.environ.get("S10YK_ASSET_DIR", "/S10ykRemove/app/assets")
SSH_KEY = os.environ.get("S10YK_SSH_KEY", "/S10ykRemove/app/keys/id_s10yk")
SAY_HOST = os.environ.get("S10YK_SAY_HOST", "user@10.21.33.103")
SAYD_HOST = os.environ.get("S10YK_SAYD_HOST", "10.21.33.103")
SAYD_PORT = int(os.environ.get("S10YK_SAYD_PORT", "18081"))
SAYD_SERVICE = "s10yk-say"
SAY_CHUNK = 3200            # 100ms @16k/mono/16bit，实测无断音
SAY_LOOP_GAP = float(os.environ.get("SAY_LOOP_GAP", "0"))  # 循环轮次间空档（秒），>1.3 才真正释放设备

# ---- 单向喊话（2026-11-17）----
# 麦克风 PCM 经 HTTPS 上传 → 后端灌入 sayd（与 say_wav 同构，sayd 不改）。
# 会话式接口（start/pcm/stop）规避 BaseHTTPRequestHandler 解析 chunked 的麻烦。
# REQ-039 实时对讲：前端每 100ms 推一片，故放宽容错（手机网络抖动常见 1~2s 空档），
# 时长上限放宽到 120s（按住说话超过 2 分钟已属异常）。
SHOUT_IDLE_S = float(os.environ.get("S10YK_SHOUT_IDLE", "5.0"))   # 无新 PCM 多久自动停
SHOUT_MAX_S = float(os.environ.get("S10YK_SHOUT_MAX", "120.0"))   # 单次对讲最长时长
SHOUT_MAX_BYTES = 8 * 1024 * 1024   # clip 单次上传上限 8MB（≈4min@16k/mono）
SHOUT_TX = threading.Lock()         # sayd 写串行化：多线程/多连接并发推片时保证帧不交错
# TLS（自签）：配置 cert+key 即在额外端口起 HTTPS 监听，与明文共用同一实例状态
TLS_CERT = os.environ.get("S10YK_TLS_CERT", "")
TLS_KEY = os.environ.get("S10YK_TLS_KEY", "")
TLS_PORT = int(os.environ.get("S10YK_TLS_PORT", "18443"))

# ---- 访问鉴权（v2.7 / D-003）----
# 后端已暴露公网（frp 20081），无鉴权等于把狗交给任何人。
# 采用 Cookie 而非纯 Authorization 头：摄像头是 <img src="/api/cam/stream">，
# <img> 无法附加自定义请求头，只有 Cookie 会被浏览器自动带上。
AUTH_TOKEN = os.environ.get("S10YK_TOKEN", "").strip()
AUTH_COOKIE = "s10yk_token"
AUTH_TTL = int(os.environ.get("S10YK_AUTH_TTL", "604800"))  # 7 天
AUTH_OPEN = ("/", "/index.html", "/favicon.ico", "/api/auth", "/api/login", "/api/bw",
             "/shout-processor.js")

# ---- 带宽测速（REQ-029）：预生成固定大小数据块，前端计时下载算下行带宽 ----
BW_BLOCK = b"\0" * (512 * 1024)   # 512 KiB，够平滑又不过度占流量

# 播报音量（103 声卡 ES8323 的 PCM 混音，0-192 对应 0-100%）
VOLUME_TARGET = 100          # 默认播报音量目标（需求：100%）
VOLUME = {"pct": None, "ts": 0.0}
SAY = {"sock": None, "stop": None, "name": "", "ts": 0.0, "active": False}   # 最新一路（兼容旧状态展示）
# D-040 叠音根治：全部活动喂流注册表。此前单槽结构下，同秒双发（前端双击/多端各发一次/
# 网络抖动重试）会让 say_stop 只停到"最新"一路，最早的一路成孤儿永久循环喂流——
# sayd 侧同一 aplay 被两路 100ms 分片交错喂入，听感即"两股音频前后有时差"（2026-11-20 OPS-89）。
SAY_FEEDERS = []                                    # [{"ev","sock","name","ts"}]，LOCK 保护
SAY_LAST_PLAY = {"key": "", "ts": 0.0}              # 同文件幂等窗口：1s 内重复下发直接忽略
# ---- 人体检测预警（REQ-047 一期：204 上 s10yk-persond 写 latest.json，本进程读取+播报）----
PERSON_DIR = os.environ.get("S10YK_PERSON_DIR", "/opt/s10yk/data/person")   # REQ-048：DATA 目录
PERSON_LATEST = os.path.join(PERSON_DIR, "latest.json")
PERSON_SWITCHES = os.path.join(PERSON_DIR, "switches.json")   # REQ-049：4 路检测开关（本进程写、s10yk-persond 读）
PERSON_CAMS = (("video4", "前视"), ("video0", "后视"), ("video2", "左视"), ("video6", "右视"))
PERSON_VOICE_GAP = float(os.environ.get("S10YK_PERSON_VOICE_GAP", "25"))  # 语音最小间隔（s）
PERSON_ALERT_TTL = float(os.environ.get("S10YK_PERSON_TTL", "60"))        # 前端横幅有效期（s）
PERSON = {"spoken_ts": 0.0, "last_speak": 0.0}
# 喊话会话：与 SAY 并列但独立；sayd 端是全局单例 PLAYER，任一播放开始必须先停另一个
SHOUT = {"sock": None, "start_ts": 0.0, "last_ts": 0.0, "bytes": 0,
         "active": False, "mode": "", "ev": None, "rate": 16000, "channels": 1}

GAIT_FLAT = 0x3002   # 平地（导航/敏捷运动模式）
GAIT_STAIR = 0x3003  # 楼梯

# /AUDIO/COMMAND : [op(0=PLAY,1=STOP), audio_id, interval_ms, repeat(0=无限), priority(0-9), preempt(0=WAIT,1=IF_HIGHER,2=FORCE)]
AUDIO_PLAY, AUDIO_STOP = 0.0, 1.0
# /AUDIO/CONFIG  : [config_type(0=SET_VOICE,1=Voice,2=Voiceplay), value]
CFG_SET_VOICE, CFG_VOICE, CFG_VOICE_PLAY = 0.0, 1.0, 2.0

# ---------------- 共享状态 ----------------
LOCK = threading.Lock()

# ---- REQ-029：前后端 HTTP 交互双向流量计量（前端实时带宽显示）----
# 上行 = 前端请求字节（请求行+请求头+请求体，_meter_request 在 do_GET/do_POST 入口累加）；
# 下行 = 后端响应字节（摄像头 MJPEG 帧 + 各业务 API 的 _send 响应）。
# 覆盖控制/音频指令、状态轮询、视频等全部交互；/api/bw 探测块不计入（人为流量）。
# 用最近 3s 滑动窗口算双向合计 bps。
NET_WINDOW_S = 3.0
NET_EVENTS = deque()          # [(ts, nbytes), ...]
NET_LOCK = threading.Lock()


def net_add(nbytes):
    if nbytes <= 0:
        return
    now = time.time()
    with NET_LOCK:
        NET_EVENTS.append((now, int(nbytes)))
        cut = now - NET_WINDOW_S
        while NET_EVENTS and NET_EVENTS[0][0] < cut:
            NET_EVENTS.popleft()


def net_bps():
    """返回最近 NET_WINDOW_S 内的前后端双向平均速率（bit/s）。"""
    now = time.time()
    with NET_LOCK:
        cut = now - NET_WINDOW_S
        while NET_EVENTS and NET_EVENTS[0][0] < cut:
            NET_EVENTS.popleft()
        total = sum(n for _, n in NET_EVENTS)
        if not NET_EVENTS:
            return 0.0
        span = max(0.5, min(NET_WINDOW_S, now - NET_EVENTS[0][0]))
    return total * 8.0 / span


MOTION_INFO = {}
BATTERY_INFO = {}
AUDIO_STATUS = {}
AUDIO_EVT = {"ts": 0.0}   # 最近一次收到 /AUDIO/STATUS 的时间（用于判断指令是否被调度器接受）
ARMED = False
# REQ-042：休眠/唤醒切换（D-039 休眠事件后新增）。
# 厂商协议没有独立"休眠"指令（ASDU 1.2.3 / 话题 2.2.1 均无；0x1001 阻尼趴下仅 ASDU 表有、
# 话题实测无效）。物理等效序列 = 趴下(4) → 贴地 → 关节阻尼(2)（关节下电，
# 与厂商休眠 CSV 关节列全 0 一致）；唤醒复用站立序列 1→17（stand_sequence）。
SLEEP_REQ = False
SLEEP_REQ_TS = 0.0   # D-040：最近一次休眠请求时刻，用于给遥测自动清标志留趴下过渡宽限
NAV_CMD = {"x": 0.0, "y": 0.0, "yaw": 0.0}
NAV_LAST_TS = 0.0
NAV_LIMIT = {"x": 0.6, "y": 0.4, "yaw": 0.8}  # 旧 /NAV_CMD 限幅（m/s, rad/s），仅备用

# ---- /STEER 三档速度（2026-11-17 全量程实测，见 docs/速度范围实测报告_20261117.md）----
# REQ-033：档位由前端按钮显式选择（低/中/高，默认中速），不再由摇杆幅值量化。
# 前端十字盘只发方向（±1），后端按当前 GEAR_IDX 取对应档位的 x/yaw 发 /STEER（20Hz 持续下发）。
# 实测拟合 vel_x ≈ 0.064 + 1.510·x（全量程 R²=0.993），饱和点 x≈1.0~1.1（约 1.3~1.5 m/s）。
# 反推：x=0.16/0.47/0.81 → 0.3/0.8/1.3 m/s（低/中/高）
STEER_GEAR_X = (0.16, 0.47, 0.81)
# 转向配套（vel_yaw ≈ 1.22·yaw 左 / 1.32 右）：yaw=0.15/0.33/0.50 → 0.19/0.40/0.62 rad/s
STEER_GEAR_YAW = (0.15, 0.33, 0.50)
GEAR_NAMES = ("slow", "mid", "fast")        # 低 / 中 / 高
GEAR_IDX = 1                                 # 当前档位索引，默认 1=中速（0=低, 2=高）
STEER_DEADZONE = 0.15      # 摇杆死区（|stick| 小于此值视为 0）
STEER_ZERO_HOLD_S = 1.5    # 刹停惯性 1.5~2s，松手后必须持续发零
STEER_PERIOD_S = 0.05      # 20Hz
# data.y（左右平移）实测无效，API 层忽略并回传 y_ignored


def steer_gear(stick, gears):
    """摇杆方向 → 当前档位取值。返回 (值, 档位名)。
    REQ-033：幅值只判方向/死区，速度大小由全局 GEAR_IDX 决定。"""
    a = abs(float(stick))
    if a < STEER_DEADZONE:
        return 0.0, "stop"
    v, name = gears[GEAR_IDX], GEAR_NAMES[GEAR_IDX]
    return (v if stick > 0 else -v), name
LOG = []

# D-041 日志防阻塞（2026-11-20）：journald 一旦停摆/限流不再消费 stdout 管道，
# print 会永久阻塞——此前 log() 在 LOCK 内 print，一阻塞就把全部业务线程拖死
# （实测 2026-11-20 16:31 整机无响应，py-spy 全部 handler 卡在 log 的 print）。
# 改造：锁内只入内存环形列表；stdout/落盘交给专职泵线程；队列满即丢，永不阻塞业务。
_LOGQ = queue.Queue(maxsize=2000)
_LOG_FP = None
_LOG_PUMP_ON = False


def _log_pump():
    global _LOG_FP
    n = 0
    while True:
        line = _LOGQ.get()
        try:
            n += 1
            try:
                if _LOG_FP is None:
                    try:
                        os.makedirs("/opt/s10yk/logs", exist_ok=True)
                        _LOG_FP = open("/opt/s10yk/logs/backend.log", "a",
                                       encoding="utf-8", buffering=1)   # 行缓冲
                    except Exception:
                        _LOG_FP = False   # 打不开就不再尝试，纯内存
                if _LOG_FP:
                    try:
                        # 每 2000 行查一次体积，>30MB 滚动到 .old（防无限增长）
                        if n % 2000 == 0 and os.path.getsize("/opt/s10yk/logs/backend.log") > 30 * 1024 * 1024:
                            _LOG_FP.close()
                            _LOG_FP = None
                            try:
                                os.replace("/opt/s10yk/logs/backend.log",
                                           "/opt/s10yk/logs/backend.log.old")
                            except Exception:
                                pass
                        else:
                            _LOG_FP.write(line + "\n")
                    except Exception:
                        _LOG_FP = None
            except Exception:
                pass
            try:
                print(line, flush=True)   # 管道堵也只堵泵线程自己
            except Exception:
                pass
        except Exception:
            time.sleep(0.1)


def _log_pump_start():
    global _LOG_PUMP_ON
    with LOCK:
        if _LOG_PUMP_ON:
            return
        _LOG_PUMP_ON = True
        threading.Thread(target=_log_pump, daemon=True, name="log-pump").start()


def log(msg):
    line = time.strftime("%H:%M:%S") + " " + str(msg)
    with LOCK:
        LOG.append(line)
        del LOG[:-200]
    try:
        _LOGQ.put_nowait(line)   # 满即丢：日志绝不能阻塞业务线程（D-041）
    except Exception:
        pass
    if not _LOG_PUMP_ON:
        _log_pump_start()


def load_presets():
    try:
        import yaml
        with open(AUDIO_YAML, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        items = []
        for a in cfg.get("audios", []):
            items.append({
                "id": int(a.get("id", 0)),
                "name": a.get("name", ""),
                "file": a.get("file", ""),
                "custom": False,
                "say": False,
            })
        items.sort(key=lambda i: i["id"])
        vol = (cfg.get("play") or {}).get("volume")
        return {"presets": items + scan_assets(), "volume": vol}
    except Exception as e:
        log("load_presets error: %r" % e)
        return {"presets": scan_assets(), "volume": None}


def scan_assets():
    """本地自定义语音（/S10ykRemove/app/assets/*.wav），走 SSH+aplay 通路
    显示名优先取 manifest.json：{"voices":[{"file":"x.wav","name":"中文名"}]}
    """
    out = []
    try:
        names = sorted(n for n in os.listdir(ASSET_DIR) if n.lower().endswith(".wav"))
    except Exception:
        return out
    disp = {}
    try:
        with open(os.path.join(ASSET_DIR, "manifest.json"), "r", encoding="utf-8") as f:
            for v in (json.load(f) or {}).get("voices", []):
                disp[v.get("file")] = v.get("name") or v.get("file")
    except Exception:
        pass
    for i, n in enumerate(names):
        out.append({
            "id": 1000 + i,
            "name": disp.get(n, os.path.splitext(n)[0]),
            "file": n,
            "custom": True,
            "say": True,
        })
    return out


def _wav_duration_ms(path):
    """wav 时长（ms）

    注意：厂商部分 wav（如 wozai.wav）头部 data 长度是流式写入的占位值，
    `getnframes()` 会给出天文数字（实测 573636608 帧 / 9.9 小时）。
    因此用「文件实际大小」做合理性校验，头部不可信时按 PCM 字节数重算。
    """
    try:
        size = os.path.getsize(path)
    except Exception:
        size = 0
    try:
        with wave.open(path, "rb") as w:
            rate = w.getframerate() or 16000
            ba = (w.getsampwidth() or 2) * (w.getnchannels() or 1)
            n = w.getnframes()
            if size and ba and (n <= 0 or n * ba > size):
                n = max(0, size - 44) // ba          # 头部帧数不可信 → 按文件大小估
            return int(round(n * 1000.0 / max(1, rate)))
    except Exception:
        try:
            _, pcm, byte_rate, _ = _wav_header_and_pcm(path)
            return int(round(len(pcm) * 1000.0 / max(1, byte_rate)))
        except Exception:
            return 0


def load_preset_durations():
    """读取厂商 audio.yaml + wav，缓存每条预置音效时长（ms）

    用途：前端单次播报结束后按该时长自动取消按钮框选，
    因为 /AUDIO/STATUS 在播放过程中始终是 IDLE，拿不到结束事件。
    """
    try:
        import yaml
        with open(AUDIO_YAML, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        voice = (cfg.get("voice") or {}).get("default", "default")
        base = os.path.dirname(AUDIO_YAML)
        dur = {}
        for a in cfg.get("audios", []) or []:
            rel = str(a.get("file", "")).replace("{voice}", voice)
            p = os.path.join(base, rel)
            d = _wav_duration_ms(p) if os.path.exists(p) else 0
            if d > 0:
                dur[int(a.get("id", 0))] = d
        PRESET_DUR.update(dur)
        log("preset durations(ms): %s" % dur)
        return dur
    except Exception as e:
        log("load_preset_durations error: %r" % e)
        return {}


class S10Bridge(Node):
    def __init__(self):
        super().__init__("s10yk_bridge")
        self.pub_state = self.create_publisher(MotionState, "/MOTION_STATE", 1)
        self.pub_gait = self.create_publisher(Gait, "/GAIT", 1)
        self.pub_nav = self.create_publisher(NavCmd, "/NAV_CMD", 1)
        # /STEER 是机器人真正执行的运动控制通道（2026-11-16 实测）
        self.pub_steer = self.create_publisher(Steer, "/STEER", 10) if Steer is not None else None
        if self.pub_steer is None:
            log("!! drdds.msg.Steer 不可用，运动控制退回 /NAV_CMD（对机器人无效）")
        self.pub_audio_cmd = self.create_publisher(Float64MultiArray, "/AUDIO/COMMAND", 10)
        self.pub_audio_cfg = self.create_publisher(Float64MultiArray, "/AUDIO/CONFIG", 10)
        self.create_subscription(MotionInfo, "/MOTION_INFO", self.on_motion, 1)
        self.create_subscription(BatteryData, "/BATTERY_DATA", self.on_battery, 1)
        if StdStatus is not None:
            self.create_subscription(StdStatus, "/AUDIO/STATUS", self.on_audio, 1)
        self._frame = 0
        log("S10Bridge node ready")

    def _header(self, msg):
        self._frame = (self._frame + 1) % 0xFFFFFFFFFFFFFFFF
        try:
            now = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame
            msg.header.stamp.sec = now.sec
            msg.header.stamp.nanosec = now.nanosec
        except Exception:
            pass
        return msg

    def on_motion(self, msg):
        try:
            info = {
                "vel_x": round(float(msg.data.vel_x), 3),
                "vel_y": round(float(msg.data.vel_y), 3),
                "vel_yaw": round(float(msg.data.vel_yaw), 3),
                "height": round(float(msg.data.height), 3),
                "state": int(msg.data.motion_state.state),
                "gait": int(msg.data.gait_state.gait),
                "remain_mile": round(float(getattr(msg.data, "remain_mile", 0.0)), 1),
                "ts": time.time(),
            }
        except Exception as e:
            log("on_motion parse error: %r" % e)
            return
        with LOCK:
            global MOTION_INFO
            MOTION_INFO = info

    def on_battery(self, msg):
        # BatteryDataValue: voltage=10mV / current=10mA / battery_level=% / battery_temperature=℃[]
        try:
            d = msg.data[0] if msg.data else None
            if d is None:
                return
            temps = [float(t) for t in (d.battery_temperature or [])]
            # 充电判定（实测 2026-11-14）：充电 I≈+1.6A、放电 I≈-0.7~-1.4A，阈值 0.3A
            info = {
                "level": int(d.battery_level),                      # 电量百分比 0-100
                "voltage": round(int(d.voltage) / 100.0, 2),        # 总电压 V
                "current": round(int(d.current) / 100.0, 2),        # 总电流 A（正充负放）
                "charging": int(d.current) > 30,                    # >0.3A 视为充电中
                "temp": round(max(temps), 1) if temps else None,    # 最高电池温度 ℃
                "cycles": int(d.cycles),                            # 循环次数
                "ts": time.time(),
            }
        except Exception as e:
            log("on_battery parse error: %r" % e)
            return
        with LOCK:
            global BATTERY_INFO
            BATTERY_INFO = info

    def on_audio(self, msg):
        # /AUDIO/STATUS: state 0=IDLE 1=BUSY 2=FAULT ; error_code 字段实为 current_audio_id
        try:
            st = {"state": int(msg.state), "id": int(msg.error_code)}
        except Exception:
            st = {}
        with LOCK:
            global AUDIO_STATUS
            AUDIO_STATUS = st
            AUDIO_EVT["ts"] = time.time()

    def publish_state(self, state_val):
        m = MotionState()
        self._header(m)
        m.data.state = int(state_val)
        self.pub_state.publish(m)
        log("MOTION_STATE -> %d" % state_val)

    def publish_gait(self, gait_val):
        g = Gait()
        self._header(g)
        g.data.gait = int(gait_val) & 0xFFFFFFFF
        self.pub_gait.publish(g)
        log("GAIT -> 0x%04X" % gait_val)

    def publish_nav(self, x, y, yaw):
        n = NavCmd()
        self._header(n)
        n.data.x_vel = float(x)
        n.data.y_vel = float(y)
        n.data.yaw_vel = float(yaw)
        self.pub_nav.publish(n)

    def publish_steer(self, x, yaw):
        """发 /STEER：data.x=前后（归一化操纵杆量）, data.yaw=转向；data.y/z 无通道，恒 0"""
        if self.pub_steer is None:
            # 兜底：Steer 类型缺失时至少把 yaw 当 yaw_vel 发旧通道（无效果，但不至于崩）
            self.publish_nav(float(x), 0.0, float(yaw))
            return
        s = Steer()
        self._header(s)
        s.data.x = float(x)
        s.data.y = 0.0
        s.data.z = 0.0
        s.data.roll = 0.0
        s.data.pitch = 0.0
        s.data.yaw = float(yaw)
        self.pub_steer.publish(s)

    # ---- 音频：DDS 直发 ----
    def audio_play(self, audio_id, priority=5, preempt=0, repeat=1, interval_ms=0):
        m = Float64MultiArray()
        m.data = [AUDIO_PLAY, float(audio_id), float(interval_ms),
                  float(repeat), float(priority), float(preempt)]
        self.pub_audio_cmd.publish(m)
        log("AUDIO play id=%s pri=%s preempt=%s repeat=%s" % (audio_id, priority, preempt, repeat))

    def audio_stop(self, audio_id=0):
        m = Float64MultiArray()
        m.data = [AUDIO_STOP, float(audio_id), 0.0, 0.0, 9.0, 2.0]
        self.pub_audio_cmd.publish(m)
        log("AUDIO stop id=%s" % audio_id)

    def audio_config(self, ctype, value):
        m = Float64MultiArray()
        m.data = [float(ctype), float(value)]
        self.pub_audio_cfg.publish(m)
        log("AUDIO config type=%s value=%s" % (ctype, value))


BRIDGE = None


def nav_loop():
    """20Hz 下发 /STEER。ARM 后才发——未 ARM 完全静默，避免与遥控器争抢通道。
    松手/断流后持续发零：实测刹停有 1.5~2s 惯性，只发一帧零会继续漂。"""
    global NAV_LAST_TS
    zero_started = None
    tick = 0
    last_note = 0.0
    while True:
        try:
            with LOCK:
                armed = ARMED
                age = time.time() - NAV_LAST_TS
                cmd = dict(NAV_CMD)
            if BRIDGE is None:
                time.sleep(0.2)
                continue
            if not armed:
                zero_started = None
                time.sleep(0.2)
                continue
            if age < 0.3:
                BRIDGE.publish_steer(cmd["x"], cmd["yaw"])
                zero_started = None
                tick = 0
            else:
                tick += 1
                if zero_started is None:
                    zero_started = time.time()
                    if time.time() - last_note > 30:
                        last_note = time.time()
                        log("STEER released -> zero hold(%.1fs)" % STEER_ZERO_HOLD_S)
                holding = (time.time() - zero_started) < STEER_ZERO_HOLD_S
                # 保持期内 20Hz 全程发零；之后降到 2Hz 保活
                if holding or tick % 10 == 0:
                    BRIDGE.publish_steer(0.0, 0.0)
        except Exception as e:
            log("nav_loop error: %r" % e)
        time.sleep(STEER_PERIOD_S)


def sleep_detect(mi):
    """REQ-042：休眠状态判定。

    - 我们的休眠序列 = 趴下(4)→关节阻尼(2)：请求标志 SLEEP_REQ 置位期间视为休眠，
      /MOTION_INFO 回报 state==2 且贴地（h<0.12）视为已确认（急停也发 2，必须结合
      请求标志区分，否则趴下后急停会被误判成休眠）；
    - 厂商休眠（手柄/自动闲置，D-039）表现为 state 恒 0 + 高度贴地 + 忽略指令，
      给出"疑似"提示由前端展示（区别于正常空闲趴地，无法完全区分，仅提示）。
    """
    global SLEEP_REQ
    if not mi:
        return {"sleep": False, "suspect": False}
    try:
        s = int(mi.get("state") or 0)
        h = float(mi.get("height") or 0.0)
    except Exception:
        return {"sleep": False, "suspect": False}
    # D-040：手柄/自动唤醒后 SLEEP_REQ 会粘滞为 True，导致前端方向盘被永久锁死。
    # 下发休眠后 6s 宽限内不自动清（趴下过渡期 state 可能仍是 17、高度未降完）；
    # 之后遥测明确处于站姿（state!=0 且 h>0.30）即视为已唤醒，自动清标志。
    if SLEEP_REQ and time.time() - SLEEP_REQ_TS > 6.0 and s != 0 and h > 0.30:
        SLEEP_REQ = False
        log("SLEEP flag cleared by telemetry: awake (state=%d h=%.3f)" % (s, h))
    confirmed = (s == 0x1001) or (SLEEP_REQ and s == 2 and h < 0.12)
    suspect = (not confirmed and not SLEEP_REQ and s == 0 and h < 0.10)
    return {"sleep": confirmed or SLEEP_REQ, "suspect": suspect}


def sleep_sequence():
    """休眠动作：趴下(4) → 等贴地 → 关节阻尼(2)（关节下电）。

    厂商协议没有独立"休眠"指令：ROS2 /MOTION_STATE 话题只支持 1/2/4/17（指南 2.2.1），
    0x1001 阻尼趴下仅 ASDU 参数表有（指南 1.2.3），话题下发实测无效（2026-11-17 真机：
    0x1001 后 state=17/h=0.387 纹丝不动）。本应用不实现 ASDU，物理等效序列 =
    趴下后切关节阻尼（厂商休眠 = 趴下 + 关节下电，见 D-039 CSV 关节列全 0）。
    """
    try:
        BRIDGE.publish_state(4)
        t0 = time.time()
        while time.time() - t0 < 8.0:
            with LOCK:
                mi = dict(MOTION_INFO)
            h = float(mi.get("height") or 1.0)
            if h < 0.12:            # 已贴地（趴稳 h≈0.08）
                break
            time.sleep(0.1)
        time.sleep(0.5)             # 稳半秒再下电，避免趴下动作未完成
        BRIDGE.publish_state(2)     # 关节阻尼 → 关节下电（与厂商休眠的物理表现一致）
        log("SLEEP seq: 4(趴下) -> 2(关节阻尼/下电), h=%.3f" % h)
    except Exception as e:
        log("sleep_sequence error: %r" % e)


def stand_sequence():
    """站立动作：先下发 1（起立），高度稳定后补发 17（RL），否则停在半蹲姿态。

    依据《S10 软件开发指南》2.2.1：通过 /MOTION_STATE 话题下发站立指令后，
    机器狗不会自动进入 RL 控制态（ASDU/遥控器接口才会自动进入），
    实测 站立(1) 高度 0.2266m，补发 RL(17) 后 0.3865m 才是遥控器的全站姿。

    v2.5 修复：刚下发 1 时 MOTION_INFO 还是上一帧缓存（state=4），
    旧逻辑会立刻判"趴下中止"，导致 17 永远发不出去、狗停在半蹲。
    现在要"已观察到起身"之后再看到 4 才算用户中途趴下；
    补 17 后回读高度确认到全站姿，没到就重试一次。
    """
    try:
        BRIDGE.publish_state(1)
        t0 = time.time()
        last_h = 0.0
        stable = 0
        seen_up = False          # 是否已观察到"离开趴下"的迹象
        while time.time() - t0 < 8.0:
            with LOCK:
                mi = dict(MOTION_INFO)
            h = float(mi.get("height") or 0.0)
            s = int(mi.get("state") or 0)
            now = time.time() - t0
            if s in (1, 17) or h >= 0.15:
                seen_up = True
            # 只有"起身之后"再次变回 4 才是用户中途趴下
            if s == 4 and seen_up and now > 0.8:
                log("STAND aborted: state=4 (用户趴下)")
                return
            if now > 1.5 and s == 4 and h < 0.12:
                log("STAND aborted: 起立未生效（仍 state=4 h=%.3f）" % h)
                return
            if h >= 0.20:
                if last_h and abs(h - last_h) < 0.002:
                    stable += 1
                else:
                    stable = 0
                if stable >= 3:
                    break
            last_h = h
            time.sleep(0.1)
        # 补 17 并确认真的到全站姿（≥0.33m），未到则再补一次
        h = last_h
        for attempt in (1, 2):
            BRIDGE.publish_state(17)
            t1 = time.time()
            ok = False
            while time.time() - t1 < 4.0:
                with LOCK:
                    h = float(dict(MOTION_INFO).get("height") or 0.0)
                if h >= 0.33:
                    ok = True
                    break
                time.sleep(0.15)
            if ok:
                log("STAND seq ok: 1 -> 17, height=%.3f (try %d)" % (h, attempt))
                # v2.8：站立后必须挂步态，否则 gait=0，摇杆前后左右/转向全部被忽略
                try:
                    BRIDGE.publish_gait(4097)          # 0x1001 平地步态
                    log("STAND -> auto GAIT 0x1001（不挂步态摇杆无效）")
                except Exception as e:
                    log("STAND auto GAIT failed: %r" % e)
                return
        log("STAND warn: 补 17 两次后高度仍 %.3f（未到全站姿）" % h)
    except Exception as e:
        log("stand_sequence error: %r" % e)


def audio_bootstrap():
    """启动后确保播报开关打开（play.enable / voice.enable）"""
    time.sleep(1.5)
    try:
        BRIDGE.audio_config(CFG_VOICE_PLAY, 1)
        time.sleep(0.3)
        BRIDGE.audio_config(CFG_VOICE, 1)
    except Exception as e:
        log("audio bootstrap error: %r" % e)


def _ssh103(cmd, timeout=8):
    """在 103 业务板上执行一条命令（免密钥通道），返回 (rc, stdout, stderr)"""
    r = subprocess.run(["ssh", "-i", SSH_KEY,
                        "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no",
                        "-o", "ConnectTimeout=4", "-o", "LogLevel=ERROR",
                        SAY_HOST, cmd],
                       capture_output=True, timeout=timeout)
    return (r.returncode,
            (r.stdout or b"").decode("utf-8", "ignore").strip(),
            (r.stderr or b"").decode("utf-8", "ignore").strip())


def read_volume():
    """读 103 声卡 PCM 当前百分比（0-100），失败返回 None"""
    try:
        rc, out, _ = _ssh103("amixer -c 0 sget PCM 2>/dev/null | grep -o '[0-9]*%' | head -1")
        if rc == 0 and out.endswith("%"):
            pct = int(out[:-1])
            with LOCK:
                VOLUME["pct"] = pct
                VOLUME["ts"] = time.time()
            return pct
    except Exception as e:
        log("read_volume error: %r" % e)
    return None


def set_volume(pct):
    """设置 103 声卡 PCM 音量（百分比）"""
    pct = max(0, min(100, int(pct)))
    try:
        rc, _, err = _ssh103("amixer -c 0 sset PCM %d%%" % pct)
        if rc == 0:
            with LOCK:
                VOLUME["pct"] = pct
                VOLUME["ts"] = time.time()
            log("VOL %d%%" % pct)
            return pct
        log("VOL set %d%% failed rc=%s %s" % (pct, rc, err[:120]))
    except Exception as e:
        log("set_volume error: %r" % e)
    return None


def volume_keeper():
    """默认播报音量锁 100%：启动即设；每 60s 巡检，若被外部改小/复位则拉回

    音量无任何话题可用（/AUDIO/CONFIG 只支持音色），只能操作 amixer 混音器控件；
    该操作不打开 PCM 流，与播放互不阻塞，故不算"抢占硬件"。巡检从 20s 放宽到 60s，
    减少 SSH 频次（每次约 200ms 建链开销）。
    """
    time.sleep(2)
    try:
        set_volume(VOLUME_TARGET)
    except Exception as e:
        log("volume_keeper init error: %r" % e)
    while True:
        try:
            pct = read_volume()
            if pct is not None and pct != VOLUME_TARGET:
                log("VOL drift %d%% -> %d%%" % (pct, VOLUME_TARGET))
                set_volume(VOLUME_TARGET)
        except Exception as e:
            log("volume_keeper error: %r" % e)
        time.sleep(60)


F_START, F_PCM, F_STOP, F_PING = 1, 2, 3, 4   # sayd 帧类型


def _say_frame(sock, ftype, payload=b""):
    sock.sendall(struct.pack(">BI", ftype, len(payload)) + payload)


def _say_reply(sock):
    """读 sayd 的一行文本回复（OK / BUSY … / ERR …）"""
    buf = b""
    while not buf.endswith(b"\n") and len(buf) < 256:
        try:
            ch = sock.recv(1)
        except Exception:
            break
        if not ch:
            break
        buf += ch
    return buf.decode("utf-8", "ignore").strip()


def _sayd_restart():
    """sayd 掉线时拉一次 systemd（103 免密钥通道）"""
    try:
        rc, _, err = _ssh103("sudo systemctl restart %s" % SAYD_SERVICE, timeout=15)
        log("SAY restart %s rc=%s %s" % (SAYD_SERVICE, rc, err[:120]))
    except Exception as e:
        log("SAY restart error: %r" % e)


def _say_kill_feeder(fe):
    """停掉单个喂流：置事件 + 发 F_STOP + 断开（sayd 收到 STOP/EOF 即关 aplay 释放设备）"""
    ev = fe.get("ev")
    if ev is not None:
        ev.set()
    sock = fe.get("sock")
    fe["sock"] = None
    if sock is not None:
        try:
            _say_frame(sock, F_STOP)
        except Exception:
            pass
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass


def say_stop():
    """停止自定义语音：停掉**全部**活动喂流（D-040），并清空幂等窗口

    - 逐路 ev.set() + F_STOP + 断连，sayd 端最终只剩一个被 STOP 的会话；
    - 清空 SAY_LAST_PLAY：先停后放（stop → 立即重播同一文件）不受幂等窗口影响。
    """
    with LOCK:
        feeders = list(SAY_FEEDERS)
        SAY_FEEDERS.clear()
        SAY.update({"sock": None, "stop": None, "active": False})
        SAY_LAST_PLAY["key"] = ""
    n = 0
    for fe in feeders:
        _say_kill_feeder(fe)
        n += 1
    if n > 1:
        log("SAY stop -> killed %d feeder(s) (叠音风险已清除)" % n)
    if n == 1:
        log("SAY stop -> device released")
        return True
    # 控制连接已断（例如推流线程已退出），但 sayd 可能仍在播：补发一帧 STOP
    try:
        s = socket.create_connection((SAYD_HOST, SAYD_PORT), timeout=1.5)
        _say_frame(s, F_STOP)
        _say_reply(s)
        s.close()
        log("SAY stop (orphan) -> device released")
        return True
    except Exception:
        return False


def play_preset(audio_id, priority=5, repeat=1, interval_ms=0, preempt=2):
    """播放预置音效（走 /AUDIO/COMMAND）：先 STOP 清场，再以 FORCE 抢占下发

    背景（实测）：厂商调度器一旦被"无限循环 + preempt=WAIT"的项卡住，
    后续所有播报会被静默丢弃（/AUDIO/STATUS 完全不再翻转，CONFIG 也无回应），
    只能靠 STOP + FORCE 解锁。故此处统一"先停后放"，并在 500ms 内收不到任何
    状态翻转时自动补发一次，避免用户点按钮没反应。
    """
    BRIDGE.audio_stop(0)
    time.sleep(0.05)
    for attempt in (0, 1):
        t0 = time.time()
        BRIDGE.audio_play(audio_id, priority=priority, preempt=preempt,
                          repeat=repeat, interval_ms=interval_ms)
        deadline = t0 + 0.5
        while time.time() < deadline:
            with LOCK:
                if AUDIO_EVT["ts"] >= t0:
                    return True
            time.sleep(0.03)
        if attempt == 0:
            log("PLAY id=%s 无状态翻转(500ms) -> 视为调度器卡死，STOP 后重试" % audio_id)
            BRIDGE.audio_stop(0)
            time.sleep(0.1)
    log("PLAY id=%s 重试后仍无响应" % audio_id)
    return False


def _wav_info(path_or_bytes):
    """解析 wav → {pcm, rate, channels, bits, cycle_ms}

    入参可为文件路径或原始 wav 字节（喊话 clip 直接传内存 bytes，无需落盘）。
    注意：厂商部分 wav（如 wozai.wav）头部 data 长度是流式写入的占位值，
    不能用 getnframes()，一律以实际字节数 / 字节率计时。
    """
    if isinstance(path_or_bytes, (bytes, bytearray)):
        raw = bytes(path_or_bytes)
    else:
        with open(path_or_bytes, "rb") as f:
            raw = f.read()
    i = raw.find(b"data")
    if i < 0 or i + 8 > len(raw) or len(raw) < 44:
        raise ValueError("非法 wav 文件（缺少 RIFF/data 头或过短）")
    pcm = raw[i + 8:]
    rate, channels, bits, byte_rate = 16000, 1, 16, 32000
    j = raw.find(b"fmt ")
    if j > 0 and j + 20 <= len(raw):
        try:
            channels = struct.unpack_from("<H", raw, j + 10)[0] or 1
            rate = struct.unpack_from("<I", raw, j + 12)[0] or 16000
            bits = struct.unpack_from("<H", raw, j + 22)[0] or 16
            br = struct.unpack_from("<I", raw, j + 16)[0]
            byte_rate = br or (rate * channels * bits // 8)
        except Exception:
            pass
    cycle_ms = int(round(len(pcm) * 1000.0 / max(1, byte_rate)))
    return {"pcm": pcm, "rate": rate, "channels": channels, "bits": bits,
            "byte_rate": byte_rate, "cycle_ms": cycle_ms}


# ---- 单向喊话核心（2026-11-17）----
def _shout_connect(rate=16000, channels=1):
    """连 sayd 并发 START，返回 sock；失败时拉一次 systemd 再重试一次。"""
    payload = json.dumps({"rate": int(rate), "channels": int(channels)}).encode()
    def _try():
        s = socket.create_connection((SAYD_HOST, SAYD_PORT), timeout=3.0)
        s.settimeout(8.0)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        except Exception:
            pass
        # REQ-039：Nagle 会把 100ms 的 PCM 小片攒出额外延迟，实时对讲必须关掉
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        _say_frame(s, F_START, payload)
        reply = _say_reply(s)
        if not reply.startswith("OK"):
            try:
                s.close()
            except Exception:
                pass
            raise RuntimeError("sayd busy: %s" % (reply or "no reply"))
        return s
    try:
        return _try()
    except Exception as e:
        log("SHOUT connect failed (%r) -> restart sayd" % e)
        _sayd_restart()
        time.sleep(1.0)
        return _try()


def _shout_clear():
    """互斥清理：停自定义播报 + 停 DDS 音效 + 停可能的遗留喊话会话 + 清僵尸 aplay。"""
    say_stop()                       # 停 say_wav 通路（含 orphan STOP）
    BRIDGE.audio_stop(0)             # 停 DDS 预置音效
    with LOCK:
        s = SHOUT.get("sock")
        SHOUT["sock"] = None
        SHOUT["active"] = False
    if s is not None:
        # 推流线程可能正卡在 sendall 上，这里最多等 300ms；拿不到就直接 shutdown 打断它
        got = SHOUT_TX.acquire(timeout=0.3)
        try:
            if got:
                _say_frame(s, F_STOP)
        except Exception:
            pass
        finally:
            if got:
                SHOUT_TX.release()
        try:
            s.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            s.close()
        except Exception:
            pass
    # 清遗留 aplay（sayd 内部单实例本会清理，这里双保险，应对旧版本残留进程）
    try:
        _ssh103("pkill -9 -f 'aplay -q -D plughw' ; true", timeout=6)
    except Exception:
        pass


def shout_stop():
    """供其它音频入口复用：若有喊话会话则停止（幂等）。"""
    with LOCK:
        active = SHOUT.get("active")
    if not active:
        return False
    _shout_clear()
    log("SHOUT stopped by audio entry")
    return True


def _shout_watchdog(ev):
    """看门狗：无新 PCM 超过 SHOUT_IDLE_S、或超过 SHOUT_MAX_S、或收到 stop → 自动收尾。"""
    while not ev.is_set():
        time.sleep(0.2)
        if ev.is_set():
            break
        with LOCK:
            last = SHOUT.get("last_ts", 0.0)
            start = SHOUT.get("start_ts", 0.0)
            active = SHOUT.get("active")
        if not active:
            break
        now = time.time()
        if (now - last) > SHOUT_IDLE_S or (now - start) > SHOUT_MAX_S:
            log("SHOUT watchdog: idle/timeout -> stop")
            _shout_clear()
            break


def _say_frame_bytes(payload=b""):
    """构造一帧 sayd 协议字节（F_PCM + 长度 + payload）。"""
    return struct.pack(">BI", F_PCM, len(payload)) + payload


def shout_clip(wav_bytes):
    """阶段一：整段 WAV → sayd 播完即止。异步推流，HTTP 立即返回 duration_ms。"""
    info = _wav_info(wav_bytes)
    if info["bits"] != 16:
        raise ValueError("仅支持 16bit PCM，收到 %dbit" % info["bits"])
    if info["cycle_ms"] > SHOUT_MAX_S * 1000:
        raise ValueError("音频超过最长时长 %.0fs" % SHOUT_MAX_S)
    _shout_clear()
    sock = _shout_connect(info["rate"], info["channels"])
    ev = threading.Event()
    now = time.time()
    with LOCK:
        SHOUT.update({"sock": sock, "start_ts": now, "last_ts": now, "bytes": 0,
                      "active": True, "mode": "clip", "ev": ev,
                      "rate": info["rate"], "channels": info["channels"]})
    pcm, bps = info["pcm"], max(1, info["byte_rate"])

    def _feed():
        cur = sock
        t0, sent = time.time(), 0
        try:
            for off in range(0, len(pcm), SAY_CHUNK):
                if ev.is_set():
                    return
                chunk = pcm[off:off + SAY_CHUNK]
                with SHOUT_TX:
                    cur.sendall(_say_frame_bytes(chunk))
                sent += len(chunk)
                with LOCK:
                    SHOUT["bytes"] = sent
                    SHOUT["last_ts"] = time.time()
                # 按音频实时速率节流，保证 sayd/aplay 不溢出、停止指令能及时生效
                delay = t0 + sent / float(bps) - time.time()
                if delay > 0:
                    time.sleep(min(delay, 0.2))
        except Exception as e:
            if not ev.is_set():
                log("SHOUT clip feed error: %r" % e)
        finally:
            with LOCK:
                if SHOUT.get("ev") is ev:
                    SHOUT["active"] = False
            try:
                cur.close()
            except Exception:
                pass
            log("SHOUT clip done %.1fs" % (info["cycle_ms"] / 1000.0))

    threading.Thread(target=_feed, daemon=True).start()
    return {"duration_ms": info["cycle_ms"], "bytes": len(pcm)}


def shout_start(rate=16000, channels=1):
    """阶段二：开会话。"""
    rate = int(rate) if rate in (8000, 16000, 22050, 24000, 32000, 44100, 48000) else 16000
    channels = 1 if int(channels) > 0 else 1
    _shout_clear()
    sock = _shout_connect(rate, channels)
    ev = threading.Event()
    now = time.time()
    with LOCK:
        SHOUT.update({"sock": sock, "start_ts": now, "last_ts": now, "bytes": 0, "n": 0,
                      "active": True, "mode": "stream", "ev": ev,
                      "rate": rate, "channels": channels})
    threading.Thread(target=_shout_watchdog, args=(ev,), daemon=True).start()
    log("SHOUT start %dHz/%dch" % (rate, channels))
    return {"ok": True, "rate": rate, "channels": channels}


def _pcm_rms(pcm_bytes):
    """S16_LE PCM 的 RMS（0~1），诊断上行是否为静音（D-036）。"""
    n = len(pcm_bytes) // 2
    if n <= 0:
        return 0.0
    vals = struct.unpack("<%dh" % n, pcm_bytes[:n * 2])
    return (sum(v * v for v in vals) / float(n)) ** 0.5 / 32768.0


def shout_push(pcm_bytes):
    """阶段二：推一片 PCM。返回 False 表示会话已失效。"""
    with LOCK:
        sock = SHOUT.get("sock")
        active = SHOUT.get("active")
        if not active or sock is None:
            return False
        SHOUT["bytes"] += len(pcm_bytes)
        SHOUT["last_ts"] = time.time()
        n = SHOUT.get("n", 0) + 1
        SHOUT["n"] = n
        take_probe = (n % 50 == 1)          # ~每 5s 抽 1 片算 RMS，服务端独立验证静音
        nbytes = SHOUT["bytes"]
    if take_probe:
        import math
        rms = _pcm_rms(pcm_bytes)
        db = 20 * math.log10(rms) if rms > 0 else -99.0
        log("SHOUT pcm#%d rms=%.4f (%.0f dBFS) total=%dKB" % (n, rms, db, nbytes >> 10))
    try:
        # 串行写：ThreadingHTTPServer 下同一会话可能有并发请求，帧字节交错会毁掉音频流
        with SHOUT_TX:
            sock.sendall(_say_frame_bytes(pcm_bytes))
        return True
    except Exception as e:
        log("SHOUT push error: %r" % e)
        _shout_clear()
        return False


def shout_state():
    with LOCK:
        active = SHOUT.get("active")
        mode = SHOUT.get("mode", "")
        start = SHOUT.get("start_ts", 0.0)
        nbytes = SHOUT.get("bytes", 0)
    return {"active": bool(active), "mode": mode,
            "seconds": round(time.time() - start, 1) if active else 0,
            "bytes": int(nbytes)}


# ---------------- 双向对讲（REQ-041，2026-11-17）----------------
# 上行（手机→机器人喇叭）复用 /api/shout/*（sayd 播放）；
# 下行（机器人 MIC→手机）由后端在业务板上拉 arecord，经 chunked HTTP 实时推给浏览器。
# 为什么不把采集塞进 sayd：sayd 是「按需开设备、播完即关」的播放器，加采集会让它长期
# 占住麦克风，反而和厂商节点、会话生命周期纠缠。这里单开一条 SSH+arecord 管道，随会话
# 生灭，断了就重开——顺带能精确统计「MIC 被抢占」次数（busy / 中途被夺 / USB 复位）。
TALK_DEV = os.environ.get("S10YK_TALK_DEV", "plughw:1,0")   # card1 = USB 麦阵列
TALK_RATE = int(os.environ.get("S10YK_TALK_RATE", "16000"))
# OPS-91（2026-11-26）：三咪头前端（选能量最大路 + speex 降噪）。
# MIC 6ch 实测仅 ch0/ch1/ch2 有效，ch1 比 ch0 强 5.7dB；前端脚本在业务板上处理，
# 仍输出 16k/mono/S16_LE，帧结构/队列/前端零改动。
# 一键回退：设 S10YK_TALK_FRONT=0 → 恢复原样 arecord -c 1（固定 ch0）。
TALK_FRONT = os.environ.get("S10YK_TALK_FRONT", "1") != "0"
TALK_FRONT_SCRIPT = os.environ.get("S10YK_TALK_FRONT_SCRIPT", "/home/user/mic_front.py")
# L3 降噪默认关闭：实测 speex 对持续音源衰减 66%（纯 1kHz 正弦 RMS 5657→1944），
# 真实人声虽断续、理论影响较小，但未做真人 A/B 前不默认启用，避免削语音。
# 现场验证通过后在 systemd 里设 S10YK_TALK_DENOISE=1 即可开启，无需改代码。
TALK_DENOISE = os.environ.get("S10YK_TALK_DENOISE", "0") != "0"
TALK_PERIOD_US = int(os.environ.get("S10YK_TALK_PERIOD", "40000"))   # 40ms 一帧，降延迟
TALK_BUF_US = int(os.environ.get("S10YK_TALK_BUFFER", "200000"))     # 采集缓冲 200ms
TALK_FRAME = max(256, TALK_RATE * 2 * TALK_PERIOD_US // 1000000)     # 1280B @16k/mono/40ms
TALK_IDLE_S = float(os.environ.get("S10YK_TALK_IDLE", "20"))         # 无消费者多久自动收麦
TALK_MAX_S = float(os.environ.get("S10YK_TALK_MAX", "600"))          # 单次对讲上限
TALK_RETRY = int(os.environ.get("S10YK_TALK_RETRY", "3"))            # 麦被抢后重开次数
TALK_QMAX = 40                                                       # 下行队列上限（40×40ms=1.6s）
TALK = {"active": False, "proc": None, "ev": None, "start_ts": 0.0, "bytes": 0,
        "preempt": 0, "preempt_total": 0, "retry": 0, "err": "",
        "streams": 0, "stream_gen": 0, "last_read": 0.0, "why": "",
        "no_stream_since": 0.0}   # streams 归零起点，用于无消费者收麦（与 bytes 无关）
TALK_LOCK = threading.Lock()          # 会话态
TALK_COND = threading.Condition()     # 下行 PCM 队列
TALK_Q = deque()                      # 下行 PCM 帧（bytes）


def _talk_cmd():
    """业务板上的采集命令：stdbuf -o0 关掉 stdio 缓冲（否则 4KB 块缓冲白加 128ms 延迟）。

    OPS-91：默认走 mic_front.py（3ch 选路 + 降噪），输出仍是 mono raw；
    S10YK_TALK_FRONT=0 可回退到原 arecord -c 1 单路采集。
    """
    if TALK_FRONT:
        cmd = "exec stdbuf -o0 python3 %s" % TALK_FRONT_SCRIPT
        if not TALK_DENOISE:
            cmd += " --no-denoise"
        return cmd
    return ("exec stdbuf -o0 arecord -q -D %s -f S16_LE -r %d -c 1 -t raw "
            "-B %d -F %d -" % (TALK_DEV, TALK_RATE, TALK_BUF_US, TALK_PERIOD_US))


def _talk_spawn():
    cmd = ["ssh", "-i", SSH_KEY,
           "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no",
           "-o", "ConnectTimeout=4", "-o", "LogLevel=ERROR",
           "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
           SAY_HOST, _talk_cmd()]
    return subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            bufsize=0)


def _talk_kill(proc, grace=0.4):
    if proc is None:
        return
    for f in (proc.stdout, proc.stderr):
        try:
            if f:
                f.close()
        except Exception:
            pass
    try:
        proc.terminate()
        proc.wait(timeout=grace)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _talk_mark_preempt(msg):
    """记一次「MIC 被抢占」：会话内计数 + 累计计数 + 原因。"""
    with TALK_LOCK:
        TALK["preempt"] += 1
        TALK["preempt_total"] += 1
        TALK["err"] = (msg or "")[:160]
        total = TALK["preempt_total"]
        cur = TALK["preempt"]
    log("TALK mic preempted #%d (session %d): %s" % (total, cur, (msg or "")[:120]))


def _talk_reader(ev):
    """读 arecord → 下行队列；进程退出（麦被夺/USB 复位）计数并尝试重开。"""
    buf = b""
    while not ev.is_set():
        try:
            proc = _talk_spawn()
        except Exception as e:
            _talk_mark_preempt("spawn failed: %r" % (e,))
            if not _talk_backoff(ev):
                break
            continue
        with TALK_LOCK:
            TALK["proc"] = proc
        got_any = False
        while not ev.is_set():
            try:
                r, _, _ = select.select([proc.stdout], [], [], 0.5)
            except Exception:
                break
            if not r:
                continue
            try:
                data = os.read(proc.stdout.fileno(), 8192)
            except Exception:
                data = b""
            if not data:
                break
            buf += data
            while len(buf) >= TALK_FRAME:
                frame, buf = buf[:TALK_FRAME], buf[TALK_FRAME:]
                got_any = True
                with TALK_COND:
                    TALK_Q.append(frame)
                    while len(TALK_Q) > TALK_QMAX:
                        TALK_Q.popleft()      # 队列积压就丢最旧：宁可跳音也不能越说越慢
                    TALK_COND.notify_all()
                with TALK_LOCK:
                    TALK["bytes"] += len(frame)
                    TALK["last_read"] = time.time()
        # 采集进程退出：读 stderr 说明原因（Device or resource busy / EOF / USB 掉）
        err = b""
        try:
            proc.wait(timeout=1.0)
        except Exception:
            pass
        try:
            err = proc.stderr.read() or b""
        except Exception:
            pass
        _talk_kill(proc)
        with TALK_LOCK:
            if TALK["proc"] is proc:
                TALK["proc"] = None
        if ev.is_set():
            break
        msg = err.decode("utf-8", "ignore").strip().splitlines()
        msg = msg[-1] if msg else ("arecord EOF" if got_any else "arecord exited")
        _talk_mark_preempt(msg)
        with TALK_LOCK:
            retry = TALK["retry"]
            TALK["retry"] = retry + 1
        if retry >= TALK_RETRY:
            log("TALK mic lost -> give up after %d retries" % retry)
            _talk_finish("mic-lost")
            break
        log("TALK mic reopen %d/%d" % (retry + 1, TALK_RETRY))
        if not _talk_backoff(ev):
            break
    with TALK_LOCK:
        mine = (TALK["ev"] is ev and TALK["active"])
    if mine:                           # 先松锁再收尾（Lock 不可重入）
        _talk_finish("reader-exit")


def _talk_backoff(ev, sec=0.3):
    """抢占后短暂等待再重开；返回 False 表示会话已停。"""
    end = time.time() + sec
    while time.time() < end:
        if ev.is_set():
            return False
        time.sleep(0.05)
    return not ev.is_set()


def _talk_finish(why=""):
    """收尾：停采集进程、清队列、置空闲（幂等）。"""
    with TALK_LOCK:
        ev = TALK["ev"]
        proc = TALK["proc"]
        TALK["proc"] = None
        TALK["active"] = False
        TALK["stream_gen"] += 1          # 让正在推流的 HTTP 处理器退出
        TALK["why"] = why
    if ev is not None:
        ev.set()
    _talk_kill(proc)
    with TALK_COND:
        TALK_Q.clear()
        TALK_COND.notify_all()
    log("TALK stop (%s)" % (why or "-"))


def _talk_watchdog(ev):
    """三项兜底：无消费者超时收麦 / 采集卡死（5s 无数据）重开 / 超过单次上限。"""
    last_bytes, last_ts = -1, time.time()
    while not ev.is_set():
        time.sleep(0.5)
        if ev.is_set():
            break
        now = time.time()
        with TALK_LOCK:
            if not TALK["active"]:
                break
            start = TALK["start_ts"]
            streams = TALK["streams"]
            nbytes = TALK["bytes"]
            last_read = TALK["last_read"]
            proc = TALK["proc"]
        if nbytes != last_bytes:
            last_bytes, last_ts = nbytes, now
        if now - start > TALK_MAX_S:
            _talk_finish("max-timeout")
            break
        # 无消费者收麦：按「连续无 streams」计时，不能按 bytes 变化——
        # 前端 fetch 失败时 streams=0 但采集线程仍在写队列，旧逻辑下麦会被无限占用
        # （2026-11-26 事故：卡了 279s）。start 后到 talk/pcm 连上有约 1.5s 空窗，
        # TALK_IDLE_S=20 足够覆盖，不会误伤正常对讲。
        if streams == 0:
            with TALK_LOCK:
                if TALK["no_stream_since"] == 0.0:
                    TALK["no_stream_since"] = now
                ns_start = TALK["no_stream_since"]
        else:
            with TALK_LOCK:
                TALK["no_stream_since"] = 0.0
            ns_start = 0.0
        if streams == 0 and now - ns_start > TALK_IDLE_S:
            _talk_finish("no-listener")      # 页面关了/断网/前端 fetch 失败，别一直占着麦
            break
        if proc is not None and now - last_read > 5.0:
            log("TALK mic stalled 5s -> recycle")
            _talk_kill(proc)                 # 读线程会看到 EOF 并计数重开


def talk_start():
    """开对讲会话（幂等）；立即返回，不等首帧（v3.5.6）。

    2026-11-26 事故：原逻辑最多等 3s 首帧，公网 HTTPS 链路在等待期间断开连接，
    前端 fetch 抛 "Failed to fetch"（"无法开始：..."），而后端其实已成功启动采集。
    首帧改由 /api/talk/pcm 流自然承载；麦克风真打不开时 _talk_reader 会置
    active=False，前端 status 轮询里 if(!s.talk.active) talkStop() 负责兜底。
    """
    with TALK_LOCK:
        already = TALK["active"]
    if already:                        # 注意：先松锁再取状态，Lock 不可重入
        st = talk_state()
        st["ok"] = True
        return st
    try:
        say_stop()                       # 与自定义播报互斥（喇叭只有一套）
    except Exception:
        pass
    try:
        BRIDGE.audio_stop(0)             # 停 DDS 预置音效
    except Exception:
        pass
    ev = threading.Event()
    with TALK_LOCK:
        TALK.update({"active": True, "ev": ev, "proc": None, "start_ts": time.time(),
                     "bytes": 0, "preempt": 0, "retry": 0, "err": "",
                     "last_read": time.time(), "why": "", "no_stream_since": 0.0})
    with TALK_COND:
        TALK_Q.clear()
    threading.Thread(target=_talk_reader, args=(ev,), daemon=True).start()
    threading.Thread(target=_talk_watchdog, args=(ev,), daemon=True).start()
    log("TALK start dev=%s %dHz frame=%dB" % (TALK_DEV, TALK_RATE, TALK_FRAME))
    # 不等待首帧：立即返回，避免公网 HTTPS 长响应被中断（见 docstring）
    st = talk_state()
    st["ok"] = bool(st["active"])
    return st


def talk_stop(reason="client"):
    with TALK_LOCK:
        active = TALK["active"]
    if not active:
        return False
    _talk_finish(reason)
    return True


def talk_active():
    with TALK_LOCK:
        return bool(TALK["active"])


def talk_read(timeout=0.5):
    """从下行队列取一帧；无数据返回 None。"""
    with TALK_COND:
        if not TALK_Q:
            TALK_COND.wait(timeout)
        if TALK_Q:
            return TALK_Q.popleft()
    return None


def talk_stream_begin():
    with TALK_LOCK:
        TALK["streams"] += 1
        TALK["stream_gen"] += 1
        return TALK["stream_gen"]


def talk_stream_end():
    with TALK_LOCK:
        TALK["streams"] = max(0, TALK["streams"] - 1)


def talk_state():
    with TALK_LOCK:
        active = bool(TALK["active"])
        return {"active": active,
                "seconds": round(time.time() - TALK["start_ts"], 1) if active else 0,
                "bytes": int(TALK["bytes"]),
                "preempt": int(TALK["preempt"]),            # 本次会话被抢次数
                "preempt_total": int(TALK["preempt_total"]),  # 后端启动以来累计
                "err": TALK["err"], "why": TALK["why"],
                "streams": int(TALK["streams"]),
                "rate": TALK_RATE, "frame": TALK_FRAME,
                "front": bool(TALK_FRONT), "denoise": bool(TALK_FRONT and TALK_DENOISE)}


def say_wav(filename, loop=False, gap=None):
    """经 103 上的 sayd 播放自定义 wav（TCP 推流，单实例、播完即释放声卡）

    - 每次播放只开一次设备，结束后（含用户点停止）立刻关闭，不留残留进程；
    - loop=True 由本端驱动循环，gap>1.3s 时轮次间会真正释放设备，
      给厂商提示音留出可插入的窗口（实测厂商播报 20ms 内即放弃，必须设备空闲）。
    """
    name = os.path.basename(str(filename))
    path = os.path.join(ASSET_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    info = _wav_info(path)
    if info["bits"] != 16:
        log("SAY warning: %s is %d-bit, sayd 以 S16_LE 播放" % (name, info["bits"]))
    gap = SAY_LOOP_GAP if gap is None else float(gap)
    # D-040 幂等去重：1s 内同文件同模式重复下发直接忽略（前端双击/多端同秒各发一次/
    # 网络抖动重试都会在这里被挡掉，从源头杜绝双喂流叠音）
    dedup_key = name + (":loop" if loop else "")
    now = time.time()
    dup = False
    with LOCK:
        if SAY_LAST_PLAY["key"] == dedup_key and now - SAY_LAST_PLAY["ts"] < 1.0:
            dup = True
    if dup:
        # D-040 幂等去重命中。注意 log() 内部也要取 LOCK（不可重入），
        # 绝不能在 with LOCK 块内调用 log()，否则自死锁（2026-11-20 实测）。
        log("SAY dedupe: %s within %.2fs -> ignore duplicate start" %
            (dedup_key, now - SAY_LAST_PLAY["ts"]))
        return {"mode": "sayd", "duration_ms": info["cycle_ms"],
                "loop": bool(loop), "dedup": True}
    shout_stop()   # 互斥：喊话会话必须先停（sayd 全局单例 PLAYER）
    talk_stop("say-entry")   # 互斥：双向对讲也走 sayd + 占麦，一起收掉
    say_stop()

    try:
        sock = socket.create_connection((SAYD_HOST, SAYD_PORT), timeout=3.0)
    except Exception as e:
        log("SAY connect %s:%d failed (%r) -> restart" % (SAYD_HOST, SAYD_PORT, e))
        _sayd_restart()
        time.sleep(1.0)
        sock = socket.create_connection((SAYD_HOST, SAYD_PORT), timeout=3.0)
    sock.settimeout(8.0)
    try:
        # 收窄发送缓冲，避免整段音频被内核一次性吞掉（否则停止指令要等缓冲播完才生效）
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
    except Exception:
        pass

    ev = threading.Event()
    fe = {"ev": ev, "sock": sock, "name": name, "ts": time.time()}   # D-040：注册表条目
    start_payload = json.dumps({"rate": info["rate"], "channels": info["channels"]}).encode()
    _say_frame(sock, F_START, start_payload)
    reply = _say_reply(sock)
    if not reply.startswith("OK"):
        try:
            sock.close()
        except Exception:
            pass
        raise RuntimeError("sayd busy: %s" % (reply or "no reply"))

    with LOCK:
        SAY_FEEDERS.append(fe)
        SAY.update({"sock": sock, "stop": ev, "name": name,
                    "ts": time.time(), "active": True})
        SAY_LAST_PLAY["key"] = dedup_key
        SAY_LAST_PLAY["ts"] = time.time()
        nfeeder = len(SAY_FEEDERS)
    if nfeeder > 1:
        log("SAY warning: %d live feeder(s) before start (should be 0)" % (nfeeder - 1))
    log("SAY %s loop=%s %dms gap=%.1fs (sayd)" % (name, bool(loop), info["cycle_ms"], gap))

    def _connect():
        s = socket.create_connection((SAYD_HOST, SAYD_PORT), timeout=3.0)
        s.settimeout(8.0)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        except Exception:
            pass
        _say_frame(s, F_START, start_payload)
        reply = _say_reply(s)
        if not reply.startswith("OK"):
            try:
                s.close()
            except Exception:
                pass
            raise RuntimeError("sayd busy: %s" % (reply or "no reply"))
        return s

    def _feeder():
        pcm, n = info["pcm"], len(info["pcm"])
        bps = max(1, info["byte_rate"])
        cur = sock
        try:
            while True:
                try:
                    t_cycle = time.time()
                    sent = 0
                    for off in range(0, n, SAY_CHUNK):
                        if ev.is_set():
                            return
                        data = pcm[off:off + SAY_CHUNK]
                        _say_frame(cur, F_PCM, data)
                        sent += len(data)
                        # 按实时速率节流：既保证 100ms 分片不饿死 aplay，也让"停止"随时生效
                        delay = t_cycle + sent / float(bps) - time.time()
                        if delay > 0:
                            time.sleep(min(delay, 0.2))
                    if not loop or ev.is_set():
                        return
                    if gap > 0:
                        time.sleep(gap)          # 让路窗口：sayd 空闲 >1.2s 会关设备
                        if ev.is_set():
                            return
                        _say_frame(cur, F_START, start_payload)
                        if not _say_reply(cur).startswith("OK"):
                            log("SAY loop reopen refused -> stop")
                            return
                except Exception as e:
                    if ev.is_set() or not loop:
                        if not ev.is_set():
                            log("SAY feed error: %r" % e)
                        return
                    # v2.7.1 自愈：循环播报中断线自动重连（对抗 WiFi 抖动/瞬断），
                    # 最多 6 次 × 2s ≈ 覆盖 30s 内的链路抖动；重连后从头继续循环
                    log("SAY feed error: %r -> auto-reconnect" % e)
                    try:
                        cur.close()
                    except Exception:
                        pass
                    ok = False
                    for attempt in range(1, 7):
                        if ev.is_set():
                            return
                        time.sleep(2.0)
                        try:
                            cur = _connect()
                            log("SAY feed reconnected (attempt %d)" % attempt)
                            ok = True
                            break
                        except Exception as e2:
                            log("SAY reconnect attempt %d failed: %r" % (attempt, e2))
                    if not ok:
                        log("SAY reconnect gave up after 6 attempts")
                        return
                    with LOCK:
                        fe["sock"] = cur               # D-040：注册表同步
                        if SAY.get("stop") is ev:
                            SAY["sock"] = cur
        finally:
            with LOCK:
                if fe in SAY_FEEDERS:
                    SAY_FEEDERS.remove(fe)             # D-040：退出注销
                if SAY.get("stop") is ev:
                    SAY["sock"] = None
                    SAY["stop"] = None
                    SAY["active"] = False
            try:
                cur.close()
            except Exception:
                pass

    threading.Thread(target=_feeder, daemon=True).start()
    return {"mode": "sayd", "duration_ms": info["cycle_ms"], "loop": bool(loop)}


# ---------- v2.6 摄像头预览（RTSP → MJPEG 低时延流） ----------
# 厂商相机节点输出 4 路 RTSP（103:8554，H.264 1024x576@30 2048kbps，无音轨）；
# DDS 图像口默认关闭且需重启厂商容器，不动它。这里用 ffmpeg 拉流转 JPEG，
# 单进程帧总线供多客户端共享；无客户端 20s 后自动停，不常年吃 CPU。
# 时延要点：输入侧 nobuffer/low_delay/max_delay=0 去掉 ffmpeg 的探测与抖动缓冲，
# 输出 fps=15 控制每帧间隔，链路时延 ≈ 源编码(约100ms) + 1/fps + 转码。
RTSP_BASE = os.environ.get("S10YK_RTSP_BASE", "rtsp://10.21.33.103:8554")
CAM_SRCS = ("video0", "video2", "video4", "video6")
CAM_FPS = 15
CAM_WIDTH = 960
CAM_IDLE_STOP = 20.0


def _extract_jpeg(buf):
    """从拼接的 MJPEG 字节流里切出一帧完整 JPEG，返回 (frame, rest)；不完整返回 (None, 待续字节)。"""
    i = buf.find(b"\xff\xd8")
    if i < 0:
        return None, b""                     # 连 SOI 都没有，前面的残包直接丢
    n = len(buf)
    j = i + 2
    while True:
        if j + 1 >= n:
            return None, buf[i:]
        if buf[j] != 0xFF:                   # 损坏处重新同步到下一个 FF
            k = buf.find(b"\xff", j)
            if k < 0 or k + 1 >= n:
                return None, buf[i:]
            j = k
        m = buf[j + 1]
        if m == 0xFF:                        # 填充字节
            j += 1
            continue
        if m == 0xD9:                        # EOI：一帧结束
            return buf[i:j + 2], buf[j + 2:]
        if m == 0x01 or 0xD0 <= m <= 0xD7:   # 无长度段（RSTn/TEM）
            j += 2
            continue
        if j + 4 > n:
            return None, buf[i:]
        seg = (buf[j + 2] << 8) | buf[j + 3]
        if seg < 2:
            j += 2
            continue
        if j + 2 + seg > n:
            return None, buf[i:]
        if m == 0xDA:                        # SOS：扫熵编码区，直到下一个真 marker
            j += 2 + seg
            while True:
                k = buf.find(b"\xff", j)
                if k < 0 or k + 1 >= n:
                    return None, buf[i:]
                m2 = buf[k + 1]
                if m2 == 0x00 or 0xD0 <= m2 <= 0xD7:
                    j = k + 2
                    continue
                j = k
                break
        else:
            j += 2 + seg


class CamSource(object):
    """一路 RTSP 源：单 ffmpeg 进程 → 帧总线，多客户端共享，空闲自动停。"""

    def __init__(self, src):
        self.src = src
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frame = b""
        self.seq = 0
        self.clients = 0
        self.last_used = 0.0
        self.proc = None

    def _cmd(self):
        return ["ffmpeg", "-hide_banner", "-loglevel", "error",
                "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
                "-avioflags", "direct", "-probesize", "500000",
                "-analyzeduration", "2000000", "-max_delay", "0",
                "-i", "%s/%s" % (RTSP_BASE, self.src),
                "-vf", "fps=%d,scale=%d:-2" % (CAM_FPS, CAM_WIDTH),
                "-q:v", "4", "-an", "-f", "mjpeg", "pipe:1"]

    def _reader(self, proc):
        buf = b""
        try:
            while proc.poll() is None:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    f, rest = _extract_jpeg(buf)
                    if f is None:
                        buf = rest
                        break
                    buf = rest
                    with self.lock:
                        self.frame = f
                        self.seq += 1
                        self.cond.notify_all()
        except Exception:
            pass
        finally:
            with self.lock:
                self.frame = b""
                self.seq += 1
                self.cond.notify_all()

    def acquire(self):
        with self.lock:
            self.clients += 1
            self.last_used = time.time()
            if self.proc is None:
                try:
                    self.proc = subprocess.Popen(self._cmd(), stdout=subprocess.PIPE,
                                                 stderr=subprocess.DEVNULL)
                    threading.Thread(target=self._reader, args=(self.proc,),
                                     daemon=True).start()
                    log("CAM %s: ffmpeg start" % self.src)
                except Exception:
                    self.clients -= 1
                    raise

    def release(self):
        with self.lock:
            self.clients = max(0, self.clients - 1)
            self.last_used = time.time()

    def latest(self, last_seq, timeout):
        """等新帧；last_seq=None 表示等首帧。返回 (frame, seq)，断流时 frame 为空。"""
        end = time.time() + timeout
        with self.cond:
            while True:
                if self.frame and (last_seq is None or self.seq != last_seq):
                    return self.frame, self.seq
                if time.time() >= end or self.proc is None:
                    return b"", self.seq
                self.cond.wait(0.5)

    def stop_if_idle(self):
        with self.lock:
            if self.proc is not None and self.clients <= 0 \
                    and time.time() - self.last_used > CAM_IDLE_STOP:
                self._shutdown()

    def _shutdown(self):
        p, self.proc = self.proc, None
        self.frame = b""
        if p:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
            log("CAM %s: idle stop" % self.src)


CAMS = dict((s, CamSource(s)) for s in CAM_SRCS)


def cam_reaper():
    while True:
        time.sleep(5)
        for c in CAMS.values():
            try:
                c.stop_if_idle()
            except Exception:
                pass


def person_alert_state():
    """REQ-047：读检测服务的 latest.json，返回最近一次人体预警（无则 None）

    供 /api/status 下发前端横幅，也供语音守护线程判断"是否是新预警"。
    """
    try:
        with open(PERSON_LATEST, "r", encoding="utf-8") as f:
            d = json.load(f)
        ts = float(d.get("ts") or 0)
        age = time.time() - ts
        return {"ts": ts, "age": round(age, 1), "fresh": age <= PERSON_ALERT_TTL,
                "cam_id": d.get("cam_id"), "cam_name": d.get("cam_name"),
                "conf": d.get("conf"), "n": d.get("n"), "img": d.get("img"),
                "voice_key": d.get("voice_key")}
    except Exception:
        return None


def person_voice_watcher():
    """REQ-047：新人体预警 → 语音播报（复用 say_wav / sayd 通道）

    - 每条预警最多播一次（按 ts 去重）；
    - 全局最小间隔 PERSON_VOICE_GAP，防 4 路轮流命中时刷屏；
    - 播报失败不重试（避免卡死在异常 WAV 上），只记日志。
    """
    while True:
        try:
            st = person_alert_state()
            if st and st["ts"] > PERSON["spoken_ts"]:
                if time.time() - PERSON["last_speak"] >= PERSON_VOICE_GAP:
                    name = "person_alert_%s.wav" % (st.get("voice_key") or "front")
                    try:
                        say_wav(name)          # 阻塞至播完
                        PERSON["last_speak"] = time.time()
                        log("PERSON 播报 %s (%s %.2f)" %
                            (name, st.get("cam_name"), st.get("conf") or 0))
                    except Exception as e:
                        log("PERSON 播报失败 %s: %r" % (name, e))
                    PERSON["spoken_ts"] = st["ts"]
                else:
                    PERSON["spoken_ts"] = st["ts"]   # 间隔内静默跳过，仍标记已处理
        except Exception as e:
            log("person watcher error: %r" % e)
        time.sleep(2.0)


class Handler(BaseHTTPRequestHandler):
    server_version = "S10YK/1.4"
    # REQ-039：实时对讲每秒约 10 个 PCM 小包，必须复用连接——
    # HTTP/1.0 每条请求都要重新 TCP+TLS 握手，公网/无线下会直接卡成断续音。
    protocol_version = "HTTP/1.1"
    # 空闲连接 65s 后回收（避免浏览器常驻连接把线程挂死）；小响应禁用 Nagle 降延迟。
    timeout = 65
    disable_nagle_algorithm = True

    def setup(self):
        # v3.5.9：TLS 握手在 handler 线程执行（get_request 用 do_handshake_on_connect=False
        # 包装、已设 15s 超时）。公网扫描器/坏客户端握手失败或超时只废本线程，
        # 绝不阻塞 accept 线程（16:10 事故：握手挂死 accept，队列堆 111 个连接全站超时）。
        # 注意用 self.request：self.connection 要到 super().setup() 里才赋值。
        conn = self.request
        if isinstance(conn, ssl.SSLSocket) and getattr(conn, "_sslobj", None) is None:
            conn.do_handshake()
        super().setup()

    def _send(self, code, obj, ctype="application/json; charset=utf-8", extra=None, meter=True):
        body = obj if isinstance(obj, bytes) else json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
            if meter:
                net_add(len(body))   # REQ-029：真实业务响应计入出向流量
        except Exception:
            pass

    # ---- 鉴权（v2.7）：Cookie 或 Authorization: Bearer 均可 ----
    def _token_from_req(self):
        auth = self.headers.get("Authorization", "") or ""
        if auth.lower().startswith("bearer "):
            return auth[7:].strip()
        ck = self.headers.get("Cookie", "") or ""
        for part in ck.split(";"):
            kv = part.strip().split("=", 1)
            if len(kv) == 2 and kv[0] == AUTH_COOKIE:
                return kv[1].strip()
        return ""

    def _auth_ok(self):
        if not AUTH_TOKEN:          # 未配置 token 时保持旧行为（本地/兜底部署）
            return True
        got = self._token_from_req()
        return bool(got) and hmac.compare_digest(got, AUTH_TOKEN)

    def _deny(self):
        self._send(401, {"error": "unauthorized", "auth": True})

    def _meter_request(self):
        """REQ-029：统计前端→后端的上行请求字节（请求行 + 请求头 + 请求体），
        纳入 net.bps 双向流量窗口。覆盖控制/音频/状态/video 等全部 HTTP 交互。"""
        try:
            # 请求行：METHOD path HTTP/x.y
            n = len(self.requestline.encode("utf-8", "replace"))
            # 请求头：每行 "K: V\r\n"
            for k, v in self.headers.items():
                n += len(k) + len(v) + 4
            # 请求体长度
            n += int(self.headers.get("Content-Length", 0) or 0)
            net_add(n)
        except Exception:
            pass

    def _json_body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
        except Exception:
            return {}

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        global ARMED
        self._meter_request()   # REQ-029：计入上行请求字节（全部交互）
        path = self.path.split("?")[0]
        # v2.7 鉴权：页面本身放行（登录层在前端），其余 /api/* 必须带 token
        if path not in AUTH_OPEN and not self._auth_ok():
            self._deny()
            return
        if path == "/api/auth":
            self._send(200, {"authed": True, "required": bool(AUTH_TOKEN)})
            return
        if path in ("/", "/index.html"):
            try:
                with open(FRONTEND, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except Exception as e:
                self._send(500, {"error": str(e)})
        elif path == "/shout-processor.js":
            # 单向喊话 AudioWorklet 脚本，与前端同目录
            try:
                js_path = os.path.join(os.path.dirname(FRONTEND), "shout-processor.js")
                with open(js_path, "rb") as f:
                    self._send(200, f.read(), "application/javascript; charset=utf-8")
            except Exception as e:
                self._send(404, {"error": str(e)})
        elif path == "/api/bw":
            # REQ-029：旧测速块保留兼容，但 meter=False 不计入真实流量统计
            self._send(200, BW_BLOCK, "application/octet-stream", meter=False)
        elif path == "/api/status":
            with LOCK:
                mi = dict(MOTION_INFO)
                au = dict(AUDIO_STATUS)
                bat = dict(BATTERY_INFO)
                armed = ARMED
                cmd = dict(NAV_CMD)
            with LOCK:
                vol = VOLUME["pct"]
            mi2 = dict(mi)
            mi2.update(sleep_detect(mi))   # REQ-042：休眠状态（确认 0x1001 / 请求标志 / 疑似厂商休眠）
            self._send(200, {"motion": mi2, "audio": au, "battery": bat, "armed": armed,
                             "nav_cmd": cmd, "limits": NAV_LIMIT,
                             "steer": {"gear_x": list(STEER_GEAR_X),
                                       "gear_yaw": list(STEER_GEAR_YAW),
                                       "gear_idx": GEAR_IDX,
                                       "gear_name": GEAR_NAMES[GEAR_IDX],
                                       "gear_speed_mps": (0.3, 0.8, 1.3)[GEAR_IDX],
                                       "deadzone": STEER_DEADZONE,
                                       "zero_hold_s": STEER_ZERO_HOLD_S,
                                       "y_supported": False},
                             "log": list(LOG[-30:]),
                             "volume": vol, "volume_target": VOLUME_TARGET,
                             "net": {"bps": net_bps()},   # REQ-029：前后端双向交互流量速率（bit/s）
                             "shout": shout_state(),      # 单向喊话会话状态
                             "talk": talk_state(),        # 双向对讲会话状态（含 MIC 抢占计数）
                             "person": person_alert_state(),   # REQ-047：最近一次人体预警
                             "now": time.time()})  # v2.3: 服务器时钟，供前端计算数据停留
        elif path == "/api/person/stats":
            # REQ-048：人员检测页 1Hz 轮询——统计信息（总人数/当前人数/存储配额）+ 最新抓拍
            st = {}
            try:
                with open(os.path.join(PERSON_DIR, "stats.json"), "r", encoding="utf-8") as f:
                    st = json.load(f)
            except Exception:
                pass
            if "switches" not in st:   # REQ-049：检测服务未运行时缺省全开
                st["switches"] = {c: True for c, _ in PERSON_CAMS}
            self._send(200, {"stats": st, "alert": person_alert_state()})
        elif path == "/api/person/latest":
            # REQ-047：最近一次人体预警（前端轮询用，比 /api/status 轻）
            self._send(200, {"alert": person_alert_state()})
        elif path == "/api/person/list":
            # REQ-047：最近 N 条预警记录（读 history.jsonl 尾部）
            n = 20
            qs = self.path.split("?", 1)[1] if "?" in self.path else ""
            for kv in qs.split("&"):
                if kv.startswith("n="):
                    try:
                        n = max(1, min(100, int(kv[2:])))
                    except ValueError:
                        pass
            items = []
            try:
                with open(os.path.join(PERSON_DIR, "history.jsonl"), "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            items.append(json.loads(line))
                        except Exception:
                            pass
            except Exception:
                pass
            self._send(200, {"items": items[-n:][::-1]})
        elif path == "/api/person/img":
            # REQ-047：下发抓拍图（仅允许 PERSON_DIR 内的相对路径，防目录穿越）
            qs = self.path.split("?", 1)[1] if "?" in self.path else ""
            name = ""
            for kv in qs.split("&"):
                if kv.startswith("name="):
                    name = kv[5:]
            name = name.replace("%2F", "/").replace("%2f", "/")
            fp = os.path.realpath(os.path.join(PERSON_DIR, name))
            root = os.path.realpath(PERSON_DIR)
            if (not name) or (not fp.startswith(root + os.sep)) or (not os.path.isfile(fp)):
                self._send(404, {"error": "not found"})
                return
            try:
                with open(fp, "rb") as f:
                    self._send(200, f.read(), "image/jpeg")
            except Exception as e:
                self._send(500, {"error": str(e)})
        elif path == "/api/shout/state":
            self._send(200, shout_state())
        elif path == "/api/talk/state":
            self._send(200, talk_state())
        elif path == "/api/talk/pcm":
            # 下行：机器人 MIC → 浏览器。v3.5.7 改为短请求轮询：
            # 原 chunked 长流在公网 HTTPS 链路上会被中途掐断（实测 20s 只传出 2.7s PCM，
            # 手机侧"下行中断/无声"），而喊话的短 POST 稳定——公网链路对长连接不友好。
            # 现每次请求阻塞凑满 ms 毫秒（默认 200，可传 40~500）即以 Content-Length
            # 立即返回；无数据返回空体，前端串行轮询。语义与 /api/shout/pcm 同款。
            if not talk_active():
                self._send(409, {"error": "talk not active"})
                return
            ms = 200
            try:
                qs = self.path.split("?", 1)[1] if "?" in self.path else ""
                for kv in qs.split("&"):
                    if kv.startswith("ms="):
                        ms = min(max(int(kv[3:]), 40), 500)
            except Exception:
                ms = 200
            gen = talk_stream_begin()
            try:
                deadline = time.time() + ms / 1000.0
                want = TALK_RATE * 2 * ms // 1000     # ms 毫秒 PCM 的字节数
                buf = bytearray()
                while len(buf) < want and time.time() < deadline:
                    data = talk_read(max(0.01, deadline - time.time()))
                    if data:
                        buf.extend(data)
                # 不发 Connection: close：HTTP/1.1 + Content-Length 下浏览器
                # 可复用 TLS 连接，轮询省去每次握手（实测握手开销吃掉 1/3 吞吐）
                self._send(200, bytes(buf), ctype="application/octet-stream")
            except Exception:
                pass                                  # 客户端断开/会话被停，均属正常
            finally:
                talk_stream_end()
        elif path == "/api/audio/presets":
            self._send(200, load_presets())
        elif path == "/api/cam/probe":
            self._send(200, {"sources": [{"src": s, "clients": CAMS[s].clients,
                                          "running": CAMS[s].proc is not None}
                                         for s in CAM_SRCS],
                             "fps": CAM_FPS, "width": CAM_WIDTH})
        elif path == "/api/cam/stream":
            qs = self.path.split("?", 1)[1] if "?" in self.path else ""
            src = CAM_SRCS[0]
            for kv in qs.split("&"):
                if kv.startswith("src="):
                    src = kv[4:]
            if src not in CAMS:
                self._send(404, {"error": "bad src"})
                return
            cam = CAMS[src]
            try:
                cam.acquire()
            except Exception as e:
                self._send(500, {"error": "ffmpeg start failed: %r" % e})
                return
            try:
                first = cam.latest(None, 8.0)[0]     # 最多等 8s 出首帧（源断开则报错）
                if not first:
                    self._send(502, {"error": "camera stream unavailable"})
                    return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                seq = None
                while True:
                    f, seq2 = cam.latest(seq, 6.0)
                    if not f:
                        break
                    seq = seq2
                    # D-030 修复：multipart/x-mixed-replace 要求【每帧前】都有 boundary。
                    # 原实现只在流开头发一次 --frame，后续帧没有分隔符 → 浏览器解析完
                    # 第一帧后一直等下一个 boundary，画面永久卡住（curl 不解析故看似正常）。
                    head = (b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: %d\r\n\r\n" % len(f))
                    self.wfile.write(head)
                    self.wfile.write(f)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    net_add(len(head) + len(f) + 2)   # REQ-029：摄像头帧计入真实出向流量
            except Exception:
                pass                                  # 客户端断开属正常退出
            finally:
                cam.release()
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        global ARMED, NAV_CMD, NAV_LAST_TS, GEAR_IDX, SLEEP_REQ, SLEEP_REQ_TS
        self._meter_request()   # REQ-029：计入上行请求字节（控制/音频等指令）
        path = self.path.split("?")[0]
        # 喊话音频上传是二进制 body（WAV / PCM），不走 JSON 解析
        if path in ("/api/shout/clip", "/api/shout/pcm"):
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(n) if n else b""
            except Exception:
                body = b""
        else:
            body = self._json_body()
        # v2.7 登录/登出（login 自身放行，其余动作必须已鉴权）
        if path == "/api/login":
            if not AUTH_TOKEN:
                self._send(200, {"ok": True, "required": False})
                return
            tk = str(body.get("token", "")).strip()
            if tk and hmac.compare_digest(tk, AUTH_TOKEN):
                self._send(200, {"ok": True, "required": True}, extra={
                    "Set-Cookie": "%s=%s; Path=/; Max-Age=%d; HttpOnly; SameSite=Strict"
                                  % (AUTH_COOKIE, AUTH_TOKEN, AUTH_TTL)})
            else:
                time.sleep(0.4)          # 抑制暴力枚举
                self._send(401, {"ok": False, "error": "bad token"})
            return
        if path == "/api/logout":
            self._send(200, {"ok": True}, extra={
                "Set-Cookie": "%s=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict" % AUTH_COOKIE})
            return
        if not self._auth_ok():
            self._deny()
            return
        try:
            if path == "/api/gear":
                # REQ-033：前端档位按钮设置低/中/高（0/1/2），默认中速
                idx = int(body.get("idx", 1))
                if idx not in (0, 1, 2):
                    self._send(400, {"error": "invalid gear index"})
                    return
                GEAR_IDX = idx
                log("GEAR -> %s (x=%.2f yaw=%.2f)" % (GEAR_NAMES[idx], STEER_GEAR_X[idx], STEER_GEAR_YAW[idx]))
                self._send(200, {"ok": True, "idx": idx, "name": GEAR_NAMES[idx],
                                 "speed_mps": 0.3 if idx == 0 else (0.8 if idx == 1 else 1.3)})
            elif path == "/api/arm":
                ARMED = bool(body.get("armed"))
                if not ARMED:
                    with LOCK:
                        NAV_CMD = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                log("ARM -> %s" % ARMED)
                self._send(200, {"armed": ARMED})
            elif path == "/api/motion/state":
                s = int(body["state"])
                if s == 1:
                    # 站立 = 起立(1) + 自动补 RL(17)，否则只能站起来一半（半蹲）
                    threading.Thread(target=stand_sequence, daemon=True).start()
                    self._send(200, {"ok": True, "seq": "stand(1)->rl(17)"})
                else:
                    BRIDGE.publish_state(s)
                    self._send(200, {"ok": True})
            elif path == "/api/motion/rl":
                BRIDGE.publish_state(17)
                self._send(200, {"ok": True})
            elif path == "/api/motion/sleep":
                # REQ-042：休眠/唤醒切换（D-039 后新增）
                # 休眠 = 序列 4(趴下)→贴地→2(关节阻尼/下电)，物理等效厂商休眠
                #         （指南无独立休眠指令；0x1001 仅 ASDU 表有、话题实测无效）
                # 唤醒 = 站立序列 1→(等高稳定)→17→自动挂步态（与「站立」按钮同路径）
                sl = bool(body.get("sleep"))
                SLEEP_REQ = sl
                SLEEP_REQ_TS = time.time()
                if sl:
                    # 进休眠前先撤速度使能并清零速度，避免唤醒后残留行走指令
                    ARMED = False
                    with LOCK:
                        NAV_CMD.update({"x": 0.0, "y": 0.0, "yaw": 0.0})
                    threading.Thread(target=sleep_sequence, daemon=True).start()
                    log("SLEEP -> sleep_sequence(4->2)")
                    self._send(200, {"ok": True, "sleep": True})
                else:
                    threading.Thread(target=stand_sequence, daemon=True).start()
                    log("WAKE -> stand_sequence(1->17)")
                    self._send(200, {"ok": True, "sleep": False})
            elif path == "/api/gait":
                BRIDGE.publish_gait(int(body["gait"]))
                self._send(200, {"ok": True})
            elif path == "/api/nav":
                if not ARMED:
                    self._send(403, {"error": "not armed"})
                    return
                # 前端给 [-1,1]，后端量化成三档（STEER 无法精细调速，见标定报告第八节）
                stick_x = max(-1.0, min(1.0, float(body.get("x", 0))))
                stick_y = max(-1.0, min(1.0, float(body.get("y", 0))))
                stick_w = max(-1.0, min(1.0, float(body.get("yaw", 0))))
                gx, gx_name = steer_gear(stick_x, STEER_GEAR_X)
                gw, gw_name = steer_gear(stick_w, STEER_GEAR_YAW)
                with LOCK:
                    NAV_CMD = {"x": gx, "y": 0.0, "yaw": gw}
                    NAV_LAST_TS = time.time()
                self._send(200, {"ok": True,
                                 "gear": {"fwd": gx_name, "turn": gw_name, "idx": GEAR_IDX},
                                 "steer": {"x": gx, "yaw": gw},
                                 "y_ignored": abs(stick_y) > 0.05})
            elif path == "/api/audio/play":
                loop = bool(body.get("loop", False))
                aid = int(body.get("id", 0))
                # 三路互斥：喊话 / 循环音效 / 自定义语音共享同一声卡，先停后放避免叠音
                shout_stop()
                talk_stop("audio-play")
                say_stop()
                if aid >= 1000 and body.get("file"):
                    BRIDGE.audio_stop(0)
                    info = say_wav(body["file"], loop=loop)
                    self._send(200, {"ok": True, "mode": "sayd", "loop": loop,
                                     "duration_ms": info["duration_ms"]})
                else:
                    # repeat=0 才是无限循环
                    repeat = int(body.get("repeat", 0 if loop else 1))
                    dur = 0 if loop else PRESET_DUR.get(aid, 0)
                    ok = play_preset(aid, priority=int(body.get("priority", 5)),
                                     repeat=repeat,
                                     interval_ms=int(body.get("interval_ms", 0)))
                    self._send(200, {"ok": True, "mode": "dds", "loop": loop,
                                     "confirm": ok, "repeat": repeat,
                                     "duration_ms": dur})
            elif path == "/api/audio/say":
                loop = bool(body.get("loop", False))
                shout_stop()          # 互斥：停喊话
                talk_stop("audio-say")  # 互斥：停双向对讲（占麦 + 走 sayd）
                BRIDGE.audio_stop(0)  # 互斥：停掉预置音效（含循环）再播自定义语音
                info = say_wav(body.get("file") or "keepdistance.wav", loop=loop)
                self._send(200, {"ok": True, "mode": "sayd", "loop": loop,
                                 "duration_ms": info["duration_ms"]})
            elif path == "/api/audio/stop":
                shout_stop()          # 停止播报也要停喊话
                talk_stop("audio-stop")
                killed = say_stop()
                BRIDGE.audio_stop(int(body.get("id", 0) or 0))
                self._send(200, {"ok": True, "say_stopped": killed})
            elif path == "/api/audio/voice":
                BRIDGE.audio_config(CFG_SET_VOICE, int(body["voice"]))
                self._send(200, {"ok": True})
            elif path == "/api/estop":
                # 先连发零 0.6s（STEER 刹停惯性 1.5~2s，撤 ARM 后 nav_loop 就不再发了）
                for _ in range(12):
                    BRIDGE.publish_steer(0.0, 0.0)
                    time.sleep(0.05)
                with LOCK:
                    NAV_CMD = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    NAV_LAST_TS = 0.0
                    ARMED = False
                BRIDGE.publish_state(2)
                log("E-STOP")
                self._send(200, {"ok": True, "armed": False})
            elif path == "/api/shout/clip":
                # 阶段一：整段 WAV（二进制 body）→ sayd 播完即止
                if not body or len(body) > SHOUT_MAX_BYTES:
                    self._send(400, {"error": "empty or too large audio"})
                    return
                try:
                    info = shout_clip(body)
                    self._send(200, {"ok": True, "mode": "clip", **info})
                except ValueError as e:
                    self._send(400, {"error": str(e)})
            elif path == "/api/shout/start":
                # 阶段二：开会话
                rate = body.get("rate", 16000); channels = body.get("channels", 1)
                try:
                    self._send(200, shout_start(rate, channels))
                except Exception as e:
                    self._send(503, {"error": "sayd unavailable: %s" % e})
            elif path == "/api/shout/pcm":
                # 阶段二：推一片 PCM（二进制 body）
                if not body:
                    self._send(400, {"error": "empty pcm"})
                elif len(body) > 128 * 1024:
                    self._send(400, {"error": "pcm chunk too large"})
                else:
                    ok = shout_push(body)
                    self._send(200 if ok else 409,
                               {"ok": ok, **shout_state()})
            elif path == "/api/shout/stop":
                shout_stop()
                self._send(200, {"ok": True, **shout_state()})
            elif path == "/api/talk/start":
                try:
                    st = talk_start()
                    self._send(200 if st.get("ok") else 503, st)
                except Exception as e:
                    self._send(500, {"error": "talk start failed: %s" % e})
            elif path == "/api/talk/stop":
                talk_stop("client")
                self._send(200, {"ok": True, **talk_state()})
            elif path == "/api/person/switch":
                # REQ-049：4 路摄像头人员检测开关（on=True 检测，False 停止）
                # 写 switches.json 原子文件，s10yk-persond 监听后释放/重连对应 RTSP 流
                cam = str(body.get("cam", "")).strip()
                names = dict(PERSON_CAMS)
                if cam not in names:
                    self._send(400, {"error": "invalid cam, expect one of %s" % ",".join(names)})
                    return
                on = bool(body.get("on"))
                # 读改写保留未涉及路的现值（文件缺失时缺省全开）
                cur = {c: True for c, _ in PERSON_CAMS}
                try:
                    with open(PERSON_SWITCHES, "r", encoding="utf-8") as f:
                        loaded = json.load(f)
                    for c, _ in PERSON_CAMS:
                        if c in loaded:
                            cur[c] = bool(loaded[c])
                except Exception:
                    pass
                if cur[cam] == on:
                    self._send(200, {"ok": True, "switches": cur})   # 幂等：无变化
                    return
                cur[cam] = on
                tmp = PERSON_SWITCHES + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(cur, f, ensure_ascii=False)
                os.replace(tmp, PERSON_SWITCHES)
                log("PERSON switch %s(%s) -> %s" % (cam, names[cam], "ON" if on else "OFF"))
                self._send(200, {"ok": True, "switches": cur})
            else:
                self._send(404, {"error": "not found"})
        except Exception as e:
            log("POST %s error: %r" % (path, e))
            self._send(500, {"error": repr(e)})


def main():
    global BRIDGE
    rclpy.init()
    BRIDGE = S10Bridge()
    spin = threading.Thread(target=rclpy.spin, args=(BRIDGE,), daemon=True)
    spin.start()
    threading.Thread(target=nav_loop, daemon=True).start()
    threading.Thread(target=audio_bootstrap, daemon=True).start()
    threading.Thread(target=volume_keeper, daemon=True).start()
    threading.Thread(target=cam_reaper, daemon=True).start()
    threading.Thread(target=person_voice_watcher, daemon=True).start()   # REQ-047：人体预警语音
    load_preset_durations()
    host = "0.0.0.0"
    port = int(os.environ.get("S10YK_PORT", "8080"))
    auth_s = "ON" if AUTH_TOKEN else "OFF (S10YK_TOKEN 未设置)"
    log("S10YK backend listening on http://%s:%d  [auth: %s]" % (host, port, auth_s))
    # TLS 双监听：与明文共用同一 Handler/全局状态（ARMED/SAY/SHOUT 不割裂）。
    # 证书未配置时维持纯 HTTP 行为；配置后额外起 HTTPS 线程，麦克风在安全上下文可用。
    # v3.5.9：TLS 握手绝不能发生在 accept 线程。
    # 旧实现 wrap_socket(listen_socket) 让 accept() 同步做 TLS 握手——公网扫描器
    # TCP 连上却不发 ClientHello，accept 线程即永久挂死：accept 队列堆满（16:10
    # 事故堆了 111 个），18443 全站超时而 18090 明文正常。v3.5.5 的 backlog=128
    # 只是延后症状。现在 get_request 只收裸 TCP，握手挪到 handler 线程（见 Handler.setup）。
    class BackendHTTPServer(ThreadingHTTPServer):
        request_queue_size = 128
        daemon_threads = True
        tls_ctx = None                     # 非 None 时在 handler 线程做 TLS 握手

        def get_request(self):
            sock, addr = self.socket.accept()
            if self.tls_ctx is not None:
                sock.settimeout(15)        # 握手超时：坏连接 15s 内作废释放线程
                sock = self.tls_ctx.wrap_socket(sock, server_side=True,
                                                do_handshake_on_connect=False)
            return sock, addr

        def handle_error(self, request, client_address):
            import sys as _sys
            et, ev, _tb = _sys.exc_info()
            if isinstance(ev, ssl.SSLError):
                return                     # 坏握手/握手超时，公网常态噪音，静默丢弃
            super().handle_error(request, client_address)

    if TLS_CERT and TLS_KEY and os.path.exists(TLS_CERT) and os.path.exists(TLS_KEY):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(TLS_CERT, TLS_KEY)
        https = BackendHTTPServer((host, TLS_PORT), Handler)
        https.tls_ctx = ctx
        threading.Thread(target=https.serve_forever, daemon=True).start()
        log("S10YK backend HTTPS listening on https://%s:%d (SHOUT mic enabled, backlog=128, handshake-in-handler)" % (host, TLS_PORT))
    BackendHTTPServer((host, port), Handler).serve_forever()


if __name__ == "__main__":
    main()
