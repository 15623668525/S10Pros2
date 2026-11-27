#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S10 遥控应用 —— 103 业务板常驻语音播放器（sayd）

背景
----
原本自定义语音是「每次播放 -> ssh 到 103 -> 起一个 aplay 独占 plughw:0,0」。
问题有三：
  1) 每次播放要付 ~600ms 的 ssh 建链 + aplay 打开设备开销；
  2) 进程一旦异常（管道断裂、循环播报未退出）会**长期占住整个 PCM 设备**，
     厂商音频节点（/AUDIO/COMMAND 的系统音）就再也放不出来；
  3) 多路并发时会互相抢设备，出现 "Device or resource busy"。

本守护进程只做一件事：常驻在 103 上，按需开设备、空闲即释放。
  - 单实例、单播放：任何新 START 都会先关掉上一个 aplay，保证不叠音、不残留；
  - 空闲 IDLE_CLOSE 秒无数据 -> 主动关闭 aplay，把设备还给厂商节点；
  - 任何异常退出都走 finally close()，不留僵尸进程占设备。

协议（TCP，默认 18081）
------------------------
帧格式：  type(1B) | length(4B, big-endian) | payload
  type=1  START   payload = JSON {"rate":16000,"channels":1}
  type=2  PCM     payload = 原始 S16_LE 采样字节（可任意分片）
  type=3  STOP    payload 空
  type=4  PING    payload 空
控制帧（1/3/4）服务器回一行文本：
  "OK"                成功
  "BUSY <原因>"        设备被占用/打开失败
  "STATE idle|playing" PING 的回复
  "ERR <原因>"         其它错误

环境变量
  SAYD_ALSA_DEV  默认 plughw:0,0
  SAYD_PORT      默认 18081
  SAYD_IDLE      默认 1.2（秒）
  SAYD_BUFFER_US 默认 250000（aplay -B 缓冲，250ms；调小降延迟、调大抗抖动）
  SAYD_PERIOD_US 默认 50000 （aplay -F 周期，50ms）
"""

import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

ALSA_DEV = os.environ.get("SAYD_ALSA_DEV", "plughw:0,0")
PORT = int(os.environ.get("SAYD_PORT", "18081"))
IDLE_CLOSE = float(os.environ.get("SAYD_IDLE", "1.2"))
# 播放缓冲/周期（微秒）。aplay 默认缓冲约 500ms，是喊话端到端延迟的主要来源；
# 收到 100ms 一片的 PCM，250ms 缓冲 = 约 2.5 片余量，既降延迟又不至于一抖就断音。
BUFFER_US = int(os.environ.get("SAYD_BUFFER_US", "250000"))   # 250ms
PERIOD_US = int(os.environ.get("SAYD_PERIOD_US", "50000"))    # 50ms（每片 5 个周期）
MAX_SESSION = 1800.0        # 单连接最长存活，防止客户端失联后无限占用
FMT = "S16_LE"

F_START, F_PCM, F_STOP, F_PING = 1, 2, 3, 4


def log(msg):
    sys.stdout.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))
    sys.stdout.flush()


class Player(object):
    """唯一的播放器：保证同一时刻最多一个 aplay 进程。"""

    def __init__(self):
        self.p = None
        self.lock = threading.Lock()
        self.last_ts = 0.0
        self.rate = 16000
        self.channels = 1

    # ---- 内部 ----
    def _spawn(self, rate, channels):
        cmd = ["aplay", "-q", "-D", ALSA_DEV, "-t", "raw",
               "-f", FMT, "-r", str(rate), "-c", str(channels),
               "-B", str(BUFFER_US), "-F", str(PERIOD_US), "-"]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, close_fds=True)

    def _kill(self, wait_grace=1.0):
        p = self.p
        self.p = None
        if p is None:
            return
        try:
            if p.stdin:
                try:
                    p.stdin.close()
                except Exception:
                    pass
            try:
                p.wait(timeout=wait_grace)
            except Exception:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except Exception:
                    p.kill()
                    try:
                        p.wait(timeout=1.0)
                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            # 确保设备句柄一定被释放
            for f in (p.stdin, p.stderr):
                try:
                    if f:
                        f.close()
                except Exception:
                    pass

    # ---- 对外 ----
    def start(self, rate, channels):
        with self.lock:
            self._kill()
            self.rate, self.channels = rate, channels
            try:
                self.p = self._spawn(rate, channels)
            except Exception as e:
                return False, "spawn failed: %r" % (e,)
            # aplay 打开失败（设备被厂商节点占用）会立刻退出
            time.sleep(0.06)
            if self.p.poll() is not None:
                try:
                    err = (self.p.stderr.read() or b"").decode("utf-8", "ignore").strip()
                except Exception:
                    err = ""
                self.p = None
                return False, err or "aplay exited"
            self.last_ts = time.time()
            return True, ""

    def write(self, data):
        with self.lock:
            if self.p is None:
                return False
            try:
                self.p.stdin.write(data)
                self.p.stdin.flush()
                self.last_ts = time.time()
                return True
            except Exception as e:
                log("write error: %r" % (e,))
                self._kill()
                return False

    def stop(self):
        with self.lock:
            self._kill()

    def state(self):
        with self.lock:
            return "playing" if self.p is not None else "idle"

    def idle_check(self):
        with self.lock:
            if self.p is not None and (time.time() - self.last_ts) > IDLE_CLOSE:
                log("idle %.1fs -> release device" % (time.time() - self.last_ts))
                self._kill()


PLAYER = Player()


def _idle_loop():
    while True:
        time.sleep(0.2)
        try:
            PLAYER.idle_check()
        except Exception as e:
            log("idle loop error: %r" % (e,))


def _read_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _reply(conn, text):
    try:
        conn.sendall((text + "\n").encode("utf-8"))
    except Exception:
        pass


def handle(conn, addr):
    conn.settimeout(30.0)
    t0 = time.time()
    log("client %s:%d connected" % addr)
    try:
        while True:
            if time.time() - t0 > MAX_SESSION:
                log("session too long, closing")
                break
            hdr = _read_exact(conn, 5)
            if hdr is None:
                break
            t0 = time.time()  # v2.7.1 活跃即续期：收到任何帧都刷新计时，30 分钟上限只约束静默连接
            ftype, length = struct.unpack(">BI", hdr)
            payload = b""
            if length:
                payload = _read_exact(conn, length)
                if payload is None:
                    break

            if ftype == F_PCM:
                if not PLAYER.write(payload):
                    _reply(conn, "ERR write failed")
                continue

            if ftype == F_START:
                try:
                    cfg = json.loads(payload.decode("utf-8") or "{}")
                except Exception:
                    cfg = {}
                rate = int(cfg.get("rate", 16000))
                ch = int(cfg.get("channels", 1))
                ok, err = PLAYER.start(rate, ch)
                if ok:
                    _reply(conn, "OK")
                else:
                    log("start failed: %s" % err)
                    _reply(conn, "BUSY " + err.replace("\n", " ")[:160])
                continue

            if ftype == F_STOP:
                PLAYER.stop()
                _reply(conn, "OK")
                log("stopped by client")
                continue

            if ftype == F_PING:
                _reply(conn, "STATE " + PLAYER.state())
                continue

            _reply(conn, "ERR unknown frame %d" % ftype)
    except socket.timeout:
        log("client %s timeout" % (addr[0],))
    except Exception as e:
        log("client %s error: %r" % (addr[0], e))
    finally:
        # 客户端断开：立刻停播并释放设备（关键：不留占用）
        PLAYER.stop()
        try:
            conn.close()
        except Exception:
            pass
        log("client %s:%d closed" % addr)


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(4)
    threading.Thread(target=_idle_loop, daemon=True).start()
    log("sayd listening on :%d (dev=%s, idle=%.1fs, buf=%dms, period=%dms)"
        % (PORT, ALSA_DEV, IDLE_CLOSE, BUFFER_US // 1000, PERIOD_US // 1000))
    while True:
        try:
            conn, addr = srv.accept()
        except Exception as e:
            log("accept error: %r" % (e,))
            time.sleep(0.2)
            continue
        threading.Thread(target=handle, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
