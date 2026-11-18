#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S10 人体检测抓拍服务（REQ-047 一期，跑在 204 服务器）

4 路 RTSP（103 板 8554）抽帧 → YOLOv8n(person) 推理 → 命中抓拍 + latest.json
后端 app.py 读 latest.json 注入 /api/status.person 并触发语音播报。

- 每路 3fps 抽帧检测（人体移动慢，捕获率 >99%，CPU 开销可控）
- 同路 8s 冷却去重；命中保存当前帧 + 前 1s 环形缓冲
- 抓拍存 DATA 目录（/opt/s10yk/data/person），保留 2 天自动清理
- 配额按 3 天图片存储空间预估（REQ-048）：超出即报警并停止保存
- stats.json：总人数/当前人数/存储配额，供前端人员检测页 1Hz 轮询
"""

import json
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort

# ---- 配置（环境变量可覆盖）----
# REQ-049 距离升级（2026-11-18）：imgsz=1280 + conf=0.35，有效检测距离 ≈13~14m → ≈30m
# （《人员检测有效距离分析_20261118》：640@0.45 可靠框高需 ≥51px，20m 人降采样后仅 23px 必漏检）
RTSP_BASE = os.environ.get("S10YK_RTSP_BASE", "rtsp://<ROBOT_IP>:8554")
MODEL = os.environ.get("PERSON_MODEL", "/opt/s10yk/models/yolov8n_1280.onnx")
SAVE_ROOT = os.environ.get("PERSON_SAVE_ROOT", "/opt/s10yk/data/person")   # REQ-048：DATA 目录
LATEST = os.path.join(SAVE_ROOT, "latest.json")
HISTORY = os.path.join(SAVE_ROOT, "history.jsonl")
STATUS = os.path.join(SAVE_ROOT, "detector_status.json")
STATS = os.path.join(SAVE_ROOT, "stats.json")   # REQ-048：统计信息（总人数/当前人数/存储）
SWITCHES = os.path.join(SAVE_ROOT, "switches.json")   # REQ-049：4 路摄像头检测开关（app.py 写、本服务读）
CONF_TH = float(os.environ.get("PERSON_CONF", "0.35"))
NMS_TH = float(os.environ.get("PERSON_NMS", "0.45"))
COOLDOWN_S = float(os.environ.get("PERSON_COOLDOWN", "8"))
DETECT_FPS = float(os.environ.get("PERSON_FPS", "3"))
KEEP_DAYS = int(os.environ.get("PERSON_KEEP_DAYS", "2"))          # REQ-048：只保留 2 天
QUOTA_DAYS = int(os.environ.get("PERSON_QUOTA_DAYS", "3"))        # REQ-048：配额按 3 天图片空间预估
QUOTA_MIN = float(os.environ.get("PERSON_QUOTA_MIN_MB", "100")) * 1024 * 1024   # 配额下限
INTRA_THREADS = int(os.environ.get("PERSON_THREADS", "8"))        # 1280 输入实测 8 线程 ≈190ms/帧

# cam_id: (url后缀, 名称, 声音键) —— 标签按 D-033 修正：video0后视 video2左视 video4前视 video6右视
CAMS = [
    ("video0", "后视", "back"),
    ("video2", "左视", "left"),
    ("video4", "前视", "front"),
    ("video6", "右视", "right"),
]


def log(msg):
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + str(msg)
    print(line, flush=True)
    try:
        with open("/opt/s10yk/logs/person_detect.log", "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        pass


# ---- YOLOv8 ONNX 解码 ----
class Detector:
    def __init__(self, path):
        so = ort.SessionOptions()
        so.intra_op_num_threads = INTRA_THREADS
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        inp = self.sess.get_inputs()[0]
        self.in_name = inp.name
        self.in_h = inp.shape[2] if isinstance(inp.shape[2], int) else 640
        self.in_w = inp.shape[3] if isinstance(inp.shape[3], int) else 640
        log("model loaded: %s input=%s" % (os.path.basename(path), (self.in_h, self.in_w)))

    def infer_person(self, bgr):
        """返回 [(x1,y1,x2,y2,conf), ...] 原图坐标"""
        h, w = bgr.shape[:2]
        r = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * r)), int(round(h * r))
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
        dx, dy = (self.in_w - nw) // 2, (self.in_h - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (self.in_w, self.in_h),
                                     swapRB=True, crop=False)
        out = self.sess.run(None, {self.in_name: blob})[0]
        arr = np.asarray(out)
        if arr.ndim == 3:
            arr = arr[0]
        # 兼容 (84,8400) 与 (8400,84) 两种排布
        if arr.shape[0] < arr.shape[1]:
            arr = arr.T                     # -> (8400,84)
        boxes = arr[:, :4]
        cls_scores = arr[:, 4:]
        person = cls_scores[:, 0]
        cand = []
        for i in np.where(person >= CONF_TH)[0]:
            cx, cy, bw, bh = boxes[i]
            cand.append([int(cx - bw / 2 - dx) / r, int(cy - bh / 2 - dy) / r,
                         int(cx + bw / 2 - dx) / r, int(cy + bh / 2 - dy) / r,
                         float(person[i])])
        if not cand:
            return []
        keep = cv2.dnn.NMSBoxes([[c[0], c[1], c[2] - c[0], c[3] - c[1]] for c in cand],
                                [c[4] for c in cand], CONF_TH, NMS_TH)
        if len(keep) == 0:
            return []
        keep = np.array(keep).flatten()
        return [cand[i] for i in keep]


def _atomic_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp, path)


# ---- REQ-048：统计与配额状态（多线程共享，加锁） ----
STATS_LOCK = threading.Lock()
STATS_OBJ = {"total_persons": 0, "current": 0,
             "storage": {"used": 0, "quota": 0, "full": False}, "ts": 0.0}
QUOTA = {"used": 0, "bytes": 0, "full": False, "warned": False}


def bump_total(n):
    with STATS_LOCK:
        STATS_OBJ["total_persons"] += n


OPEN_LOCK = threading.Lock()   # FFMPEG RTSP 并发 open 会死锁，串行化打开阶段


# ---- REQ-049：4 路检测开关（app.py 写 switches.json，本服务监听；缺省全开） ----
SWITCH_LOCK = threading.Lock()
SWITCH_STATE = {cid: True for cid, _, _ in CAMS}
SWITCH_MTIME = 0.0


def _poll_switches():
    """检查 switches.json 是否变化（mtime 触发，未变化零开销）。返回本路开关。"""
    global SWITCH_MTIME
    try:
        mt = os.path.getmtime(SWITCHES)
    except OSError:
        return
    if mt == SWITCH_MTIME:
        return
    SWITCH_MTIME = mt
    try:
        with open(SWITCHES, "r", encoding="utf-8") as f:
            obj = json.load(f)
        with SWITCH_LOCK:
            for cid in SWITCH_STATE:
                if cid in obj:
                    SWITCH_STATE[cid] = bool(obj[cid])
        log("switches reload: %s" % {cid: SWITCH_STATE[cid] for cid in SWITCH_STATE})
    except Exception as e:
        log("switches load error: %r" % e)


def switch_on(cam_id):
    with SWITCH_LOCK:
        return SWITCH_STATE.get(cam_id, True)


class CamWorker(threading.Thread):
    def __init__(self, cam_id, cam_name, voice_key, det):
        super().__init__(daemon=True, name="cam-" + cam_id)
        self.cam_id, self.cam_name, self.voice_key = cam_id, cam_name, voice_key
        self.det = det
        self.url = "%s/%s" % (RTSP_BASE, cam_id)
        self.last_hit = 0.0
        self.ring = deque(maxlen=5)         # ~1.6s @3fps
        self.drop_cnt = 0                   # REQ-048：配额满丢弃计数
        self.state = {"fps": 0.0, "frames": 0, "hits": 0, "last_err": "",
                      "cur_n": 0, "cur_ts": 0.0}   # cur_*：当前画面人数（REQ-048）

    def _save(self, frame, boxes):
        if QUOTA["full"]:                   # REQ-048：超配额报警后停止保存
            self.drop_cnt += 1
            if self.drop_cnt % 30 == 1:     # 限频记录，避免刷屏
                log("QUOTA full, snapshot dropped x%d (%s)" % (self.drop_cnt, self.cam_id))
            return
        ts = time.time()
        day = time.strftime("%Y%m%d", time.localtime(ts))
        tstr = time.strftime("%H%M%S", time.localtime(ts))
        d = os.path.join(SAVE_ROOT, self.cam_id, day)
        os.makedirs(d, exist_ok=True)
        tag = "%s_%s_c%d" % (day, tstr, int(ts % 1000))
        vis = frame.copy()
        for x1, y1, x2, y2, cf in boxes:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(vis, "person %.2f" % cf, (int(x1), max(14, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        img_path = os.path.join(d, tag + ".jpg")
        cv2.imwrite(img_path, vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
        pre = []
        for i, (pts, bts) in enumerate(list(self.ring)[:-1]):
            p = os.path.join(d, "%s_pre%d.jpg" % (tag, i))
            cv2.imwrite(p, bts, [cv2.IMWRITE_JPEG_QUALITY, 80])
            pre.append(os.path.relpath(p, SAVE_ROOT))
        rel = os.path.relpath(img_path, SAVE_ROOT)
        meta = {"ts": ts, "cam_id": self.cam_id, "cam_name": self.cam_name,
                "voice_key": self.voice_key, "conf": round(max(b[4] for b in boxes), 3),
                "n": len(boxes),
                "boxes": [[round(v, 1) for v in b[:4]] for b in boxes],
                "img": rel, "pre": pre}
        _atomic_json(LATEST, meta)
        bump_total(len(boxes))              # REQ-048：累计人次
        try:
            size = os.path.getsize(HISTORY) if os.path.exists(HISTORY) else 0
            if size > 5 * 1024 * 1024:
                os.replace(HISTORY, HISTORY + ".old")
            with open(HISTORY, "a", encoding="utf-8") as f:
                f.write(json.dumps(meta, ensure_ascii=False) + "\n")
        except Exception as e:
            log("history append error: %r" % e)
        self.state["hits"] += 1
        log("HIT %s %s conf=%.2f n=%d -> %s" %
            (self.cam_id, self.cam_name, meta["conf"], meta["n"], rel))

    def run(self):
        interval = 1.0 / DETECT_FPS
        last_t, nframes, fps_t0 = 0.0, 0, time.time()
        cap = None
        while True:
            # REQ-049：开关为关 → 释放连接停检测（省 CPU/带宽），为开 → 正常取流
            _poll_switches()
            if not switch_on(self.cam_id):
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    log("%s detection OFF, stream released" % self.cam_id)
                self.ring.clear()
                self.state["cur_n"] = 0
                self.state["cur_ts"] = time.time()
                self.state["last_err"] = ""
                time.sleep(1.0)
                continue
            try:
                if cap is None or not cap.isOpened():
                    log("%s connecting..." % self.url)
                    with OPEN_LOCK:
                        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = \
                            "rtsp_transport;tcp|timeout;5000000|stimeout;5000000"
                        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                        opened = cap.isOpened()
                        if not opened:
                            cap.release()
                            cap = None
                    if opened:
                        log("%s connected" % self.url)
                    else:
                        log("%s open failed, retry" % self.url)
                        time.sleep(2.0)
                    continue
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.state["last_err"] = "read fail"
                    cap.release()
                    cap = None
                    time.sleep(2.0)
                    continue
                nframes += 1
                self.ring.append((time.time(), frame.copy()))
                now = time.time()
                if now - last_t >= interval:
                    last_t = now
                    boxes = self.det.infer_person(frame)
                    self.state["cur_n"] = len(boxes) if boxes else 0    # REQ-048
                    self.state["cur_ts"] = now
                    if boxes and now - self.last_hit >= COOLDOWN_S:
                        self.last_hit = now
                        self._save(frame, boxes)
                if now - fps_t0 >= 10:
                    self.state["fps"] = round(nframes / (now - fps_t0), 1)
                    nframes, fps_t0 = 0, now
            except Exception as e:
                self.state["last_err"] = repr(e)
                log("cam %s error: %r" % (self.cam_id, e))
                time.sleep(2.0)
                try:
                    if cap is not None:
                        cap.release()
                except Exception:
                    pass
                cap = None


def _jpg_size_tree(root):
    """统计目录下全部 .jpg 字节数（配额只算图片）"""
    total = 0
    for r, _, fs in os.walk(root):
        for fn in fs:
            if fn.endswith(".jpg"):
                try:
                    total += os.path.getsize(os.path.join(r, fn))
                except OSError:
                    pass
    return total


def _estimate_quota():
    """REQ-048：配额 = 3 天图片存储空间预估。

    按天目录（cam*/YYYYMMDD）统计历史日均图片字节数，配额 = 日均 × QUOTA_DAYS；
    无历史数据时取 QUOTA_MIN 下限。
    """
    days = {}
    try:
        entries = os.listdir(SAVE_ROOT)
    except OSError:
        return int(QUOTA_MIN)
    for cam in entries:
        cd = os.path.join(SAVE_ROOT, cam)
        if not os.path.isdir(cd):
            continue
        for day in os.listdir(cd):
            dd = os.path.join(cd, day)
            if not os.path.isdir(dd):
                continue
            s = 0
            for fn in os.listdir(dd):
                if fn.endswith(".jpg"):
                    try:
                        s += os.path.getsize(os.path.join(dd, fn))
                    except OSError:
                        pass
            days[day] = days.get(day, 0) + s
    if not days:
        return int(QUOTA_MIN)
    return max(int(QUOTA_MIN), int(sum(days.values()) / len(days) * QUOTA_DAYS))


def _cleanup_loop():
    """REQ-048：每分钟清理 KEEP_DAYS 天前抓拍 + 核算配额（超出即报警停存）"""
    while True:
        try:
            cutoff = time.time() - KEEP_DAYS * 86400
            for root, _, files in os.walk(SAVE_ROOT):
                if root == SAVE_ROOT:
                    continue
                for fn in files:
                    if fn.endswith((".jpg", ".json")):
                        p = os.path.join(root, fn)
                        try:
                            if os.path.getmtime(p) < cutoff:
                                os.remove(p)
                        except OSError:
                            pass
            for d in os.listdir(SAVE_ROOT):
                cd = os.path.join(SAVE_ROOT, d)
                if os.path.isdir(cd):
                    try:
                        if not os.listdir(cd):
                            os.rmdir(cd)
                    except OSError:
                        pass
            used = _jpg_size_tree(SAVE_ROOT)
            quota = _estimate_quota()
            full = used >= quota and quota > 0
            if full and not QUOTA["warned"]:
                log("QUOTA EXCEEDED 报警: used=%.1fMB >= quota=%.1fMB，停止保存抓拍"
                    % (used / 1048576.0, quota / 1048576.0))
                QUOTA["warned"] = True
            elif not full and QUOTA["warned"]:
                log("QUOTA recovered: used=%.1fMB < quota=%.1fMB，恢复保存"
                    % (used / 1048576.0, quota / 1048576.0))
                QUOTA["warned"] = False
            QUOTA["used"], QUOTA["bytes"], QUOTA["full"] = used, quota, full
        except Exception as e:
            log("cleanup error: %r" % e)
        time.sleep(60)


def _stats_loop(workers):
    """REQ-048：每 2s 产出 stats.json（总人数/当前人数/存储配额），前端人员检测页 1Hz 轮询"""
    total = 0     # 启动时从 history.jsonl 恢复累计人次
    try:
        with open(HISTORY, "r", encoding="utf-8") as f:
            for line in f:
                try:
                    total += int(json.loads(line).get("n") or 0)
                except Exception:
                    pass
    except Exception:
        pass
    with STATS_LOCK:
        STATS_OBJ["total_persons"] = total
    log("stats init: total_persons=%d" % total)
    while True:
        try:
            now = time.time()
            cur = 0
            for w in workers:           # 当前人数 = 3s 内各路检测的最大人数
                if now - w.state.get("cur_ts", 0) <= 3.0:
                    cur = max(cur, w.state.get("cur_n", 0))
            with STATS_LOCK:
                STATS_OBJ["current"] = cur
                STATS_OBJ["storage"] = {"used": QUOTA["used"], "quota": QUOTA["bytes"],
                                        "full": QUOTA["full"]}
                # REQ-049：各路开关状态（前端人员检测页渲染 4 个开关）
                with SWITCH_LOCK:
                    STATS_OBJ["switches"] = dict(SWITCH_STATE)
                STATS_OBJ["ts"] = now
            _atomic_json(STATS, STATS_OBJ)
        except Exception as e:
            log("stats error: %r" % e)
        time.sleep(2)


def main():
    if not os.path.exists(MODEL):
        log("FATAL: model not found: " + MODEL)
        raise SystemExit(2)
    os.makedirs(SAVE_ROOT, exist_ok=True)
    os.makedirs("/opt/s10yk/logs", exist_ok=True)
    det = Detector(MODEL)
    threading.Thread(target=_cleanup_loop, daemon=True).start()
    workers = []
    for cam_id, cam_name, vk in CAMS:
        w = CamWorker(cam_id, cam_name, vk, det)
        w.start()
        workers.append(w)
    threading.Thread(target=_stats_loop, args=(workers,), daemon=True).start()   # REQ-048
    log("person detect service started: %d cams, conf>=%.2f, cooldown=%.0fs, %.0ffps, keep=%dd"
        % (len(workers), CONF_TH, COOLDOWN_S, DETECT_FPS, KEEP_DAYS))
    while True:
        time.sleep(10)
        _atomic_json(STATUS, {"ts": time.time(),
                              "cams": {w.cam_id: w.state for w in workers}})


if __name__ == "__main__":
    main()
