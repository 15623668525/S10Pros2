#!/usr/bin/env python3
"""A/B 对照：趴下(gait=0) vs 站立(gait=4097)，用跨机真实路径发速度，比较机器人自报 vel_x。
判据用 vel_x（不依赖 UWB/ODOM，避免测量噪声）。
"""
import json, time, urllib.request

B = "http://127.0.0.1:18090"
def post(p, o):
    req = urllib.request.Request(B+p, data=json.dumps(o).encode(),
                                 headers={"Content-Type":"application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=4) as r:
        return json.load(r)
def motion():
    with urllib.request.urlopen(B+"/api/status", timeout=4) as r:
        return json.load(r)["motion"]

def send_speed(x, dur=2.0):
    """持续发速度 dur 秒，返回观测到的 vel_x 最大值"""
    mx = 0.0
    t0 = time.time()
    while time.time() - t0 < dur:
        post("/api/nav", {"x": x, "y": 0.0, "yaw": 0.0})
        v = abs(float(motion().get("vel_x") or 0.0))
        mx = max(mx, v)
        time.sleep(0.1)
    for _ in range(8):
        post("/api/nav", {"x": 0.0, "y": 0.0, "yaw": 0.0}); time.sleep(0.1)
    time.sleep(2)
    return mx

post("/api/arm", {"armed": True})

print("=" * 50, flush=True)
print("A: 趴下状态 (gait=0)", flush=True)
post("/api/motion/state", {"state": 2})
time.sleep(6)
d = motion(); print(f"  状态 h={d['height']:.3f} state={d['state']} gait={d['gait']}", flush=True)
mxA = send_speed(0.05)
print(f"  → vel_x 最大 = {mxA:.4f}", flush=True)

print("=" * 50, flush=True)
print("B: 站起 (自动挂 gait 4097)", flush=True)
post("/api/motion/state", {"state": 1})
time.sleep(11)
d = motion(); print(f"  状态 h={d['height']:.3f} state={d['state']} gait={d['gait']}", flush=True)
mxB = send_speed(0.05)
print(f"  → vel_x 最大 = {mxB:.4f}", flush=True)

print("=" * 50, flush=True)
print(f"A(gait=0) vel_x={mxA:.4f}   B(gait=4097) vel_x={mxB:.4f}", flush=True)
if mxA < 0.005 and mxB > 0.005:
    print("结论：gait=0 完全忽略速度，gait=4097 接受 → 挂步态是必要条件", flush=True)
elif mxA > 0.005 and mxB > 0.005:
    print("结论：两种状态都接受速度 → gait 不是原因，需另查", flush=True)
else:
    print("结论：两种状态都不接受速度 → 还有其他前置条件", flush=True)
