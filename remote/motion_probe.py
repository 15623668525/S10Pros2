#!/usr/bin/env python3
"""从后端 API 下发动作并轮询 motion 状态，验证跨机 MOTION_STATE 是否生效。
用法: motion_probe.py <state> [观察秒数]
"""
import json, sys, time, urllib.request

B = "http://127.0.0.1:18090"
state = int(sys.argv[1]) if len(sys.argv) > 1 else 1
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 18.0

def get_status():
    with urllib.request.urlopen(B + "/api/status", timeout=5) as r:
        return json.load(r)

def post(path, obj):
    data = json.dumps(obj).encode()
    req = urllib.request.Request(B + path, data=data,
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.load(r)

m = get_status()["motion"]
print(f"初始: height={m['height']:.4f} state={m['state']} gait={m['gait']}", flush=True)
print(f">>> POST /api/motion/state {state}", flush=True)
print("resp:", post("/api/motion/state", {"state": state}), flush=True)

t0 = time.time()
while time.time() - t0 < dur:
    m = get_status()["motion"]
    print(f"  {time.time()-t0:5.1f}s height={m['height']:.4f} state={m['state']} gait={m['gait']}", flush=True)
    time.sleep(1.0)
m = get_status()["motion"]
print(f"结束: height={m['height']:.4f} state={m['state']} gait={m['gait']}", flush=True)
