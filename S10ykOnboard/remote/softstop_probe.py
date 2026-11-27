#!/usr/bin/env python3
"""测试 MOTION_STATE 2（软急停）的语义：保持站姿？还是趴下？"""
import json, sys, time, urllib.request
B = "http://127.0.0.1:18090"
def post(p, o):
    req = urllib.request.Request(B+p, data=json.dumps(o).encode(),
                                 headers={"Content-Type":"application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=4) as r:
        return json.load(r)
def m():
    with urllib.request.urlopen(B+"/api/status", timeout=4) as r:
        return json.load(r)["motion"]

print("before:", {k: m()[k] for k in ("height","state","gait")}, flush=True)
print("POST state=2:", post("/api/motion/state", {"state": 2}), flush=True)
for i in range(8):
    time.sleep(1)
    d = m()
    print(f"  {i+1}s height={d['height']:.4f} state={d['state']} gait={d['gait']}", flush=True)
