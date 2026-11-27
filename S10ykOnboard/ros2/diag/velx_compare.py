#!/usr/bin/env python3
# 用机器人自报的 vel_x 做判据：gait=0 vs gait=4097 时速度指令是否被接受
import sys, time
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, NavCmd

PHASE = sys.argv[1] if len(sys.argv) > 1 else 'test'   # nogait|gait

class V(Node):
    def __init__(self):
        super().__init__('velx_cmp')
        self.vx = None; self.g = None; self.h = None; self.st = None
        self.pub = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
    def cb(self, m):
        self.vx = m.data.vel_x; self.g = m.data.gait_state.gait
        self.h = m.data.height; self.st = m.data.motion_state.state
    def nav(self, x):
        m = NavCmd(); m.data.x_vel = x; m.data.y_vel = 0.0; m.data.yaw_vel = 0.0
        self.pub.publish(m)

rclpy.init(); v = V()
t = time.time()
while time.time() - t < 2: rclpy.spin_once(v, timeout_sec=0.05)
print(f'[{PHASE}] 起始 h={round(v.h,3)} state={v.st} gait={v.g} vel_x={v.vx}', flush=True)

samples = []
t0 = time.time()
while time.time() - t0 < 2.0:
    v.nav(0.05)
    rclpy.spin_once(v, timeout_sec=0.02)
    samples.append(v.vx)
for _ in range(15):
    v.nav(0.0); rclpy.spin_once(v, timeout_sec=0.01); time.sleep(0.03)

mx = max([s for s in samples if s is not None] or [0.0])
mn = min([s for s in samples if s is not None] or [0.0])
print(f'[{PHASE}] 发 x=0.05 两秒：vel_x 区间 [{mn:.4f}, {mx:.4f}]  样本={len(samples)}', flush=True)
t = time.time()
while time.time() - t < 2: rclpy.spin_once(v, timeout_sec=0.05)
print(f'[{PHASE}] 松手后 vel_x={v.vx}  gait={v.g} h={round(v.h,3)}', flush=True)
print(f'[{PHASE}] 判定: {"接受速度指令" if mx > 0.005 else "忽略速度指令"}', flush=True)
v.destroy_node(); rclpy.shutdown()
