#!/usr/bin/env python3
# 行走测试：保持当前步态（不主动设 gait），发小速度看是否位移；位移硬限保护
import sys, time, math
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, NavCmd
from nav_msgs.msg import Odometry

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2
LIMIT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20

class W(Node):
    def __init__(self):
        super().__init__('walk_test')
        self.h = None; self.st = None; self.g = None
        self.p = None; self.p0 = None
        self.pub = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
        self.create_subscription(Odometry, '/ODOM', self.cbo, 50)
    def cb(self, m):
        self.h = m.data.height; self.st = m.data.motion_state.state; self.g = m.data.gait_state.gait
    def cbo(self, m):
        q = m.pose.pose.position
        self.p = (q.x, q.y)
        if self.p0 is None:
            self.p0 = self.p
    def nav(self, x):
        m = NavCmd(); m.data.x_vel = x; m.data.y_vel = 0.0; m.data.yaw_vel = 0.0
        self.pub.publish(m)

rclpy.init(); w = W()
t = time.time()
while time.time() - t < 2:
    rclpy.spin_once(w, timeout_sec=0.05)
print(f'起始: height={round(w.h,4) if w.h else None} state={w.st} gait={w.g} odom={w.p}', flush=True)

t0 = time.time(); last = -1
while time.time() - t0 < HOLD:
    d = math.hypot(w.p[0]-w.p0[0], w.p[1]-w.p0[1]) if (w.p and w.p0) else 0.0
    if d > LIMIT:
        print(f'!! 位移 {d:.3f} 超限 -> 立即停', flush=True)
        break
    w.nav(VX)
    el = time.time() - t0
    if int(el*2) != last:
        last = int(el*2)
        print(f'  t={el:4.1f}s 位移={d:.3f}m gait={w.g}', flush=True)
    rclpy.spin_once(w, timeout_sec=0.02)

for _ in range(12):
    w.nav(0.0); rclpy.spin_once(w, timeout_sec=0.01); time.sleep(0.05)
print('零速已发，观察 3s', flush=True)
t = time.time()
while time.time() - t < 3:
    rclpy.spin_once(w, timeout_sec=0.05)
d = math.hypot(w.p[0]-w.p0[0], w.p[1]-w.p0[1]) if (w.p and w.p0) else 0.0
print(f'位移合计 = {d:.3f} m   末态: height={round(w.h,4) if w.h else None} state={w.st} gait={w.g}', flush=True)
w.destroy_node(); rclpy.shutdown()
