#!/usr/bin/env python3
# 1) 激活步态 4097（0x1001 基础步态）  2) 若成功，发极短小速度验证行走，位移硬限 0.2m
import sys, time, math
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, NavCmd, Gait
from nav_msgs.msg import Odometry

DO_MOVE = '--move' in sys.argv
LIMIT = 0.2          # 硬限：一旦累计位移超过 0.2m 立刻停
VX = 0.05            # m/s
HOLD = 1.2           # 秒

class P(Node):
    def __init__(self):
        super().__init__('gait_probe')
        self.h = None; self.st = None; self.g = None
        self.p0 = None; self.p = None
        self.pub_nav = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.pub_gait = self.create_publisher(Gait, '/GAIT', 10)
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
        self.create_subscription(Odometry, '/ODOM', self.cbo, 50)
    def cb(self, m):
        self.h = m.data.height; self.st = m.data.motion_state.state; self.g = m.data.gait_state.gait
    def cbo(self, m):
        p = m.pose.pose.position
        self.p = (p.x, p.y)
        if self.p0 is None:
            self.p0 = self.p
    def gait(self, v):
        m = Gait(); m.data.gait = v; self.pub_gait.publish(m)
        print(f'>>> 下发 GAIT = {v} (0x{v:04x})', flush=True)
    def nav(self, x, y=0.0, yaw=0.0):
        m = NavCmd(); m.data.x_vel = x; m.data.y_vel = y; m.data.yaw_vel = yaw
        self.pub_nav.publish(m)

rclpy.init(); n = P()
def spin_for(sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.05)

spin_for(2)
print(f'起始: height={round(n.h,4) if n.h else None} state={n.st} gait={n.g} odom={n.p}', flush=True)
n.gait(4097)
spin_for(6)
print(f'发步态后: height={round(n.h,4) if n.h else None} state={n.st} gait={n.g}', flush=True)

if n.g and DO_MOVE:
    p0 = n.p
    print(f'--- 开始小速度测试 x={VX} m/s 保持 {HOLD}s ---', flush=True)
    t0 = time.time(); last = 0
    stop = False
    while time.time() - t0 < HOLD:
        d = 0.0
        if p0 and n.p:
            d = math.hypot(n.p[0]-p0[0], n.p[1]-p0[1])
        if d > LIMIT:
            print(f'!! 位移 {d:.3f} 超限，立即停', flush=True); stop = True; break
        n.nav(VX)
        el = time.time()-t0
        if int(el*2) != last:
            last = int(el*2)
            print(f'  t={el:.1f}s 位移={d:.3f}m', flush=True)
        rclpy.spin_once(n, timeout_sec=0.02)
    for _ in range(10):
        n.nav(0.0); rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.05)
    print('--- 已发零速，观察 3s ---', flush=True)
    spin_for(3)
    if p0 and n.p:
        print(f'位移合计 = {math.hypot(n.p[0]-p0[0], n.p[1]-p0[1]):.3f} m', flush=True)
elif n.g == 0:
    print('步态未激活（gait 仍为 0），暂不测速度', flush=True)
print(f'结束: height={round(n.h,4) if n.h else None} state={n.st} gait={n.g}', flush=True)
n.destroy_node(); rclpy.shutdown()
