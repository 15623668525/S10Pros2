#!/usr/bin/env python3
# 本地直接下发站立序列（1 -> 17），逐秒观察 height/state/gait，判断本体是否可动
import sys, time
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, MotionState

class P(Node):
    def __init__(self):
        super().__init__('stand_probe')
        self.h = None; self.st = None; self.g = None
        self.pub = self.create_publisher(MotionState, '/MOTION_STATE', 10)
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
    def cb(self, m):
        self.h = m.data.height
        self.st = m.data.motion_state.state
        self.g = m.data.gait_state.gait
    def send(self, s):
        m = MotionState(); m.data.state = s
        self.pub.publish(m)
        print(f'>>> 下发 MOTION_STATE = {s}', flush=True)

rclpy.init(); n = P()
t0 = time.time()
last = -1
sent17 = False
while time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
    el = time.time() - t0
    if int(el) != last:
        last = int(el)
        print(f't={el:4.0f}s height={n.h if n.h is None else round(n.h,4)} state={n.st} gait={n.g}', flush=True)
    if not sent17 and el > 9.0:
        sent17 = True
        n.send(17)
    if el > 1.0 and el < 1.2 and not getattr(n, '_s1', False):
        n._s1 = True
        n.send(1)
n.destroy_node(); rclpy.shutdown()
print(f'FINAL height={round(n.h,4) if n.h else None} state={n.st} gait={n.g}')
