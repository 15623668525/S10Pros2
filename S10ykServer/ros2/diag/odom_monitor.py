#!/usr/bin/env python3
# 被动监控：只订阅 ODOM + MOTION_INFO，不发任何指令（避免与后端抢发 NAV_CMD）
import sys, time, math
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
LOG = sys.argv[2] if len(sys.argv) > 2 else '/home/user/odom.log'
f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")}\n')

class M(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        self.h = None; self.st = None; self.g = None
        self.p = None; self.p0 = None
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
        self.create_subscription(Odometry, '/ODOM', self.cbo, 50)
    def cb(self, m):
        self.h = m.data.height; self.st = m.data.motion_state.state; self.g = m.data.gait_state.gait
        if self.p:
            d = math.hypot(self.p[0]-self.p0[0], self.p[1]-self.p0[1])
            f.write(f'{time.time():.3f} x={self.p[0]:+.3f} y={self.p[1]:+.3f} d={d:+.3f} h={self.h:.3f} st={self.st} gait={self.g}\n')
    def cbo(self, m):
        q = m.pose.pose.position
        self.p = (q.x, q.y)
        if self.p0 is None:
            self.p0 = self.p

rclpy.init(); n = M()
end = time.time() + DUR
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
f.write(f'# end\n')
f.close()
n.destroy_node(); rclpy.shutdown()
