#!/usr/bin/env python3
# 记录 NAV_CMD 停止后的速度/位移衰减曲线：vel_x 来自 MOTION_INFO，位移来自 UWB
import sys, time, math
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo
from ubeacon_driver.msg import LocationResult

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
LOG = '/home/user/stop_curve.log'
f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")}\n')

class S(Node):
    def __init__(self):
        super().__init__('stop_curve')
        self.vx = None; self.h = None; self.g = None
        self.p = None; self.p0 = None
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
        self.create_subscription(LocationResult, '/ubeacon_driver/location_result', self.cu, 50)
    def cb(self, m):
        self.vx = m.data.vel_x; self.h = m.data.height; self.g = m.data.gait_state.gait
    def cu(self, m):
        self.p = (m.pos[0], m.pos[1])

rclpy.init(); n = S()
t0 = time.time(); last = 0
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.time() - t0
    if el - last >= 0.5:
        last = el
        if n.p and n.p0 is None:
            n.p0 = n.p
        d = math.hypot(n.p[0]-n.p0[0], n.p[1]-n.p0[1]) if (n.p and n.p0) else 0.0
        f.write(f'{time.time():.3f} t={el:5.1f}s vel_x={n.vx if n.vx is None else round(n.vx,4):>8} d={d:.3f} h={n.h if n.h is None else round(n.h,3)} gait={n.g}\n')
f.write('# end\n')
f.close()
n.destroy_node(); rclpy.shutdown()
