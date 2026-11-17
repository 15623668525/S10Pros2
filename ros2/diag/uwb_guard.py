#!/usr/bin/env python3
# 位移守护：以 UWB 绝对坐标为准，一旦位移超限立即连续发零速并保持零速
import sys, time, math
import rclpy
from rclpy.node import Node
from drdds.msg import NavCmd
from ubeacon_driver.msg import LocationResult

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
LOG = '/home/user/uwb_guard.log'

f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")} limit={LIMIT}\n')

class G(Node):
    def __init__(self):
        super().__init__('uwb_guard')
        self.p0 = None
        self.cur = None
        self.buf = []
        self.tripped = False
        self.maxd = 0.0
        self.pub = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.create_subscription(LocationResult, '/ubeacon_driver/location_result', self.cb, 50)
    def cb(self, m):
        p = (m.pos[0], m.pos[1])
        self.cur = p
        if self.p0 is None:
            self.buf.append(p)
            if len(self.buf) >= 30:                    # 用前 30 帧均值做基准
                self.p0 = (sum(a for a, _ in self.buf)/len(self.buf),
                           sum(b for _, b in self.buf)/len(self.buf))
                f.write(f'# base=({self.p0[0]:.3f},{self.p0[1]:.3f})\n')
            return
        d = math.hypot(p[0]-self.p0[0], p[1]-self.p0[1])
        self.maxd = max(self.maxd, d)
        if d > LIMIT and not self.tripped:
            self.tripped = True
            f.write(f'!!! TRIP d={d:.3f} > {LIMIT} at {time.strftime("%H:%M:%S")}\n')
            print(f'!!! 位移 {d:.3f} 超限，立即零速', flush=True)
    def zero(self, n=6):
        for _ in range(n):
            m = NavCmd(); m.data.x_vel = 0.0; m.data.y_vel = 0.0; m.data.yaw_vel = 0.0
            self.pub.publish(m)

rclpy.init(); g = G()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(g, timeout_sec=0.05)
    if g.tripped:
        g.zero(3)           # 触限后持续压制零速
f.write(f'# end maxd={g.maxd:.3f} tripped={g.tripped}\n')
f.close()
print(f'maxd={g.maxd:.3f} tripped={g.tripped}', flush=True)
g.destroy_node(); rclpy.shutdown()
