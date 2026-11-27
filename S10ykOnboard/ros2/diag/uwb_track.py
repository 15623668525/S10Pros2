#!/usr/bin/env python3
# 每秒输出一次 UWB 位置均值（滤噪），用于观察指令引起的台阶式位移
import sys, time, statistics
import rclpy
from rclpy.node import Node
from ubeacon_driver.msg import LocationResult

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LOG = '/home/user/uwb_track.log'
f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")}\n')

class T(Node):
    def __init__(self):
        super().__init__('uwb_track')
        self.buf = []
        self.create_subscription(LocationResult, '/ubeacon_driver/location_result', self.cb, 100)
    def cb(self, m):
        self.buf.append((m.pos[0], m.pos[1]))

rclpy.init(); n = T()
t0 = time.time(); last = t0
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    now = time.time()
    if now - last >= 1.0 and n.buf:
        xs = [p[0] for p in n.buf]; ys = [p[1] for p in n.buf]
        f.write(f'{now:.3f} n={len(n.buf):3d} x={statistics.mean(xs):.3f} y={statistics.mean(ys):.3f}\n')
        n.buf.clear(); last = now
f.write('# end\n')
f.close()
n.destroy_node(); rclpy.shutdown()
