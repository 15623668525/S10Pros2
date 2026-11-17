#!/usr/bin/env python3
# 长驻监听 /NAV_CMD，落盘每帧（含到达时间），用于判断跨机下行是否真的到达 106
import sys, time
import rclpy
from rclpy.node import Node
from drdds.msg import NavCmd

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
LOG = sys.argv[2] if len(sys.argv) > 2 else '/home/user/navcmd.log'

f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")}\n')

class S(Node):
    def __init__(self):
        super().__init__('navcmd_watch')
        self.n = 0
        self.create_subscription(NavCmd, '/NAV_CMD', self.cb, 100)
    def cb(self, m):
        self.n += 1
        f.write(f'{time.time():.3f} x={m.data.x_vel:+.3f} y={m.data.y_vel:+.3f} yaw={m.data.yaw_vel:+.3f}\n')

rclpy.init(); n = S()
end = time.time() + DUR
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
f.write(f'# end frames={n.n}\n')
f.close()
n.destroy_node(); rclpy.shutdown()
