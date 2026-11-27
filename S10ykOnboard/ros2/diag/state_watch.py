#!/usr/bin/env python3
# 监听 /MOTION_STATE 收到的指令（判断跨机下行是否到达）
import sys, time
import rclpy
from rclpy.node import Node
from drdds.msg import MotionState

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LOG = sys.argv[2] if len(sys.argv) > 2 else '/home/user/state.log'
f = open(LOG, 'a', buffering=1)
f.write(f'# start {time.strftime("%F %T")}\n')
class S(Node):
    def __init__(self):
        super().__init__('state_watch')
        self.n = 0
        self.create_subscription(MotionState, '/MOTION_STATE', self.cb, 100)
    def cb(self, m):
        self.n += 1
        f.write(f'{time.time():.3f} state={m.data.state}\n')
rclpy.init(); n = S()
end = time.time() + DUR
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
f.write(f'# end frames={n.n}\n')
f.close()
n.destroy_node(); rclpy.shutdown()
