#!/usr/bin/env python3
# 遥控器实测监听：120s，落盘 MotionInfo / NAV_CMD 每帧
import time
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, NavCmd

DUR = 120.0
LOG = '/home/user/rc_listen.log'
f = open(LOG, 'w', buffering=1)
f.write('# start %s\n' % time.strftime('%F %T'))

class S(Node):
    def __init__(self):
        super().__init__('rc_listen')
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
        self.create_subscription(NavCmd, '/NAV_CMD', self.nav, 100)
        self.navn = 0
    def nav(self, m):
        self.navn += 1
        try:
            f.write('%.3f NAV x=%+.3f y=%+.3f yaw=%+.3f\n' % (
                time.time(), m.data.x_vel, m.data.y_vel, m.data.yaw_vel))
        except Exception as e:
            f.write('%.3f NAV err %s\n' % (time.time(), e))
    def cb(self, m):
        d = m.data
        wz = 0.0
        for k in ('yaw_rate', 'yaw_vel', 'angular_vel_z', 'wz'):
            if hasattr(d, k):
                wz = getattr(d, k); break
        try:
            f.write('%.3f MI vx=%+.4f vy=%+.4f wz=%+.4f gait=%d h=%.3f st=%d\n' % (
                time.time(), d.vel_x, d.vel_y, wz,
                d.gait_state.gait, d.height, d.motion_state.state))
        except Exception as e:
            f.write('%.3f MI err %s\n' % (time.time(), e))

rclpy.init(); n = S()
end = time.time() + DUR
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
f.write('# end nav=%d %s\n' % (n.navn, time.strftime('%F %T')))
f.close()
print('done nav=%d' % n.navn)
