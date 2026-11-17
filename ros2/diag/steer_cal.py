#!/usr/bin/env python3
# /STEER 量纲标定：分档发 x，读 MOTION_INFO.vel_x 拟合比例；再验证 y(左右) / yaw(转向)
import time, sys
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, Steer

class C(Node):
    def __init__(self):
        super().__init__('steer_cal')
        self.vx = 0.0; self.vy = 0.0; self.wy = 0.0
        self.h = 0.0; self.st = 0; self.g = 0
        self.fid = 0
        self.pub = self.create_publisher(Steer, '/STEER', 10)
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.cb, 50)
    def cb(self, m):
        d = m.data
        self.vx = d.vel_x; self.vy = d.vel_y; self.wy = d.vel_yaw
        self.h = d.height; self.st = d.motion_state.state; self.g = d.gait_state.gait
    def send(self, x=0.0, y=0.0, yaw=0.0):
        m = Steer()
        self.fid += 1
        m.header.frame_id = self.fid
        m.header.stamp.sec = int(time.time())
        m.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        m.data.x = float(x); m.data.y = float(y); m.data.z = 0.0
        m.data.roll = 0.0; m.data.pitch = 0.0; m.data.yaw = float(yaw)
        self.pub.publish(m)

def hold(n, x=0.0, y=0.0, yaw=0.0, dur=2.0, tag=''):
    """持续 20Hz 下发，返回后半段 vel 稳态均值与峰值"""
    t0 = time.time(); rec = []
    while time.time() - t0 < dur:
        n.send(x, y, yaw)
        rclpy.spin_once(n, timeout_sec=0.02)
        rec.append((time.time() - t0, n.vx, n.vy, n.wy))
        time.sleep(0.03)
    late = rec[len(rec)//2:]
    avx = sum(r[1] for r in late)/len(late) if late else 0
    avy = sum(r[2] for r in late)/len(late) if late else 0
    awy = sum(r[3] for r in late)/len(late) if late else 0
    pk = max(abs(r[1]) for r in rec) if rec else 0
    print('[%s] 发 x=%+.3f y=%+.3f yaw=%+.3f  %ds -> vel_x后半均值=%+.4f 峰值=%.4f | vel_y=%+.4f vel_yaw=%+.4f | h=%.3f st=%d gait=%d'
          % (tag, x, y, yaw, dur, avx, pk, avy, awy, n.h, n.st, n.g), flush=True)
    return avx, pk

rclpy.init(); n = C()
t = time.time()
while time.time() - t < 1.5: rclpy.spin_once(n, timeout_sec=0.05)
print('起始 h=%.3f state=%d gait=%d vx=%+.4f' % (n.h, n.st, n.g, n.vx), flush=True)

if n.st != 17 or n.g != 4097:
    print('!! 非运动态(state=%d gait=%d)，先停在原地不动，中止' % (n.st, n.g), flush=True)
    n.send(0,0,0)
    rclpy.spin(n)
    sys.exit(2)

# ---- 阶段0：轻探针，确认 /STEER 发布是否真的被机器人接受 ----
avx, pk = hold(n, x=0.03, dur=2.0, tag='探针')
if abs(pk) < 0.004:
    print('!! x=0.03 无响应，升到 0.10 再试', flush=True)
    hold(n, x=0.0, dur=1.0, tag='回零')
    avx, pk = hold(n, x=0.10, dur=2.0, tag='探针2')
    if abs(pk) < 0.004:
        print('!! /STEER 发布无效（可能 QoS/权限不匹配）—— 中止', flush=True)
        for _ in range(20):
            n.send(0,0,0); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
        rclpy.spin(n)
        sys.exit(3)
hold(n, x=0.0, dur=1.5, tag='回零')

# ---- 阶段1：三档标定 ----
res = []
for xv in (0.05, 0.10, 0.15):
    if abs(n.vx) > 0.8:
        print('!! 超速保护触发 vx=%.3f' % n.vx, flush=True); break
    avx, pk = hold(n, x=xv, dur=2.0, tag='标定')
    res.append((xv, avx))
    hold(n, x=0.0, dur=1.5, tag='回零')

print('=== 标定结果 ===')
for xv, avx in res:
    k = avx/xv if xv else 0
    print('  x=%.2f -> vel_x=%+.4f  (k=%.3f 倍)' % (xv, avx, k))

# ---- 阶段2：左右与转向 ----
hold(n, y=0.06, dur=1.5, tag='左右')
hold(n, x=0.0, dur=1.2, tag='回零')
hold(n, yaw=0.06, dur=1.5, tag='转向')
hold(n, x=0.0, dur=1.2, tag='回零')

# ---- 收尾 ----
print('收尾：连续发零 1.5s', flush=True)
t0 = time.time()
while time.time() - t0 < 1.5:
    n.send(0,0,0); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
print('末态 h=%.3f state=%d gait=%d vx=%+.4f vy=%+.4f' % (n.h, n.st, n.g, n.vx, n.vy), flush=True)
print('DONE', flush=True)
n.destroy_node(); rclpy.shutdown()
