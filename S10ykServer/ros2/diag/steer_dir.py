#!/usr/bin/env python3
# 遍历 STEER 的 y/z/yaw 字段，找左右平移与转向通道；带高度突变保护
import time
import rclpy
from rclpy.node import Node
from drdds.msg import MotionInfo, Steer

class C(Node):
    def __init__(self):
        super().__init__('steer_dir')
        self.vx=0.0; self.vy=0.0; self.wy=0.0; self.h=0.0; self.st=0; self.g=0
        self.fid=0
        self.pub=self.create_publisher(Steer,'/STEER',10)
        self.create_subscription(MotionInfo,'/MOTION_INFO',self.cb,50)
    def cb(self,m):
        d=m.data; self.vx=d.vel_x; self.vy=d.vel_y; self.wy=d.vel_yaw
        self.h=d.height; self.st=d.motion_state.state; self.g=d.gait_state.gait
    def send(self,x=0.0,y=0.0,z=0.0,yaw=0.0):
        m=Steer(); self.fid+=1
        m.header.frame_id=self.fid
        m.header.stamp.sec=int(time.time()); m.header.stamp.nanosec=int((time.time()%1)*1e9)
        m.data.x=float(x); m.data.y=float(y); m.data.z=float(z)
        m.data.roll=0.0; m.data.pitch=0.0; m.data.yaw=float(yaw)
        self.pub.publish(m)

rclpy.init(); n=C()
t=time.time()
while time.time()-t<1.2: rclpy.spin_once(n,timeout_sec=0.05)
print('起始 h=%.3f st=%d gait=%d'%(n.h,n.st,n.g),flush=True)
h0=n.h

def seg(tag,x=0.0,y=0.0,z=0.0,yaw=0.0,dur=1.2):
    t0=time.time(); rec=[]; aborted=False
    while time.time()-t0<dur:
        n.send(x,y,z,yaw)
        rclpy.spin_once(n,timeout_sec=0.02)
        rec.append((n.vy,n.wy,n.h,n.vx))
        if abs(n.h-h0)>0.06:
            aborted=True; break
        time.sleep(0.03)
    late=rec[len(rec)//2:] or rec
    f=lambda i: sum(r[i] for r in late)/len(late) if late else 0
    print('[%s] y=%+.2f z=%+.2f yaw=%+.2f %ds -> vel_y=%+.4f vel_yaw=%+.4f vel_x=%+.4f h=%.3f%s'
          %(tag,y,z,yaw,dur,f(0),f(1),f(3),n.h,'  !!高度突变中止' if aborted else ''),flush=True)
    return aborted

for tag,kw in (('y',dict(y=0.12)),('z',dict(z=0.12)),('yaw',dict(yaw=0.12)),('y*-1',dict(y=-0.12)),('yaw*-1',dict(yaw=-0.12))):
    if seg('试探',dur=1.2,**kw): break
    seg('归零',dur=0.8)

t0=time.time()
while time.time()-t0<1.2:
    n.send(0,0,0,0); rclpy.spin_once(n,timeout_sec=0.02); time.sleep(0.03)
print('末态 h=%.3f st=%d gait=%d vx=%+.4f vy=%+.4f wy=%+.4f'%(n.h,n.st,n.g,n.vx,n.vy,n.wy),flush=True)
print('DONE',flush=True)
n.destroy_node(); rclpy.shutdown()
