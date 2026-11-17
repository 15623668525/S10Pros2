#!/usr/bin/env python3
import re

def parse_steer(p):
    out=[]; t=None; x=None; yaw=None
    for ln in open(p, errors='ignore'):
        s=ln.strip()
        if s.startswith('sec:'): t=int(s.split(':')[1])
        elif s.startswith('nanosec:'):
            if t is not None: t=t+int(s.split(':')[1])/1e9
        elif s.startswith('x:'): x=float(s.split(':')[1])
        elif s.startswith('yaw:'):
            yaw=float(s.split(':')[1])
            if t and x is not None: out.append((t,x,yaw)); x=None
    return out

def parse_mi(p):
    out=[]
    for ln in open(p, errors='ignore'):
        m=re.match(r'([\d.]+) MI vx=([+-][\d.]+) vy=([+-][\d.]+) wz=([+-][\d.]+) gait=(-?\d+) h=([\d.]+) st=(-?\d+)', ln)
        if m: out.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), int(m.group(5)), float(m.group(6)), int(m.group(7))))
    return out

st=parse_steer('/home/user/st_steer.log')
hd=parse_steer('/home/user/st_handle.log')
mi=parse_mi('/home/user/rc_listen.log')
print('steer frames=%d handle frames=%d mi frames=%d'%(len(st),len(hd),len(mi)))
if not mi: raise SystemExit
t0=mi[0][0]; t1=mi[-1][0]
print('MI 时间窗 %.1f s'%(t1-t0))
print()
print('%8s %-22s %-22s %s'%('t(s)','STEER x (min/max)','HANDLE x (min/max)','MOTION_INFO vx min/max | gait st'))
for k in range(0, int(t1-t0)+1, 2):
    a=t0+k; b=a+2
    sx=[f[1] for f in st if a<=f[0]<b]
    hx=[f[1] for f in hd if a<=f[0]<b]
    mv=[f[1] for f in mi if a<=f[0]<b]
    gs=[(f[4],f[6]) for f in mi if a<=f[0]<b]
    sa=('%+.3f/%+.3f'%(min(sx),max(sx))) if sx else '-'
    ha=('%+.3f/%+.3f'%(min(hx),max(hx))) if hx else '-'
    ma=('%+.4f/%+.4f'%(min(mv),max(mv))) if mv else '-'
    g=set(gs)
    print('%8.1f %-22s %-22s %s | %s'%(k, sa, ha, ma, sorted(g)))
