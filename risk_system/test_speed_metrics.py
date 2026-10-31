#!/usr/bin/env python3
"""Speed-related measurements (controller must NOT run), slalom world.
A) braking from v: drive open loop in a clear lane, command zero, measure GT distance and pitch.
B) detection delay: obstacle placed in view; time until detector reports STOP (wall and sim).
Writes logs/slalom_speed_metrics.txt"""
import math, subprocess, time, json, threading, sys
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
RS = '/workspace/risk_system'; OUT = open(f'{RS}/logs/slalom_speed_metrics.txt', 'w')
def log(*a):
    s = ' '.join(str(x) for x in a); print(s, flush=True); OUT.write(s + '\n'); OUT.flush()
cli = subprocess.Popen([f'{RS}/pose_client'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
def sp(name, x, y):
    cli.stdin.write(f'{name} {x} {y} 0\n'); cli.stdin.flush(); return cli.stdout.readline().strip()
rclpy.init(); n = Node('robot_risk_speed_metrics'); pub = n.create_publisher(Twist, '/robot_risk/cmd_vel', 10)
S = {'p': None, 'pitch': 0.0, 'risk': {}, 'hist': []}
def gt(m):
    t = m.transforms[0]; q = t.transform.rotation
    S['p'] = (t.transform.translation.x, t.transform.translation.y)
    S['pitch'] = math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
    S['hist'].append((time.time(), *S['p'])); S['hist'] = S['hist'][-300:]
n.create_subscription(TFMessage, '/world/robot_risk_world/dynamic_pose/info', gt, 10)
n.create_subscription(String, '/robot_risk/risk_state', lambda m: S.__setitem__('risk', json.loads(m.data)), 10)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
def cmd(v, dur):
    t0 = time.time(); pmax = 0
    while time.time() - t0 < dur:
        t = Twist(); t.linear.x = float(v); pub.publish(t); pmax = max(pmax, abs(S['pitch'])); time.sleep(0.05)
    return pmax
def still():
    t0 = time.time(); pmax = 0
    while time.time() - t0 < 12:
        pub.publish(Twist()); pmax = max(pmax, abs(S['pitch'])); time.sleep(0.1)
        h = [p for p in S['hist'] if p[0] > time.time() - 0.8]
        if time.time() - t0 > 1.0 and len(h) > 1 and math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2]) < 0.002:
            return time.time() - t0, pmax
    return None, pmax
time.sleep(2)
for v in (0.15, 0.30, 0.30, 0.30, 0.40):
    sp('risk_robot', -5.0, -1.7); cmd(0, 2)
    pa = cmd(v, 8); p0 = S['p']; t, pb = still(); p1 = S['p']
    log(f'A brake v={v}: travelled {p0[0] + 5.0:.2f} m, stop distance {math.hypot(p1[0]-p0[0], p1[1]-p0[1]):.3f} m, settle {t} s wall, |pitch| max drive {pa:.2f} brake {pb:.2f} deg')
sp('risk_robot', -5.0, 0.0); time.sleep(0.5)
# B: robot faces where obstacle_1 reappears, ~1.8 m from its face
sp('risk_robot', -3.5, 0.25); ox, oy = -1.4, 0.25
for i in range(6):
    sp('obstacle_1', ox, -2.3 if False else 9.0); time.sleep(3)
    before = S['risk'].get('state')
    t0 = time.time(); st0 = S['risk'].get('stamp'); sp('obstacle_1', ox, oy)
    while S['risk'].get('state') != 'STOP' and time.time() - t0 < 8: time.sleep(0.005)
    w = time.time() - t0; st1 = S['risk'].get('stamp')
    log(f'B detection {i}: before={before} -> {S["risk"].get("state")} in {w*1000:.0f} ms wall, {(st1-st0)*1000 if st0 and st1 else -1:.0f} ms sim (includes frame interval), proc {S["risk"].get("proc_ms")} ms')
sp('obstacle_1', ox, oy); sp('risk_robot', -5.0, 0.0)
cli.stdin.close()
