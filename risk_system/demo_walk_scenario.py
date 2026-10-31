#!/usr/bin/env python3
"""Scenario driver (never commands the robot): when the robot passes trigger_x
(distance from start, its odometry), the person walks across the corridor.
Position is a function of SIM time (camera stamps) and sent with Gazebo set_pose.
Usage: demo_walk_scenario.py <trigger_x> <speed> <timeout_wall_s>"""
import sys, json, time, subprocess, rclpy
from rclpy.node import Node
from std_msgs.msg import String
TRIG, SPEED, TIMEOUT = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
PX, Y0, Y1 = 3.2, -2.2, 1.7
cli = subprocess.Popen(['/workspace/risk_system/pose_client'], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, text=True)
def send(x, y):
    cli.stdin.write(f'standing_person {x} {y} 1.5708\n'); cli.stdin.flush()
rclpy.init(); n = Node('robot_risk_demo_walk'); ev = n.create_publisher(String, '/robot_risk/demo_event', 10)
S = {'ctrl': {}, 'stamp': None}
n.create_subscription(String, '/robot_risk/controller_state', lambda m: S.__setitem__('ctrl', json.loads(m.data)), 10)
def rk(m):
    st = json.loads(m.data).get('stamp')
    if st is not None: S['stamp'] = st
n.create_subscription(String, '/robot_risk/risk_state', rk, 10)
send(PX, Y0); t0 = time.time(); start = None; done = False; last = 0
while time.time() - t0 < TIMEOUT:
    try: rclpy.spin_once(n, timeout_sec=0.02)
    except Exception: break
    c = S['ctrl']
    if c.get('reason') == 'goal reached': break
    pos = c.get('pose_from_start')
    if start is None and pos and pos[0] >= TRIG and S['stamp'] is not None:
        start = S['stamp']; ev.publish(String(data='person_walk_start')); print('walk start, robot x', pos[0], 'sim', start, flush=True)
    if start is not None and not done and time.time() - last > 0.08 and S['stamp'] is not None:
        last = time.time(); y = min(Y1, Y0 + SPEED * (S['stamp'] - start))
        send(PX, y); ev.publish(String(data=f'person_pose:{PX}:{y:.3f}'))
        if y >= Y1:
            done = True; ev.publish(String(data='person_walk_end')); print('walk end sim', S['stamp'], flush=True)
cli.stdin.close()
