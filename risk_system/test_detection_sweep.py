#!/usr/bin/env python3
"""Detection / risk sweep with the robot teleported (set_pose, sim only) to known
distances in front of obstacle_1 and the person.  The controller must NOT run.
Writes logs/detection_sweep.csv and prints a table.  Ground-truth distance is
from the SDF geometry (camera at robot x + 0.23 m)."""
import json, subprocess, time, csv, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

OBST1_FACE = -1.0 - 0.175        # obstacle_1 front face x
PERSON_FACE = 2.0 - 0.16         # person torso front face x
CAM_X = 0.23


def set_pose(name, x, y, yaw=0.0):
    q = f'orientation: {{z: {math.sin(yaw/2):.6f}, w: {math.cos(yaw/2):.6f}}}'
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.0}}, {q}'
    subprocess.run(['ign', 'service', '-s', '/world/robot_risk_world/set_pose', '--reqtype',
                    'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                    '--req', req], check=True, capture_output=True)


rclpy.init()
n = Node('robot_risk_detection_sweep')
last = {}
n.create_subscription(String, '/robot_risk/risk_state', lambda m: last.update(json.loads(m.data)), 10)


def sample(settle=2.5, k=6):
    t0 = time.time()
    while time.time() - t0 < settle:
        rclpy.spin_once(n, timeout_sec=0.1)
    out = []
    while len(out) < k:
        last.clear()
        while not last:
            rclpy.spin_once(n, timeout_sec=0.1)
        out.append(dict(last))
    return out


cases = []
for d in (4.0, 3.5, 3.0, 2.5, 2.0, 1.7, 1.5, 1.3):
    cases.append(('obstacle_1', OBST1_FACE - d - CAM_X, 0.0, d))
# person sweep in an open lane (y=-1.5): the person is moved there (sim only)
LANE = -1.5
PERSON_X = 2.0
for d in (4.5, 4.0, 3.5, 3.0, 2.5, 2.0):
    cases.append(('person', PERSON_X - 0.16 - d - CAM_X, LANE, d))
cases.append(('clear_view_turned', -3.5, 0.0, None))   # facing -x: only the floor edge

rows = []
person_moved = False
for target, x, y, d in cases:
    if target == 'person' and not person_moved:
        set_pose('standing_person', PERSON_X, LANE)
        person_moved = True
    if target != 'person' and person_moved:
        set_pose('standing_person', 2.0, -0.35)
        person_moved = False
    yaw = math.pi if target == 'clear_view_turned' else 0.0
    set_pose('risk_robot', x, y, yaw)
    s = sample()
    st = [r['state'] for r in s]
    rk = [r['risk'] for r in s]
    tgt = [o for o in s[-1].get('objects', []) if o['in_path']]
    best = min(tgt, key=lambda o: o['distance']) if tgt else None
    row = {'target': target, 'gt_distance': d, 'robot_x': round(x, 3), 'robot_y': y,
           'states': '|'.join(sorted(set(st))), 'risk_mean': round(sum(rk) / len(rk), 1),
           'est_distance': best['distance'] if best else '', 'label': best['label'] if best else '',
           'conf': best['confidence'] if best else '', 'h_m': best['height_m'] if best else '',
           'w_m': best['width_m'] if best else '', 'proc_ms': s[-1].get('proc_ms')}
    rows.append(row)
    print(row, flush=True)

set_pose('risk_robot', -3.5, 0.0)
set_pose('standing_person', 2.0, -0.35)
with open('/workspace/risk_system/logs/detection_sweep.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
print('saved logs/detection_sweep.csv')
