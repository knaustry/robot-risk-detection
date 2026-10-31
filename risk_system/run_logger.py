#!/usr/bin/env python3
"""Log a closed-loop run: run_logger.py <seconds> <csv>  (stops early on goal reached)
Columns: wall, sim_t (GT), GT pose, risk state/score, nearest in-path estimate,
controller mode/reason, cmd v/w, min clearance robot<->objects (GT geometry)."""
import sys, json, math, time, csv, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
# static boxes from robot_risk_world.sdf (x, y, half_x, half_y); person moves -> read from args/default
BOXES = {'obstacle_1': (-1.0, 0.0, 0.175, 0.40), 'obstacle_2': (1.0, 0.8, 0.30, 0.30)}
PERSON = [2.0, -0.35, 0.16, 0.11]
import os
if os.environ.get('RISK_SCENE_JSON'):   # scene file: footprints x0,x1,y0,y1
    _sc = json.load(open(os.environ['RISK_SCENE_JSON']))
    BOXES = {k: ((v[0] + v[1]) / 2, (v[2] + v[3]) / 2, (v[1] - v[0]) / 2, (v[3] - v[2]) / 2) for k, v in _sc['boxes'].items()}
    PERSON = [_sc['person0'][0], _sc['person0'][1], 0.20, 0.20]
ROBOT_R = 0.33          # circle covering the chassis corners and wheels
dur, out = float(sys.argv[1]), sys.argv[2]
rclpy.init(); n = Node('robot_risk_run_logger')
S = {'risk': {}, 'ctrl': {}, 'gt': None, 'events': []}
n.create_subscription(String, '/robot_risk/risk_state', lambda m: S.__setitem__('risk', json.loads(m.data)), 10)
n.create_subscription(String, '/robot_risk/controller_state', lambda m: S.__setitem__('ctrl', json.loads(m.data)), 10)
n.create_subscription(String, '/robot_risk/demo_event', lambda m: S['events'].append(m.data), 10)
def gt(m):
    t = m.transforms[0]; q = t.transform.rotation
    S['gt'] = (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, t.transform.translation.x, t.transform.translation.y,
               math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))),
               math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
n.create_subscription(TFMessage, '/world/robot_risk_world/dynamic_pose/info', gt, 10)
def clearance(x, y, boxes):
    best = (9.0, '')
    for name, (bx, by, hx, hy) in boxes.items():
        dx = max(abs(x - bx) - hx, 0); dy = max(abs(y - by) - hy, 0)
        best = min(best, (math.hypot(dx, dy) - ROBOT_R, name))
    return best
f = open(out, 'w', newline=''); w = csv.writer(f)
w.writerow(['wall', 'sim_t', 'x', 'y', 'pitch', 'yaw', 'state', 'risk', 'nearest_in_path', 'objects',
            'mode', 'reason', 'v', 'w', 'clearance', 'closest', 'event'])
t0 = time.time(); last = 0; minc = (9, ''); summary = {}
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last < 0.2 or S['gt'] is None: continue
    last = time.time(); g = S['gt']; r = S['risk']; c = S['ctrl']
    S['stamp'] = r.get('stamp') or S.get('stamp')
    boxes = dict(BOXES); boxes['person'] = tuple(PERSON)
    cl = clearance(g[1], g[2], boxes); minc = min(minc, cl)
    ev = ''
    while S['events']:
        e = S['events'].pop(0)
        if e.startswith(('person_moved', 'person_pose')):
            _, px, py = e.split(':'); PERSON[0], PERSON[1] = float(px), float(py)
        if not e.startswith('person_pose'):
            ev = e
    objs = ';'.join(f"{o['label']}@{o['distance']}" for o in r.get('objects', []) if o['label'] != 'floor_edge')
    w.writerow([round(last - t0, 2), S.get('stamp') or 0, round(g[1], 3), round(g[2], 3), round(g[3], 2), round(g[4], 1),
                r.get('state'), r.get('risk'), r.get('nearest_in_path'), objs, c.get('mode'), c.get('reason'),
                c.get('v'), c.get('w'), round(cl[0], 3), cl[1], ev])
    f.flush()
    summary[r.get('state')] = summary.get(r.get('state'), 0) + 1
    if c.get('reason') == 'goal reached':
        break
f.flush()
print(f'final GT=({g[1]:.2f},{g[2]:.2f}) yaw={g[4]:.0f} sim_t={g[0]:.1f} min_clearance={minc[0]:.3f} m to {minc[1]} states={summary} sim_duration={(S.get("stamp") or 0) - next(float(x.split(",")[1]) for x in open(out).read().splitlines()[1:] if float(x.split(",")[1]) > 0):.1f}')
