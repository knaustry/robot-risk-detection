#!/usr/bin/env python3
"""Fault-injection tests (sim only).  Robot in a clear lane (y=-1.2):
  T4  normal driving with no obstacle in the path
  T9a detector crash (SIGKILL)        -> controller must stop the robot
  T9b detector restart                -> robot resumes
  T9c camera bridge stopped           -> detector INVALID -> robot stops
  T9d controller crash (SIGKILL)      -> cmd watchdog must stop the robot
Each fault: ground-truth speed before, time to standstill, distance coasted.
Writes logs/fault_tests.txt"""
import math, subprocess, time, json, threading
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

RS = '/workspace/risk_system'
OUT = open(f'{RS}/logs/' + __import__('os').environ.get('FAULT_LOG', 'fault_tests.txt'), 'w')


def log(*a):
    s = ' '.join(str(x) for x in a)
    print(s, flush=True)
    OUT.write(s + '\n'); OUT.flush()


def sh(cmd):
    return subprocess.run(['bash', '-c', f'. {RS}/common.sh; {cmd}'], capture_output=True, text=True).stdout.strip()


rclpy.init()
n = Node('robot_risk_fault_tests')
S = {'pose': None, 'hist': [], 'risk': {}, 'ctrl': {}}


def gt(m):
    t = m.transforms[0]
    p = (time.time(), t.transform.translation.x, t.transform.translation.y)
    S['pose'] = p
    S['hist'].append(p)
    S['hist'] = S['hist'][-400:]


n.create_subscription(TFMessage, '/world/robot_risk_world/dynamic_pose/info', gt, 10)
n.create_subscription(String, '/robot_risk/risk_state', lambda m: S.__setitem__('risk', json.loads(m.data)), 10)
n.create_subscription(String, '/robot_risk/controller_state', lambda m: S.__setitem__('ctrl', json.loads(m.data)), 10)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()


def speed(window=1.0):
    h = [p for p in S['hist'] if p[0] > time.time() - window]
    if len(h) < 2:
        return 0.0
    return math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2]) / (h[-1][0] - h[0][0])


def wait_still(timeout=15.0):
    t0 = time.time(); p0 = S['pose']
    while time.time() - t0 < timeout:
        time.sleep(0.25)
        if time.time() - t0 > 1.0 and speed(1.0) < 0.005:
            p1 = S['pose']
            return time.time() - t0, math.hypot(p1[1] - p0[1], p1[2] - p0[2])
    p1 = S['pose']
    return None, math.hypot(p1[1] - p0[1], p1[2] - p0[2])


def set_pose(name, x, y):
    subprocess.run(['ign', 'service', '-s', '/world/robot_risk_world/set_pose', '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
                    f'name: "{name}", position: {{x: {x}, y: {y}, z: 0}}, orientation: {{w: 1}}'], capture_output=True)


results = {}
sh('stop controller; stop scenario; stop recorder')
sh('bash $RS/start_nodes.sh --no-controller')
set_pose('standing_person', 2.0, -0.35)   # SDF pose (a demo may have moved it)
set_pose('risk_robot', -5.0, -1.2)   # lane: obstacle_1 edge 0.8 m, floor edge 1.3 m to the side
time.sleep(5)
import os
sh('start controller python3 robot_risk_safety_controller.py --ros-args -p goal_x:=9.0 ' + os.environ.get('CTRL_PARAMS', ''))

# T4 normal driving
time.sleep(10)
v = speed(2.0)
r = S['risk']; c = S['ctrl']
# judged on the commanded speed (GT speed per WALL second scales with RTF)
results['T4'] = float(c.get('v') or 0) >= float(os.environ.get('T4_MIN_V', '0.14')) and r.get('state') in ('CLEAR', 'OBSTACLE_FAR') and v > 0.01
log(f"T4 normal driving: GT speed {v:.3f} m/s (sim-time rate x RTF), state={r.get('state')} risk={r.get('risk')} "
    f"cmd v={c.get('v')} reason={c.get('reason')} -> {'PASS' if results['T4'] else 'FAIL'}")

# T9a detector crash
v0 = speed(1.0)
sh('kill -9 $(cat $RUN/detector.pid)')
t, d = wait_still()
c = S['ctrl']
results['T9a'] = t is not None
log(f"T9a detector SIGKILL at GT speed {v0:.3f} m/s: standstill after {t} s (wall), coasted {d:.3f} m, "
    f"controller reason='{c.get('reason')}' -> {'PASS' if results['T9a'] else 'FAIL'}")

# T9b restart detector -> resume
sh('rm -f $RUN/detector.pid; start detector python3 robot_risk_detector.py')
time.sleep(10)
v = speed(2.0)
results['T9b'] = v > 0.01 and not str(S['ctrl'].get('reason', '')).startswith('SAFETY')
log(f"T9b detector restarted: GT speed {v:.3f} m/s, reason='{S['ctrl'].get('reason')}' -> {'PASS' if results['T9b'] else 'FAIL'}")

# T9c camera bridge stop
v0 = speed(1.0)
sh('stop bridge_cam')
t, d = wait_still()
results['T9c'] = t is not None
log(f"T9c camera bridge stopped at GT speed {v0:.3f} m/s: standstill after {t} s, coasted {d:.3f} m, "
    f"risk reason='{S['risk'].get('reason')}', ctrl='{S['ctrl'].get('reason')}' -> {'PASS' if results['T9c'] else 'FAIL'}")
sh('start bridge_cam ros2 run ros_gz_bridge parameter_bridge "/robot_risk/camera@sensor_msgs/msg/Image[ignition.msgs.Image"')
time.sleep(10)
v = speed(2.0)
log(f"     camera bridge restarted: GT speed {v:.3f} m/s, state={S['risk'].get('state')}")

# T9d controller crash -> watchdog
v0 = speed(1.0)
sh('kill -9 $(cat $RUN/controller.pid)')
t, d = wait_still()
results['T9d'] = t is not None
wd = sh('tail -3 $LOG/watchdog.log')
log(f"T9d controller SIGKILL at GT speed {v0:.3f} m/s: standstill after {t} s, coasted {d:.3f} m -> "
    f"{'PASS' if results['T9d'] else 'FAIL'}; watchdog log: {wd[-160:]}")
sh('rm -f $RUN/controller.pid')
log('RESULTS', results)
set_pose('risk_robot', -5.0, 0.0)
