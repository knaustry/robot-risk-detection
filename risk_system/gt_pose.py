#!/usr/bin/env python3
"""Ground-truth robot pose logger (test only): gt_pose.py <seconds> [csv]
Prints / writes sim pose of risk_robot from /world/robot_risk_world/dynamic_pose/info."""
import sys, math, time, rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
dur = float(sys.argv[1]); out = open(sys.argv[2], 'w') if len(sys.argv) > 2 else None
rclpy.init(); n = Node('robot_risk_gt_pose'); rows = []
def rpy(q):
    r = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)); p = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
    y = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)); return r, p, y
def cb(m):
    # ros_gz_bridge drops entity names; Gazebo lists the model (risk_robot)
    # first, then its links (chassis, wheels) - verified with `ign topic -e`.
    for t in m.transforms[:1]:
        if True:
            tr = t.transform.translation; r, p, y = rpy(t.transform.rotation)
            rows.append((t.header.stamp.sec + t.header.stamp.nanosec*1e-9, tr.x, tr.y, tr.z, math.degrees(r), math.degrees(p), math.degrees(y)))
n.create_subscription(TFMessage, '/world/robot_risk_world/dynamic_pose/info', cb, 10)
t0 = time.time()
while time.time() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.1)
if out:
    out.write('sim_t,x,y,z,roll_deg,pitch_deg,yaw_deg\n'); [out.write(','.join(f'{v:.4f}' for v in r) + '\n') for r in rows]
if rows:
    import statistics as st
    P = [r[5] for r in rows]; R = [r[4] for r in rows]; Z = [r[3] for r in rows]
    print(f'samples={len(rows)} first=({rows[0][1]:.3f},{rows[0][2]:.3f}) last=({rows[-1][1]:.3f},{rows[-1][2]:.3f},yaw {rows[-1][6]:.1f}) '
          f'pitch[min {min(P):.2f} max {max(P):.2f}] roll[min {min(R):.2f} max {max(R):.2f}] z[min {min(Z):.4f} max {max(Z):.4f}]')
else:
    print('no risk_robot pose samples')
