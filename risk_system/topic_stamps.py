#!/usr/bin/env python3
"""Print sim-time rate of Image topics: topic_stamps.py <topic>... (8 s wall)"""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n = Node('robot_risk_stamp_probe'); d = {t: [] for t in sys.argv[1:]}
for t in d:
    n.create_subscription(Image, t, lambda m, t=t: d[t].append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec*1e-9)), QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.time()
while time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
for t, v in d.items():
    if len(v) > 1:
        sim = v[-1][1] - v[0][1]; wall = v[-1][0] - v[0][0]
        print(f'{t}: {len(v)} msgs, wall {len(v)/wall:.1f} Hz, sim-time rate {(len(v)-1)/sim:.1f} Hz, RTF~{sim/wall:.2f}')
    else: print(t, 'no/too few msgs', len(v))
