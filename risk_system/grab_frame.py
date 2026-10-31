#!/usr/bin/env python3
"""Save one frame from an Image topic: grab_frame.py <topic> <out.png>"""
import sys, time, rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node('robot_risk_grab_frame'); got = []
n.create_subscription(Image, sys.argv[1], lambda m: got.append(m), 1)
t0 = time.time()
while not got and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.2)
m = got[-1]; a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height, m.width, 3)
cv2.imwrite(sys.argv[2], cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a)
print('saved', sys.argv[2], m.width, m.height, m.encoding, m.header.stamp.sec)
