#!/usr/bin/env python3
"""ROS 2 node: robot camera -> risk state.

Sub : /robot_risk/camera            sensor_msgs/Image (robot front camera ONLY)
Pub : /robot_risk/risk_state        std_msgs/String  (JSON, see README)
      /robot_risk/risk_score        std_msgs/Float32 (0-100)
      /robot_risk/detection_image   sensor_msgs/Image (annotated, rgb8)

Safety: if no frame for CAMERA_TIMEOUT s, the header stamp stops advancing, or
the frame is degenerate (uniform), the state becomes INVALID (risk 100).
"""
import json
import time
import sys
import os

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import risk_core as rc  # noqa: E402

CAMERA_TOPIC = '/robot_risk/camera'
CAMERA_TIMEOUT = 1.5      # s wall time without frames -> INVALID (sim runs < 1x real time)
STAMP_FREEZE = 2.0        # s wall time with a non-advancing header stamp -> INVALID


def to_rgb(msg):
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if ch is None:
        return None, f'unsupported encoding {msg.encoding}'
    raw = np.frombuffer(bytes(msg.data), np.uint8)
    if raw.size != msg.step * msg.height:
        return None, 'data size mismatch'
    img = raw.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == 'bgr8':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif msg.encoding == 'mono8':
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    return img, ''


class RobotRiskDetector(Node):
    def __init__(self):
        super().__init__('robot_risk_detector')
        self.est = rc.RiskEstimator()
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, CAMERA_TOPIC, self.on_image, qos)
        self.state_pub = self.create_publisher(String, '/robot_risk/risk_state', 10)
        self.score_pub = self.create_publisher(Float32, '/robot_risk/risk_score', 10)
        self.img_pub = self.create_publisher(Image, '/robot_risk/detection_image', 2)
        self.last_rx = None
        self.last_stamp = None
        self.last_stamp_change = None
        self.last_hash = None
        self.unchanged_since = None
        self.frames = 0
        self.proc_ms = 0.0
        self.timer = self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f'subscribed to {CAMERA_TOPIC} (robot camera)')

    def publish(self, res, stamp, extra):
        out = {k: v for k, v in res.items() if k != 'boundary'}
        out.update(extra)
        out['stamp'] = stamp
        out['wall_time'] = time.time()
        self.state_pub.publish(String(data=json.dumps(out)))
        self.score_pub.publish(Float32(data=float(res['risk'])))

    def invalid(self, reason, stamp=None):
        if getattr(self, '_last_invalid', None) != reason.split('(')[0]:
            self._last_invalid = reason.split('(')[0]
            self.get_logger().warn(f'INVALID: {reason}')
        res = {'state': rc.STATE_INVALID, 'risk': 100.0, 'reason': reason, 'objects': [],
               'sectors': [0.0] * rc.N_SECTORS, 'nearest_in_path': 0.0}
        self.publish(res, stamp, {'frames': self.frames})
        return res

    def watchdog(self):
        now = time.monotonic()
        if self.last_rx is None:
            self.invalid('waiting for first camera frame')
        elif now - self.last_rx > CAMERA_TIMEOUT:
            self.invalid(f'camera timeout ({now - self.last_rx:.1f}s)')

    def on_image(self, msg):
        now = time.monotonic()
        self.last_rx = now
        self.frames += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None or stamp != self.last_stamp:
            self.last_stamp = stamp
            self.last_stamp_change = now
        img, err = to_rgb(msg)
        if img is None:
            self.invalid(err, stamp)
            return
        if now - self.last_stamp_change > STAMP_FREEZE:
            res = self.invalid('stale frame (header stamp not advancing)', stamp)
        else:
            t0 = time.perf_counter()
            res = self.est.process(img)
            self.proc_ms = (time.perf_counter() - t0) * 1000.0
            h = hash(img[::8, ::8].tobytes())
            if h != self.last_hash:
                self.last_hash, self.unchanged_since = h, now
            self._last_invalid = None
            self.publish(res, stamp, {
                'frames': self.frames, 'proc_ms': round(self.proc_ms, 1),
                'frame_unchanged_s': round(now - self.unchanged_since, 1),
                'encoding': msg.encoding, 'size': [msg.width, msg.height]})
        if self.img_pub.get_subscription_count() > 0:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            vis = rc.draw(bgr, res, f'| {self.proc_ms:.0f} ms')
            vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            out = Image()
            out.header = msg.header
            out.height, out.width = vis.shape[:2]
            out.encoding = 'rgb8'
            out.step = out.width * 3
            out.data = vis.tobytes()
            self.img_pub.publish(out)
        if self.frames % 50 == 0:
            self.get_logger().info(f"state={res['state']} risk={res['risk']:.0f} "
                                   f"objs={len(res['objects'])} proc={self.proc_ms:.0f}ms")


def main():
    rclpy.init()
    node = RobotRiskDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
