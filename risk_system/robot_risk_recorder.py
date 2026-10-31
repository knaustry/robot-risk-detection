#!/usr/bin/env python3
"""Record a synchronized 1280x720 demo video from LIVE topics.

One output frame is written for every external-view frame (Gazebo overview
camera, 10 Hz sim time), so the video runs in simulation time.  Each frame
combines:
  * /robot_risk/overview_camera   Gazebo external view (fixed camera in world)
  * /robot_risk/detection_image   robot onboard camera + detector overlay
  * /robot_risk/risk_state        state, risk score, detected objects
  * /robot_risk/controller_state  controller mode/reason, cmd v/w, odometry
Usage: robot_risk_recorder.py <out.mp4>   (Ctrl-C / SIGINT to finish)
"""
import json
import signal
import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

W, H = 1280, 720
try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
    JP_FONT = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 26, index=0)
except Exception:          # captions fall back to no Japanese text
    JP_FONT = None
_CAP_CACHE = {}


def jp_caption(text):
    """Render a Japanese caption strip (cached), BGR uint8 800x40."""
    if text in _CAP_CACHE:
        return _CAP_CACHE[text]
    im = PILImage.new('RGB', (800, 40), (20, 20, 20))
    if JP_FONT is not None:
        ImageDraw.Draw(im).text((12, 4), text, font=JP_FONT, fill=(255, 255, 255))
    arr = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    _CAP_CACHE[text] = arr
    return arr


def caption_for(state, ctrl, moved):
    """Japanese caption derived ONLY from the live detector / controller state."""
    mode = ctrl.get('mode', '')
    reason = str(ctrl.get('reason', ''))
    if reason == 'goal reached':
        return '目標地点に到着：停止'
    if state == 'INVALID' or 'camera invalid' in reason or 'timeout' in reason:
        return 'カメラ／検出の異常：安全側に停止'
    if mode == 'WAIT_PERSON':
        return '人が進路上：停止して待機'
    if mode == 'DETOUR':
        return '回避行動：障害物の横を通過'
    if 'obstacle: stop' in reason or 'detour planned' in reason:
        return '危険度が停止閾値(70)を超過：停止 → 回避経路を計画'
    if state == 'PERSON':
        return f"人を検知：徐行（{float(ctrl.get('v') or 0):.2f} m/s）"
    if state == 'APPROACHING':
        return '障害物に接近：リスクに応じて減速'
    if moved:
        return '人が退避：安全を確認して走行再開'
    if not ctrl:
        return 'シミュレーション開始：カメラとリスク検知を起動'
    return f"通常走行：前進（{float(ctrl.get('v') or 0):.2f} m/s）"
STATE_BGR = {'CLEAR': (90, 200, 90), 'OBSTACLE_FAR': (90, 200, 90), 'APPROACHING': (0, 200, 255),
             'PERSON': (255, 140, 0), 'STOP': (40, 40, 255), 'INVALID': (200, 0, 200)}


def img_from_msg(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
    a = a.reshape(m.height, m.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()


def put(img, txt, xy, scale=0.55, col=(240, 240, 240), th=1):
    cv2.putText(img, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)


class Recorder(Node):
    def __init__(self, out):
        super().__init__('robot_risk_recorder')
        q = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/robot_risk/overview_camera', self.on_overview, q)
        self.create_subscription(Image, '/robot_risk/detection_image', self.on_det, q)
        self.create_subscription(String, '/robot_risk/risk_state', self.on_risk, 10)
        self.create_subscription(String, '/robot_risk/controller_state', self.on_ctrl, 10)
        self.create_subscription(String, '/robot_risk/demo_event', self.on_event, 10)
        self.det = None
        self.det_stamp = None
        self.risk = {}
        self.ctrl = {}
        self.events = deque(maxlen=4)
        self.hist = deque(maxlen=400)       # (sim_t, risk, v, state)
        self.t0 = None
        self.out = out
        self.writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 10.0, (W, H))
        if not self.writer.isOpened():
            raise RuntimeError('cannot open video writer')
        self.frames = 0
        self.next_t = None
        self.waited = False
        self.resume_t = None
        self.get_logger().info(f'recording -> {out}')

    def on_det(self, m):
        self.det = img_from_msg(m)
        self.det_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_risk(self, m):
        self.risk = json.loads(m.data)

    def on_ctrl(self, m):
        self.ctrl = json.loads(m.data)

    def on_event(self, m):
        if m.data.startswith('person_pose'):
            return
        self.events.append((self.hist[-1][0] if self.hist else 0.0, m.data))

    def on_overview(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        st = t - self.t0
        risk = float(self.risk.get('risk') or 0.0)
        state = self.risk.get('state', '-')
        v = float(self.ctrl.get('v') or 0.0)
        w = float(self.ctrl.get('w') or 0.0)
        self.hist.append((st, risk, v, state))
        canvas = np.full((H, W, 3), 28, np.uint8)
        # external view
        ov = cv2.resize(img_from_msg(m), (800, 450))
        # "resuming" caption only for 4 s after leaving WAIT_PERSON
        if self.ctrl.get('mode') == 'WAIT_PERSON':
            self.waited = True
        elif self.waited:
            self.waited, self.resume_t = False, st
        moved = self.resume_t is not None and st - self.resume_t < 4.0
        ov[0:40, 0:800] = jp_caption(caption_for(state, self.ctrl, moved))
        put(ov, 'Gazebo external view (fixed world camera)', (10, 62), 0.5)
        put(ov, f'sim t = {st:6.1f} s', (650, 440), 0.5)
        canvas[0:450, 0:800] = ov
        # robot camera
        if self.det is not None:
            d = cv2.resize(self.det, (480, 360))
            lag = t - self.det_stamp if self.det_stamp else 0.0
            put(d, f'robot camera /robot_risk/camera  (age {lag:+.2f}s)', (8, 350), 0.45)
            canvas[0:360, 800:1280] = d
        else:
            put(canvas, 'robot camera: no frame', (820, 180), 0.6, (0, 0, 255))
        # info panel
        p = canvas[360:720, 800:1280]
        col = STATE_BGR.get(state, (200, 200, 200))
        cv2.rectangle(p, (10, 10), (470, 62), col, -1)
        put(p, f'{state}', (20, 48), 1.0, (255, 255, 255), 2)
        put(p, f'RISK {risk:5.1f}/100', (300, 48), 0.65, (255, 255, 255), 2)
        y = 88
        objs = [o for o in self.risk.get('objects', []) if o.get('label') != 'floor_edge']
        objs = sorted(objs, key=lambda o: -o.get('risk', 0))[:3]
        if not objs:
            put(p, 'detections: none', (14, y), 0.5); y += 24
        for o in objs:
            put(p, f"{o['label']:<8} ~{o['distance']:.1f} m  lat {o['lateral']:.1f} m  "
                   f"conf {o['confidence']:.2f}  r {o['risk']:.0f}", (14, y), 0.46); y += 22
        if self.risk.get('reason'):
            put(p, 'reason: ' + self.risk['reason'][:40], (14, y), 0.46, (200, 120, 255)); y += 22
        y = max(y, 170)
        put(p, f"controller: {self.ctrl.get('mode', '-')}", (14, y), 0.5, (255, 220, 120)); y += 22
        put(p, f"  {str(self.ctrl.get('reason', '-'))[:44]}", (14, y), 0.46, (255, 220, 120)); y += 24
        put(p, f'cmd_vel  v = {v:+.3f} m/s   w = {w:+.3f} rad/s', (14, y), 0.5); y += 24
        od = self.ctrl.get('pose_from_start')
        if od:
            put(p, f'odom from start x={od[0]:+.2f} y={od[1]:+.2f} yaw={np.degrees(od[2]):+.0f}deg', (14, y), 0.48); y += 24
        put(p, f"detector {self.risk.get('proc_ms', '-')} ms/frame", (14, y), 0.42, (170, 170, 170))
        # timeline graph
        g = canvas[450:720, 0:800]
        g[:] = (22, 22, 22)
        put(g, 'risk score (orange, 0-100) / cmd v (cyan, 0-0.4 m/s) / state band - last 40 s', (10, 20), 0.45)
        x0, x1, y0, y1 = 40, 785, 40, 235
        for thr, lab in ((30, 'slow 30'), (70, 'stop 70')):
            yy = int(y1 - (y1 - y0) * thr / 100)
            cv2.line(g, (x0, yy), (x1, yy), (70, 70, 70), 1)
            put(g, lab, (x1 - 70, yy - 4), 0.38, (150, 150, 150))
        if len(self.hist) > 1:
            tmax = self.hist[-1][0]
            tmin = tmax - 40
            pr, pv = [], []
            for (tt, rr, vv, ss) in self.hist:
                if tt < tmin:
                    continue
                xx = int(x0 + (x1 - x0) * (tt - tmin) / 40.0)
                pr.append((xx, int(y1 - (y1 - y0) * min(rr, 100) / 100)))
                pv.append((xx, int(y1 - (y1 - y0) * min(max(vv, 0), 0.4) / 0.4)))
                cv2.line(g, (xx, y1 + 8), (xx, y1 + 22), STATE_BGR.get(ss, (90, 90, 90)), 2)
            cv2.polylines(g, [np.array(pr, np.int32)], False, (0, 150, 255), 2)
            cv2.polylines(g, [np.array(pv, np.int32)], False, (255, 230, 0), 2)
        ey = 258
        for (et, txt) in list(self.events)[-1:]:
            if txt.startswith('person_moved'):
                txt = 'person steps aside (Gazebo set_pose)'
            elif txt == 'person_walk_start':
                txt = 'person starts walking (Gazebo set_pose)'
            elif txt == 'person_walk_end':
                txt = 'person finished crossing'
            put(g, f'scenario event @ {et:.1f}s: {txt}', (10, ey), 0.45, (120, 255, 255))
        # write frames on a fixed 10 Hz SIM-time grid (overview camera renders at
        # ~6 Hz under load, so a frame may be repeated; video time == sim time)
        if self.next_t is None:
            self.next_t = st
        while self.next_t <= st + 1e-6:
            self.writer.write(canvas)
            self.frames += 1
            self.next_t += 0.1

    def close(self):
        self.writer.release()
        self.get_logger().info(f'wrote {self.frames} frames to {self.out}')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/workspace/risk_system/videos/raw_demo.mp4'
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Recorder(out)

    def _raise(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
