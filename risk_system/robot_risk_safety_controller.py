#!/usr/bin/env python3
"""ROS 2 node: risk state -> /robot_risk/cmd_vel (the ONLY normal publisher).

Behaviour (speeds are deliberately low; DiffDrive limit in the SDF is 0.4 m/s):
  CLEAR / OBSTACLE_FAR : drive toward the goal at V_MAX
  APPROACHING          : slow down in proportion to risk
  PERSON               : creep at V_PERSON
  STOP + person        : stop and wait (no avoidance around people)
  STOP + obstacle      : stop; plan a 2-waypoint detour (odom frame) around the
                         obstacle edge measured by the camera, follow it, then
                         resume the goal.  The camera can stop the robot at any time.
  INVALID / timeout    : stop (camera, detector or odometry missing/stale)
"""
import json
import math
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

V_MAX = 0.15          # m/s
V_DETOUR = 0.10
V_PERSON = 0.05
V_MIN_SLOW = 0.06
W_MAX = 0.35          # rad/s
ACC_LIN = 0.15        # m/s^2
ACC_ANG = 0.8         # rad/s^2
RISK_TIMEOUT = 1.5    # s (wall) without a risk_state -> stop (sim runs < 1x)
ODOM_TIMEOUT = 1.5
DETOUR_MARGIN = 0.65  # m, robot centre to obstacle edge while passing
OBST_DEPTH = 0.9      # m, assumed obstacle depth (not observable by the camera)
CAM_X = 0.23
WP_TOL = 0.15
CLEAR_R = 0.45        # robot circle radius 0.33 m + 0.12 m margin
NEAR_BLOCK = 1.25     # m, an in-path object this close always stops the robot
N_SECT = 15


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SafetyController(Node):
    def __init__(self):
        super().__init__('robot_risk_safety_controller')
        # goal is RELATIVE to the robot pose when the controller starts
        # (odometry is not reset when the scene is reset with set_pose)
        self.declare_parameter('goal_x', 7.5)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('v_max', V_MAX)
        # speed profile (defaults = original slow demo)
        self.declare_parameter('v_detour', V_DETOUR)
        self.declare_parameter('v_person', V_PERSON)
        self.declare_parameter('v_min_slow', V_MIN_SLOW)
        self.declare_parameter('obst_depth', OBST_DEPTH)  # assumed depth when passing
        self.declare_parameter('person_hold', 0.0)        # s (wall) person must stay clear before resuming
        self.declare_parameter('use_memory', False)       # remember detoured obstacles
        self.declare_parameter('v_pass', 0.06)          # speed while passing on geometric check
        self.declare_parameter('w_turn', 0.3)           # rad/s, turning in place
        self.declare_parameter('w_max', W_MAX)
        self.declare_parameter('person_side_lat', 99.0)  # person farther sideways than this: no creep
        self.goal_rel = (float(self.get_parameter('goal_x').value), float(self.get_parameter('goal_y').value))
        self.goal = None
        self.v_max = float(self.get_parameter('v_max').value)
        g = lambda k: float(self.get_parameter(k).value)
        self.v_detour, self.v_person, self.v_min_slow = g('v_detour'), g('v_person'), g('v_min_slow')
        self.v_pass, self.w_turn, self.w_max, self.person_side_lat = g('v_pass'), g('w_turn'), g('w_max'), g('person_side_lat')
        self.pub = self.create_publisher(Twist, '/robot_risk/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_risk/controller_state', 10)
        self.create_subscription(String, '/robot_risk/risk_state', self.on_risk, 10)
        self.create_subscription(Odometry, '/robot_risk/odom', self.on_odom, 10)
        self.risk = None
        self.risk_rx = None
        self.odom = None
        self.odom_rx = None
        self.mode = 'GOAL'
        self.wps = []
        self.obst_depth = float(self.get_parameter('obst_depth').value)
        self.use_memory = bool(self.get_parameter('use_memory').value)
        self.person_hold = float(self.get_parameter('person_hold').value)
        self.person_clear_since = None
        self.wait_since = None
        self.memory = []        # odom points of obstacles the camera may no longer see
        self.force_other_side = None
        self.last_side = 1
        self.stop_since = None
        self.v = 0.0
        self.w = 0.0
        self.done = False
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'goal (relative to start) = {self.goal_rel} v_max={self.v_max}')

    # ---------------------------------------------------------------- inputs
    def on_risk(self, msg):
        try:
            self.risk = json.loads(msg.data)
            self.risk_rx = time.monotonic()
        except ValueError:
            self.risk = None

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)
        self.odom_rx = time.monotonic()
        if self.goal is None:
            self.start = (p.x, p.y, yaw)
            c, s_ = math.cos(yaw), math.sin(yaw)
            gx, gy = self.goal_rel
            self.goal = (p.x + c * gx - s_ * gy, p.y + s_ * gx + c * gy)
            self.get_logger().info(f'start odom ({p.x:.2f},{p.y:.2f},{math.degrees(yaw):.0f}deg) '
                                   f'-> goal odom ({self.goal[0]:.2f},{self.goal[1]:.2f})')

    # ---------------------------------------------------------------- output
    def command(self, v, w, reason):
        dt = 0.1
        if v == 0.0 and w == 0.0 and reason.startswith('SAFETY'):
            self.v, self.w = 0.0, 0.0            # safety stops are immediate
        else:
            self.v += max(-ACC_LIN * dt * 2, min(ACC_LIN * dt, v - self.v))  # brake 2x faster
            self.w += max(-ACC_ANG * dt, min(ACC_ANG * dt, w - self.w))
            if v == 0.0 and abs(self.v) < 0.01:
                self.v = 0.0
            if w == 0.0 and abs(self.w) < 0.02:
                self.w = 0.0
        t = Twist()
        t.linear.x = float(self.v)
        t.angular.z = float(self.w)
        self.pub.publish(t)
        st = {'mode': self.mode, 'reason': reason, 'v': round(self.v, 3), 'w': round(self.w, 3),
              'risk_state': self.risk.get('state') if self.risk else None,
              'risk': self.risk.get('risk') if self.risk else None,
              'odom': [round(c, 3) for c in self.odom] if self.odom else None,
              'pose_from_start': self.rel_pose(),
              'goal': self.goal, 'waypoints': [[round(a, 2), round(b, 2)] for a, b in self.wps],
              'wall_time': time.time()}
        self.state_pub.publish(String(data=json.dumps(st)))

    # ---------------------------------------------------------------- helpers
    def rel_pose(self):
        """Odometry pose relative to the pose when this controller started."""
        if self.odom is None or getattr(self, 'start', None) is None:
            return None
        x0, y0, a0 = self.start
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        c, s = math.cos(-a0), math.sin(-a0)
        return [round(c * dx - s * dy, 3), round(s * dx + c * dy, 3), round(wrap(self.odom[2] - a0), 3)]

    def plan_detour(self, obj, sectors):
        """Two waypoints beside the blocking object's edge (odom frame)."""
        x, y, yaw = self.odom
        left_edge, right_edge = obj['lat_left'], obj['lat_right']
        shift_left = left_edge + DETOUR_MARGIN       # lateral target if passing on the left
        shift_right = right_edge - DETOUR_MARGIN     # ... on the right
        free_left = sum(sectors[:N_SECT // 2])
        free_right = sum(sectors[N_SECT // 2 + 1:])
        # smaller lateral shift wins; camera free space breaks near-ties
        cost_l = abs(shift_left) - 0.05 * free_left
        cost_r = abs(shift_right) - 0.05 * free_right
        lat = shift_left if cost_l <= cost_r else shift_right
        if self.force_other_side is not None:       # previous plan was blocked
            lat = shift_right if self.force_other_side > 0 else shift_left
            self.force_other_side = None
        self.last_side = 1 if lat > 0 else -1
        fwd = obj['distance'] + CAM_X
        pts = [(fwd - 0.2, lat), (fwd + self.obst_depth, lat)]
        c, s = math.cos(yaw), math.sin(yaw)
        self.wps = [(x + c * a - s * b, y + s * a + c * b) for a, b in pts]
        if self.use_memory:         # front edge and an assumed 0.6 m deep back edge
            for k in range(5):
                py = right_edge + (left_edge - right_edge) * k / 4.0
                for px in (fwd, fwd + 0.6):
                    self.memory.append((x + c * px - s * py, y + s * px + c * py))
        side = 'left' if lat > 0 else 'right'
        self.get_logger().info(f"detour {side}: obstacle ~{obj['distance']:.2f} m, edges "
                               f"[{left_edge:+.2f},{right_edge:+.2f}] -> wps {self.wps}")
        return side

    def segment_clear(self, tx, ty, objs):
        """True if the straight segment robot->(tx,ty) keeps ROBOT_R+margin from
        the front edge of every detected object (camera estimates, robot frame)."""
        x, y, yaw = self.odom
        c, s = math.cos(-yaw), math.sin(-yaw)
        ex, ey = c * (tx - x) - s * (ty - y), s * (tx - x) + c * (ty - y)   # target in robot frame
        L2 = ex * ex + ey * ey
        for o in objs:
            fx = o['distance'] + CAM_X
            for k in range(5):        # sample the visible front edge
                py = o['lat_right'] + (o['lat_left'] - o['lat_right']) * k / 4.0
                t = 0.0 if L2 < 1e-6 else max(0.0, min(1.0, (fx * ex + py * ey) / L2))
                if math.hypot(fx - t * ex, py - t * ey) < CLEAR_R:
                    return False, o
        return True, None

    def memory_blocks(self, tx, ty, horizon=1.0):
        """True if the next `horizon` m toward (tx,ty) pass a remembered obstacle."""
        x, y, _ = self.odom
        L = math.hypot(tx - x, ty - y)
        if L < 1e-3:
            return False
        ux, uy = (tx - x) / L, (ty - y) / L
        for mx, my in self.memory:
            t = max(0.0, min(horizon, (mx - x) * ux + (my - y) * uy))
            if math.hypot(mx - (x + t * ux), my - (y + t * uy)) < CLEAR_R + 0.05:
                return True
        return False

    def drive_to(self, tx, ty, v_cap, why):
        x, y, yaw = self.odom
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > 0.35:
            return self.command(0.0, math.copysign(self.w_turn, err), why + ' (turning)')
        w = max(-self.w_max, min(self.w_max, 1.5 * err))
        return self.command(v_cap, w, why)

    # ---------------------------------------------------------------- loop
    def tick(self):
        now = time.monotonic()
        if self.done:
            return self.command(0.0, 0.0, 'goal reached')
        if self.risk is None or now - self.risk_rx > RISK_TIMEOUT:
            return self.command(0.0, 0.0, 'SAFETY risk_state timeout')
        if self.odom is None or now - self.odom_rx > ODOM_TIMEOUT:
            return self.command(0.0, 0.0, 'SAFETY odom timeout')
        if time.time() - self.risk.get('wall_time', 0) > RISK_TIMEOUT:
            return self.command(0.0, 0.0, 'SAFETY risk_state stale')
        st = self.risk['state']
        risk = float(self.risk['risk'])
        sectors = self.risk.get('sectors') or [0.0] * N_SECT
        if st == 'INVALID':
            return self.command(0.0, 0.0, 'SAFETY camera invalid: ' + self.risk.get('reason', ''))
        x, y, yaw = self.odom
        gx, gy = self.goal
        if math.hypot(gx - x, gy - y) < 0.3:
            self.done = True
            self.get_logger().info(f'goal reached at odom ({x:.2f},{y:.2f})')
            return self.command(0.0, 0.0, 'goal reached')

        # drop waypoints already reached (before any STOP handling)
        while self.wps and math.hypot(self.wps[0][0] - x, self.wps[0][1] - y) < WP_TOL:
            self.wps.pop(0)
        if self.mode == 'DETOUR' and not self.wps:
            self.mode = 'GOAL'
            self.get_logger().info('detour complete -> goal')

        objs = self.risk.get('objects', [])
        in_path = [o for o in objs if o['in_path']]
        # a person blocks only if that person's own risk is high (not a far/false one
        # while something else is the reason for the STOP)
        person_block = any(o['label'] == 'person' and o['risk'] >= 60.0 for o in in_path)
        nearest = min(in_path, key=lambda o: o['distance']) if in_path else None

        # ---- STOP handling ------------------------------------------------
        if st == 'STOP':
            self.stop_since = self.stop_since or now
            if person_block:      # never plan around a person: stop and wait
                self.mode = 'WAIT_PERSON'
                return self.command(0.0, 0.0, 'SAFETY person in path: stop & wait')
            # plan around the object that causes the STOP (highest risk)
            cause = max(objs, key=lambda o: o['risk']) if objs else None
            if self.mode == 'GOAL' and cause is not None:
                self.command(0.0, 0.0, 'SAFETY obstacle: stop')
                if abs(self.v) < 0.005:
                    side = self.plan_detour(cause, sectors)
                    self.mode = 'DETOUR'
                    self.stop_since = now
                    return self.command(0.0, 0.0, f'SAFETY obstacle: detour planned ({side})')
                return
            if self.mode == 'DETOUR' and self.wps:
                # The camera STOP is evaluated straight ahead; while passing a
                # planned obstacle the controller instead checks the segment to
                # the next waypoint against all detected object edges.
                tx, ty = self.wps[0]
                err = wrap(math.atan2(ty - y, tx - x) - yaw)
                if abs(err) > 0.2:
                    return self.command(0.0, math.copysign(self.w_turn, err), 'detour: turn toward waypoint')
                ok, blk = self.segment_clear(tx, ty, objs)
                too_near = nearest is not None and nearest['distance'] < NEAR_BLOCK \
                    and nearest['lateral'] < 0.30
                if ok and not too_near:
                    return self.drive_to(tx, ty, self.v_pass, 'detour: segment clear (geometric check)')
                if now - self.stop_since > 10.0:
                    self.mode, self.wps, self.stop_since = 'GOAL', [], None
                    self.force_other_side = self.last_side
                    return self.command(0.0, 0.0, 'SAFETY detour blocked 10s: re-plan')
                return self.command(0.0, 0.0, 'SAFETY detour segment blocked: stop')
            return self.command(0.0, 0.0, 'SAFETY stop')
        self.stop_since = None

        if self.mode == 'WAIT_PERSON' and self.person_hold > 0.0:
            # resume only after no person has been near the path for person_hold s
            # (up close a person may be labelled "obstacle", so any nearby object counts;
            #  12 s limit so that a static object cannot block forever)
            near = any((o['label'] == 'person' and o['lateral'] < 1.0 and o['distance'] < 4.5) or
                       (o['label'] != 'floor_edge' and o['lateral'] < 0.6 and o['distance'] < 3.2) for o in objs)
            self.wait_since = self.wait_since or now
            if near:
                self.person_clear_since = None
            else:
                self.person_clear_since = self.person_clear_since or now
            if now - self.wait_since < 12.0 and (near or now - self.person_clear_since < self.person_hold):
                return self.command(0.0, 0.0, 'SAFETY waiting for person to clear')
            self.person_clear_since = None
            self.wait_since = None

        if self.mode == 'WAIT_PERSON':
            self.mode = 'GOAL'
            self.get_logger().info('person cleared -> resume')

        # ---- speed from risk ---------------------------------------------
        if st == 'APPROACHING':
            v = self.v_max * max(self.v_min_slow / self.v_max, min(1.0, 1.0 - (risk - 30.0) / 50.0))
            why = f'slow (risk {risk:.0f})'
        elif st == 'PERSON' and any(o['label'] == 'person' and o['lateral'] < self.person_side_lat for o in objs):
            v = min(self.v_max, self.v_person)
            why = 'person detected: creep'
        else:
            v = self.v_max
            why = 'normal'

        # ---- detour following -----------------------------------------------
        if self.mode == 'DETOUR':
            while self.wps and math.hypot(self.wps[0][0] - x, self.wps[0][1] - y) < WP_TOL:
                self.wps.pop(0)
            if not self.wps:
                self.mode = 'GOAL'
                self.get_logger().info('detour complete -> goal')
            else:
                tx, ty = self.wps[0]
                return self.drive_to(tx, ty, min(v, self.v_detour), f'detour wp{2 - len(self.wps) + 1}: {why}')

        # ---- goal ------------------------------------------------------------
        if self.memory_blocks(gx, gy):
            # goal direction would swing back into an obstacle now outside the view
            return self.command(min(v, self.v_detour), 0.0, 'hold heading past known obstacle')
        return self.drive_to(gx, gy, v, why)

    def stop_now(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.05)


def main():
    # our own signal handling, so the context is still valid for the final stop
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SafetyController()

    def _raise(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_now()
        node.get_logger().info('controller exit: zero velocity sent')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
