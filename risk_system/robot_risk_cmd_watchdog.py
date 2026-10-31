#!/usr/bin/env python3
"""cmd_vel watchdog.

Ignition DiffDrive keeps executing the LAST received Twist forever. If the
controller dies (crash, kill -9) while the robot is moving, nothing would stop
it. This node watches /robot_risk/cmd_vel and, only when commands stop arriving
while the last command was non-zero, publishes zero velocity. It never sends a
non-zero command, so it does not compete with the safety controller.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TIMEOUT = 0.5


class CmdWatchdog(Node):
    def __init__(self):
        super().__init__('robot_risk_cmd_watchdog')
        self.pub = self.create_publisher(Twist, '/robot_risk/cmd_vel', 10)
        self.create_subscription(Twist, '/robot_risk/cmd_vel', self.on_cmd, 10)
        self.last = None
        self.moving = False
        self.fired = 0
        self.create_timer(0.1, self.tick)

    def on_cmd(self, msg):
        if self.fired and msg.linear.x == 0.0 and msg.angular.z == 0.0:
            return  # our own zero command echoed back
        self.last = time.monotonic()
        self.moving = abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4
        self.fired = 0

    def tick(self):
        if self.last is None or not self.moving:
            return
        if time.monotonic() - self.last > TIMEOUT and self.fired < 5:
            self.pub.publish(Twist())
            self.fired += 1
            if self.fired == 1:
                self.get_logger().warn('cmd_vel silent while moving -> zero velocity sent')
            if self.fired == 5:
                self.moving = False


def main():
    rclpy.init()
    n = CmdWatchdog()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
