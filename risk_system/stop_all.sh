#!/bin/bash
# Stop everything started by start_*.sh. Sends zero velocity first.
. /workspace/risk_system/common.sh
stop recorder; stop controller
timeout 5 ros2 topic pub --times 3 -r 10 /robot_risk/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
stop watchdog; stop detector
[ "$1" = "--all" ] && { stop posebridge; stop bridge; stop bridge_cam; stop gazebo; }
true
