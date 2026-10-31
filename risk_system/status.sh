#!/bin/bash
. /workspace/risk_system/common.sh
for n in gazebo bridge bridge_cam posebridge detector watchdog controller recorder; do alive $n && echo "$n: running ($(cat $RUN/$n.pid))" || echo "$n: -"; done
echo "--- cmd_vel publishers:"; ros2 topic info -v /robot_risk/cmd_vel --no-daemon --spin-time 3 | grep -E "Node name|Publisher count"
echo "--- gazebo servers:"; pgrep -af "ign gazebo" || echo none
