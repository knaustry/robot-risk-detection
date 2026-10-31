#!/bin/bash
# Start detector + cmd watchdog (+ controller unless --no-controller).
. /workspace/risk_system/common.sh
start detector python3 robot_risk_detector.py
start watchdog python3 robot_risk_cmd_watchdog.py
[ "$1" = "--no-controller" ] || start controller python3 robot_risk_safety_controller.py ${CTRL_ARGS:-}
