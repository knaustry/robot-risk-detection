#!/bin/bash
# One closed-loop slalom run. Usage: run_slalom.sh <label> [record]
# Tunables through env: GOAL TRIG WALK V_MAX V_DETOUR V_PERSON V_MIN V_PASS W_TURN W_MAX SIDE_LAT
. /workspace/risk_system/common.sh
export RISK_SCENE_JSON=$RS/scene_slalom.json
L=$1; REC=$2
GOAL=${GOAL:-11.0}; GOAL_Y=${GOAL_Y:--0.4}; TRIG=${TRIG:-5.0}; WALK=${WALK:-0.4}
for n in controller scenario recorder detector watchdog; do stop $n; done
rm -f $LOG/controller.log $LOG/scenario.log
printf 'risk_robot -5.0 0.0 0\nstanding_person 3.2 -2.2 1.5708\n' | $RS/pose_client >/dev/null
timeout 3 ros2 topic pub --times 3 -r 10 /robot_risk/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
start detector python3 robot_risk_detector.py; start watchdog python3 robot_risk_cmd_watchdog.py
sleep 6
[ "$REC" = record ] && { start recorder python3 robot_risk_recorder.py $RS/videos/raw_slalom_$L.mp4; sleep ${IDLE_START:-7}; }
start controller python3 robot_risk_safety_controller.py --ros-args -p goal_x:=$GOAL -p goal_y:=$GOAL_Y -p person_hold:=${HOLD:-1.0} -p v_max:=${V_MAX:-0.30} -p v_detour:=${V_DETOUR:-0.25} -p v_person:=${V_PERSON:-0.12} -p v_min_slow:=${V_MIN:-0.15} -p v_pass:=${V_PASS:-0.22} -p w_turn:=${W_TURN:-0.6} -p w_max:=${W_MAX:-0.7} -p person_side_lat:=${SIDE_LAT:-1.0} -p obst_depth:=${DEPTH:-1.0} -p use_memory:=true
start scenario python3 demo_walk_scenario.py $TRIG $WALK ${MAXT:-300}
timeout ${MAXT:-300} python3 $RS/run_logger.py ${MAXT:-300} $RS/logs/slalom_$L.csv | tee $RS/logs/slalom_$L.summary.txt
[ "$REC" = record ] && sleep ${IDLE_END:-6}
stop recorder; stop scenario; stop controller
timeout 3 ros2 topic pub --times 3 -r 10 /robot_risk/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
cp $LOG/controller.log $RS/logs/slalom_$L.controller.log; cp $LOG/scenario.log $RS/logs/slalom_$L.scenario.log
echo "PARAMS $L GOAL=$GOAL GOAL_Y=$GOAL_Y HOLD=${HOLD:-1.0} TRIG=$TRIG WALK=$WALK V_MAX=${V_MAX:-0.30} V_DETOUR=${V_DETOUR:-0.25} V_PERSON=${V_PERSON:-0.12} V_MIN=${V_MIN:-0.15} V_PASS=${V_PASS:-0.22} W_TURN=${W_TURN:-0.6} W_MAX=${W_MAX:-0.7} SIDE_LAT=${SIDE_LAT:-1.0} DEPTH=${DEPTH:-1.0} MEMORY=on" | tee -a $RS/logs/slalom_runs.txt
tail -1 $RS/logs/slalom_$L.summary.txt >> $RS/logs/slalom_runs.txt
