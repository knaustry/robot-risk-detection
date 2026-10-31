# shared helpers (run INSIDE the pai_ros2 container)
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
RS=/workspace/risk_system
RUN=$RS/run; LOG=$RS/logs; mkdir -p $RUN $LOG $RS/videos
# world file: override with WORLD=/workspace/robot_risk_office.sdf (office scene)
WORLD=${WORLD:-/workspace/robot_risk_world.sdf}
alive(){ [ -f "$RUN/$1.pid" ] && kill -0 "$(cat $RUN/$1.pid)" 2>/dev/null; }
start(){ # start <name> <command...>   (refuses duplicates)
  local n=$1; shift
  if alive $n; then echo "[skip] $n already running (pid $(cat $RUN/$n.pid))"; return 0; fi
  setsid nohup bash -c "source /opt/ros/humble/setup.bash; cd $RS; exec $*" > $LOG/$n.log 2>&1 < /dev/null &
  echo $! > $RUN/$n.pid; echo "[start] $n pid $!"
}
stop(){ # stop <name>  (only processes started by these scripts)
  local n=$1
  # signal the whole process group (setsid): also stops children of `ros2 run`
  if alive $n; then local p=$(cat $RUN/$n.pid); kill -INT -- -$p 2>/dev/null || kill -INT $p
    for i in $(seq 1 20); do pgrep -g $p >/dev/null || break; sleep 0.25; done
    pgrep -g $p >/dev/null && kill -TERM -- -$p 2>/dev/null; echo "[stop] $n ($p)"; fi
  rm -f $RUN/$n.pid
}
