#!/bin/bash
# Start Gazebo (production world, headless server) + ONE bridge process.
. /workspace/risk_system/common.sh
if ps -eo args | grep -q "^ruby .*ign gazebo" && ! alive gazebo; then
  echo "ERROR: a robot_risk Gazebo not managed by these scripts is running:"; ps -eo pid,args | grep "ruby .*ign gazebo" | grep -v grep; exit 1; fi
if alive gazebo && ! tr "\0" " " < /proc/$(cat $RUN/gazebo.pid)/cmdline | grep -q "$WORLD"; then
  echo "ERROR: managed Gazebo runs another world; run stop_all.sh --all first"; exit 1; fi
echo "world: $WORLD"
start gazebo ign gazebo -s -r -v 3 $WORLD
sleep 6
# robot camera in its own bridge process (fault isolation / camera-loss test)
start bridge_cam ros2 run ros_gz_bridge parameter_bridge \
  "/robot_risk/camera@sensor_msgs/msg/Image[ignition.msgs.Image"
start bridge ros2 run ros_gz_bridge parameter_bridge \
  "/robot_risk/overview_camera@sensor_msgs/msg/Image[ignition.msgs.Image" \
  "/robot_risk/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/robot_risk/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
# ground-truth pose, used ONLY for test logging (never by detector/controller)
start posebridge ros2 run ros_gz_bridge parameter_bridge \
  "/world/robot_risk_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
