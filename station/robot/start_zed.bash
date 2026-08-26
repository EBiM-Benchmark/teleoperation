#!/usr/bin/env bash

set -Eeuo pipefail

ws_dir="${TMR_WS:-$HOME/tams_ws}"
params_file="$ws_dir/config/head_camera_zed_params.yaml"
dds_file="$ws_dir/config/fastdds.xml"

# Domain 0, like everything else: start_robot.bash, start_teleop.bash,
# configs/tmr_laptop_env.sh and the LABS docker-compose all use 0. This defaulted to
# 100, which put the ZED on its own domain where nothing else could see it - a single
# `ros2 bag record` sees exactly one domain, so the head camera could never be recorded
# alongside the arms. Override with TMR_ROS_DOMAIN_ID if you really need to isolate it.
export ROS_DOMAIN_ID="${TMR_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$dds_file"
export FASTDDS_DEFAULT_PROFILES_FILE="$dds_file"

set +u
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source "$ws_dir/install/local_setup.bash"
set -u

exec ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm \
  camera_name:=head_camera \
  serial_number:=13024307 \
  ros_params_override_path:="$params_file" \
  publish_urdf:=false \
  publish_tf:=false \
  publish_map_tf:=false \
  publish_imu_tf:=false \
  node_log_type:=screen
