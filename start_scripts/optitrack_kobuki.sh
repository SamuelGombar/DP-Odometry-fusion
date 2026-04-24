BAG_NAME="optitrack_6"
SUFFIX="_5_01"
RECORD=true

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
read -rp "Enter choice [1-3]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm" ;;
  2) ODOM_TYPE="genz_ekf" ;;
  3) ODOM_TYPE="kin" ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

OUTPUT="/home/samuelg9/ros2_ws_host/recordings/output/kobuki/${ODOM_TYPE}/${BAG_NAME}${SUFFIX}"

# RViz
case "$ODOM_TYPE" in
  csm)     RVIZ_CONFIG="kobuki_csm_optitrack" ;;
  genz_ekf) RVIZ_CONFIG="kobuki_genz_ekf_optitrack" ;;
  kin)     RVIZ_CONFIG="kobuki_kin_optitrack" ;;
esac
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/${RVIZ_CONFIG}.rviz --ros-args -p use_sim_time:=true; exec bash" &

# Wheel odom path (common)
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/odom_path -p use_sim_time:=true; exec bash" &
# OptiTrack ground truth path (common)
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/vrpn_mocap/RigidBody_002/pose -p path_topic:=/optitrack_path -p optitrack:=true -p use_sim_time:=true; exec bash" &

if [ "$ODOM_TYPE" = "csm" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "sleep 5 && ros2 bag play /home/samuelg9/ros2_ws_host/recordings/optitrack/${BAG_NAME} --exclude-topics /tf /tf_static --remap __node:=compr_player --remap /scan:=/scan_c --clock; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher --ros-args -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -r /scan_merged:=/scan_c -r /scan_merged_c:=/scan -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher --ros-args -p scan_topic:=/scan -p wheel_odom_topic:=/odom -p use_point_to_line_distance:=1 -p use_sim_time:=true -p optitrack:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/fusion_odometry -p path_topic:=/fusion_odometry_path -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom_icp -p path_topic:=/odom_icp_path -p use_sim_time:=true; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT} /scan /odom /imu /fusion_odometry /odom_icp /tf /tf_static /vrpn_mocap/RigidBody_002/pose; exec bash" &
  fi

elif [ "$ODOM_TYPE" = "genz_ekf" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "sleep 5 && ros2 bag play /home/samuelg9/ros2_ws_host/recordings/optitrack/${BAG_NAME} --exclude-topics /tf /tf_static --remap __node:=compr_player --remap /scan:=/scan_c --clock; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -r /scan_merged:=/scan_c -r /scan_merged_c:=/scan -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc --ros-args -p input_topic:=/scan -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic min_consecutive_observations:=5 voxel_size:=0.1; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path -p use_sim_time:=true; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT} /scan /odom /imu /genz/odometry /genz/local_map /genz/non_planar_points /genz/planar_points /odometry/filtered /tf /tf_static /vrpn_mocap/RigidBody_002/pose; exec bash" &
  fi

elif [ "$ODOM_TYPE" = "kin" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "sleep 5 && ros2 bag play /home/samuelg9/ros2_ws_host/recordings/optitrack/${BAG_NAME} --remap __node:=compr_player --remap /scan:=/scan_c --clock; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -r /scan_merged:=/scan_c -r /scan_merged_c:=/scan -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch kinematic_icp online_node.launch.py lidar_topic:=/scan use_2d_lidar:=true use_sim_time:=False; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/kinematic_icp/lidar_odometry -p path_topic:=/kinematic_icp/lidar_odometry_path; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT} /scan /odom /imu /kinematic_icp/lidar_odometry /kinematic_icp/frame /kinematic_icp/keypoints /kinematic_icp/local_map /tf /tf_static /vrpn_mocap/RigidBody_002/pose; exec bash" &
  fi
fi