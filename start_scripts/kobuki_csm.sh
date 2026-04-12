BAG_NAME="ncr_1"
RECORD=true

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/kobuki_csm.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher --ros-args -p scan_topic:=/scan -p wheel_odom_topic:=/odom -p use_point_to_line_distance:=0; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/fusion_odometry -p path_topic:=/fusion_odometry_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom_icp -p path_topic:=/odom_icp_path; exec bash" &

if [ "$RECORD" = true ]; then
  rm -rf /home/samuelg9/ros2_ws_host/recordings/output/kobuki/csm_${BAG_NAME}
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o /home/samuelg9/ros2_ws_host/recordings/output/kobuki/csm_${BAG_NAME} /scan /ground_truth_wrapper /imu /fusion_odometry /odom_icp /tf /tf_static /odom; exec bash" &
fi

sleep 3
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp pid_control; exec bash" &