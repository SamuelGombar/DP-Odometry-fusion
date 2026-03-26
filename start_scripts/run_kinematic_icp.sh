BAG_NAME="Frodo_7m"
RECORD=true

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/kinematic_icp.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --clock --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; pkill -SIGINT -f 'ros2 bag record'; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher --ros-args -p use_sim_time:=true; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -p use_sim_time:=true; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch kinematic_icp online_node.launch.py lidar_topic:=/scan_merged_c use_2d_lidar:=true; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path -p use_sim_time:=true; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/kinematic_icp/lidar_odometry -p path_topic:=/kinematic_icp/lidar_odometry_path -p use_sim_time:=true; exec bash" &

if [ "$RECORD" = true ]; then
  rm -rf /home/samuelg9/ros2_ws_host/recordings/output/kin/kin_${BAG_NAME}
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o /home/samuelg9/ros2_ws_host/recordings/output/kin/kin_${BAG_NAME} /kinematic_icp/lidar_odometry /kinematic_icp/frame /kinematic_icp/keypoints /kinematic_icp/local_map /ground_truth_wrapper /imu /odometry/filtered /scan_merged_c /tf /tf_static /wheel_odom; exec bash" &
fi

sleep 4
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper --ros-args -p use_sim_time:=true; exec bash" &
sleep 2
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_${BAG_NAME} --rate 0.1 --remap __node:=gt_player; exec bash" &

# sleep 10
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path -p use_sim_time:=true; exec bash" &

sleep 5
ros2 service call /gt_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 5.0}"
