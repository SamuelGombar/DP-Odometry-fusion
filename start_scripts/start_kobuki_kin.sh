BAG_NAME="1"
RECORD=false
OUTPUT_PATH="/home/samuelg9/ros2_ws_host/recordings/output/kobuki/"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/kobuki_kin.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch kinematic_icp online_node.launch.py lidar_topic:=/scan use_2d_lidar:=true; exec bash" &

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/kinematic_icp/lidar_odometry -p path_topic:=/kinematic_icp/lidar_odometry_path; exec bash" &

if [ "$RECORD" = true ]; then
  # rm -rf ${OUTPUT_PATH}/genz_ekf_${BAG_NAME}
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT_PATH}/kin_${BAG_NAME} /ground_truth_wrapper /imu /kinematic_icp/lidar_odometry /scan /tf /tf_static /odom; exec bash" &
fi

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_${BAG_NAME} --rate 0.1 --remap __node:=gt_player; exec bash" &

# sleep 10
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &

# sleep 5
# ros2 service call /gt_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 5.0}"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp pid_control; exec bash" &