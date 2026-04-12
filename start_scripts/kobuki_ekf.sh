BAG_NAME="1"
RECORD=false
OUTPUT_PATH="/home/samuelg9/ros2_ws_host/recordings/output/kobuki/"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/kobuki.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc --ros-args -p input_topic:=/scan; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf_kobuki.yaml; exec bash" &

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &

if [ "$RECORD" = true ]; then
  # rm -rf ${OUTPUT_PATH}/genz_ekf_${BAG_NAME}
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT_PATH}/genz_ekf_${BAG_NAME} /genz/local_map /genz/non_planar_points /genz/odometry /genz/planar_points /ground_truth_wrapper /imu /odometry/filtered /scan /tf /tf_static /odom; exec bash" &
fi

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_${BAG_NAME} --rate 0.1 --remap __node:=gt_player; exec bash" &

# sleep 10
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &

# sleep 5
# ros2 service call /gt_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 5.0}"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp pid_control; exec bash" &