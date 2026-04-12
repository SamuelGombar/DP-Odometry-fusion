BAG_NAME="kobuki/csm_ncr_1"

ODOM_TOPIC="fusion_odometry"
ODOM_PATH_TOPIC="fusion_odometry_path"

RVIZ="kobuki_csm"

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/${RVIZ}.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/output/${BAG_NAME} --remap __node:=compr_player; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/odom_path; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/${ODOM_TOPIC} -p path_topic:=/${ODOM_PATH_TOPIC}; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom_icp -p path_topic:=/odom_icp_path; exec bash" &

# sleep 10
# ros2 service call /compr_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 100.0}"