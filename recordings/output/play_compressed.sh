BAG_NAME="cons_scans/genz_ekf_Frodo_4m_1"

# ODOMETRY="csm/"
ODOMETRY="genz_ekf/"
# ODOMETRY="kin/" 

# ODOM_TOPIC="/fusion_odometry"
# ODOM_PATH_TOPIC="/fusion_odometry_path"

ODOM_TOPIC="/genz/odometry"
ODOM_PATH_TOPIC="/genz/trajectory"

# ODOM_TOPIC="/kinematic_icp/lidar_odometry"
# ODOM_PATH_TOPIC="/kinematic_icp/lidar_odometry_path"

# RVIZ="csm_fusion_benchmark"
RVIZ="genz_wheel_ekf_fusion_benchmark"
# RVIZ="kinematic_icp"

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/${RVIZ}.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/output/${ODOMETRY}${BAG_NAME} --remap __node:=compr_player; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=${ODOM_TOPIC} -p path_topic:=${ODOM_PATH_TOPIC}; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &

sleep 10
ros2 service call /compr_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 10.0}"