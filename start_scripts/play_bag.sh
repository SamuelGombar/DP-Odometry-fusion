BAG_NAME="Ralf_4m_4_0075_imu_orientation"
SUBFOLDER=genz
KOBUKI=false

BAG_PATH="/home/samuelg9/ros2_ws_host/recordings/output/genz/${BAG_NAME}"

# ODOM_TOPIC="/fusion_odometry"
# ODOM_PATH_TOPIC="/fusion_odometry_path"
ODOM_TOPIC="/genz/odometry"
ODOM_PATH_TOPIC="/genz/trajectory"
# ODOM_TOPIC="/kinematic_icp/lidar_odometry"
# ODOM_PATH_TOPIC="/kinematic_icp/lidar_odometry_path"

# ODOM_TOPIC="/odom_icp"
# ODOM_PATH_TOPIC="/odom_icp_path"

# RVIZ="csm_fusion_benchmark"
RVIZ="genz_wheel_ekf_fusion_benchmark"
# RVIZ="kinematic_icp"

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/${RVIZ}.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play ${BAG_PATH} --remap __node:=compr_player; exec bash" &
if [[ "${KOBUKI}" == "true" ]]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/wheel_odom_path; exec bash" &
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/optitrack -p path_topic:=/ground_truth_path; exec bash" &
else
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom -p path_topic:=/wheel_odom_path; exec bash" &
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &
fi
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=${ODOM_TOPIC} -p path_topic:=${ODOM_PATH_TOPIC}; exec bash" &

if [[ "${SUBFOLDER}" == "csm" ]]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom_icp -p path_topic:=/odom_icp_path; exec bash" &
elif [[ "${SUBFOLDER}" == "genz" ]]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &
fi

# sleep 8
ros2 service call /compr_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 1.0}"
