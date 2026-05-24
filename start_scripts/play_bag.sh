
#---------------CONFIGURATION---------------

WORKSPACE_PATH=~/ros2_ws_host
BAG_NAME="Tetragon_7m"
KOBUKI=false

#-------------------------------------------

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
echo "  4) Pure CSM"
read -rp "Enter choice [1-4]: " ODOM_CHOICE

if [[ "${KOBUKI}" == "false" && "${ODOM_CHOICE}" == "1" ]]; then
  echo "CSM is not recorded for Autopicker. Exiting."
  exit 1
fi

case "$ODOM_CHOICE" in
  1)
    BAG_SUBFOLDER="csm"
    ODOM_TYPE="csm"
    ODOM_TOPIC="/fusion_odometry"
    ODOM_PATH_TOPIC="/fusion_odometry_path"
    RVIZ="csm_fusion_benchmark"
    ;;
  2)
    BAG_SUBFOLDER="genz"
    ODOM_TYPE="genz"
    ODOM_TOPIC="/odometry/filtered"
    ODOM_PATH_TOPIC="/odometry/path"
    RVIZ="genz_wheel_ekf_fusion_benchmark"
    ;;
  3)
    BAG_SUBFOLDER="kin"
    ODOM_TYPE="kin"
    ODOM_TOPIC="/kinematic_icp/lidar_odometry"
    ODOM_PATH_TOPIC="/kinematic_icp/lidar_odometry_path"
    RVIZ="kinematic_icp"
    ;;
  4)
    BAG_SUBFOLDER="csm"
    ODOM_TYPE="csm"
    ODOM_TOPIC="/odom_icp"
    ODOM_PATH_TOPIC="/odom_icp_path"
    RVIZ="csm_fusion_benchmark"
    ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

if [[ "${KOBUKI}" == "true" ]]; then
  BAG_PATH="/home/samuelg9/ros2_ws_host/recordings/output/kobuki/${BAG_SUBFOLDER}/${BAG_NAME}"
else
  BAG_PATH="/home/samuelg9/ros2_ws_host/recordings/output/autopicker/${BAG_SUBFOLDER}/${BAG_NAME}"
fi

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
if [[ "${ODOM_TYPE}" == "genz" ]]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/genz/odometry -p path_topic:=/genz/odometry_path; exec bash" &
fi

# sleep 8
ros2 service call /compr_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 10.0}"
