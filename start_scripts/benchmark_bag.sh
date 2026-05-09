BAG_NAME="Ralf_4m"
SUFFIX="_4_0075_imu_orientation"    # "_0075_regul_0-0001_15"
RECORD=true

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
read -rp "Enter choice [1-3]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm" ;;
  2) ODOM_TYPE="genz" ;;
  3) ODOM_TYPE="kin" ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

OUTPUT_PATH="/home/samuelg9/ros2_ws_host/recordings/output/${ODOM_TYPE}/${BAG_NAME}${SUFFIX}"

case "$ODOM_TYPE" in
  csm)      RVIZ_CONFIG="csm_fusion_benchmark" ;;
  genz)     RVIZ_CONFIG="genz_wheel_ekf_fusion_benchmark" ;;
  kin)      RVIZ_CONFIG="kinematic_icp" ;;
esac

# RViz
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/${RVIZ_CONFIG}.rviz; exec bash" &

if [ "$ODOM_TYPE" = "csm" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher --ros-args -p is_kinematic:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/fusion_odometry -p path_topic:=/fusion_odometry_path; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odom_icp -p path_topic:=/odom_icp_path; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT_PATH} /ground_truth_wrapper /imu /fusion_odometry /scan_merged_c /odom_icp /tf /tf_static /wheel_odom; exec bash" &
  fi

elif [ "$ODOM_TYPE" = "genz" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf_bag.yaml; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT_PATH} /genz/local_map /genz/non_planar_points /genz/odometry /genz/planar_points /ground_truth_wrapper /imu /odometry/filtered /scan_merged_c /tf /tf_static /wheel_odom; exec bash" &
  fi

elif [ "$ODOM_TYPE" = "kin" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --clock --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; pkill -SIGINT -f 'ros2 bag record'; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher --ros-args -p use_sim_time:=true -p is_kinematic:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch kinematic_icp online_node.launch.py lidar_topic:=/scan_merged_c use_2d_lidar:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path -p use_sim_time:=true; exec bash" &
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/kinematic_icp/lidar_odometry -p path_topic:=/kinematic_icp/lidar_odometry_path -p use_sim_time:=true; exec bash" &
  if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o ${OUTPUT_PATH} /kinematic_icp/lidar_odometry /kinematic_icp/frame /kinematic_icp/keypoints /kinematic_icp/local_map /ground_truth_wrapper /imu /odometry/filtered /scan_merged_c /tf /tf_static /wheel_odom; exec bash" &
  fi
fi

# Ground truth (common)
if [ "$ODOM_TYPE" = "kin" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper --ros-args -p use_sim_time:=true; exec bash" &
else
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper; exec bash" &
fi

if [ "$ODOM_TYPE" = "kin" ]; then
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path -p use_sim_time:=true; exec bash" &
else
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &
fi

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_${BAG_NAME} --remap __node:=gt_player; exec bash" &

# sleep 6.3
# ros2 service call /gt_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 1.0}"