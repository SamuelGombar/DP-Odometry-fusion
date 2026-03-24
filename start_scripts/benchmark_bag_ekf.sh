BAG_NAME="Frodo_4m"
RECORD=true

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/genz_wheel_ekf_fusion_benchmark.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf.yaml; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &

if [ "$RECORD" = true ]; then
  rm -rf /home/samuelg9/ros2_ws_host/recordings/output/genz_ekf_${BAG_NAME}
  /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o /home/samuelg9/ros2_ws_host/recordings/output/genz_ekf_${BAG_NAME} /genz/local_map /genz/non_planar_points /genz/odometry /genz/planar_points /ground_truth_wrapper /imu /odometry/filtered /scan_merged_c /tf /tf_static /wheel_odom; exec bash" &
fi

sleep 4
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp gt_wrapper; exec bash" &
sleep 2
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_${BAG_NAME} --rate 0.1 --remap __node:=gt_player; exec bash" &

# sleep 10
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth_wrapper -p path_topic:=/ground_truth_path; exec bash" &

sleep 5
ros2 service call /gt_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: 5.0}"