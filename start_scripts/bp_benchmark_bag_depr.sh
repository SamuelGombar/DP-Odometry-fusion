# ros2 bag play candy_7m --exclude-topics "/hw_layer/addon_mission_state"

# genz icp and wheel fusion via EKF
# /usr/bin/gnome-terminal --tab -- bash -c "rviz2; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/genz_wheel_ekf_fusion_benchmark.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/candy_7m --topics /scan_merged /amrapi/sensor/velocity /hw_layer/imu/sensor/data; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/ground_truth_topic_candy_7m; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/candy_7m --exclude-topics "/hw_layer/addon_mission_state"; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf.yaml; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/odometry/filtered -p path_topic:=/odometry/path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp imu_publisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp groundtruth_republisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/ground_truth -p path_topic:=/ground_truth_path; exec bash" &
