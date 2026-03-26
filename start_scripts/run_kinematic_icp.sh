BAG_NAME="Ralf_7m"

/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/kinematic_icp.rviz; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher --ros-args -p use_sim_time:=true; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher --ros-args -p use_sim_time:=true; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch kinematic_icp online_node.launch.py lidar_topic:=/scan_merged_c use_2d_lidar:=true; exec bash" & #bag_filename:=/home/samuelg9/ros2_ws_host/recordings/bp/Ralf_7m

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/wheel_odom -p path_topic:=/wheel_odom_path -p use_sim_time:=true; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_to_path --ros-args -p odometry_topic:=/kinematic_icp/lidar_odometry -p path_topic:=/kinematic_icp/lidar_odometry_path -p use_sim_time:=true; exec bash" &

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -a -o nevimciicpkinide; exec bash" &

# sleep 8
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/${BAG_NAME} --clock --topics /scan_merged /scan_merged_filtered /amrapi/sensor/velocity /hw_layer/imu/sensor/data; exec bash" &