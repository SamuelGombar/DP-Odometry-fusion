# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/.rviz2/koleso_a_lidar_odom_3.rviz; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &

/usr/bin/gnome-terminal --tab -- bash -c "rviz2; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
# ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --exclude-regex "/drawer_camera/[0-9]+/.*"
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --topics /scan_merged; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf.yaml; exec bash" &

