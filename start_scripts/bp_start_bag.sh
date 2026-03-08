# ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --exclude-regex "/drawer_camera/[0-9]+/.*"

# povodny stav
# /usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/default_bp.rviz; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --topics /scan_merged_filtered /amrapi/sensor/velocity /amrapi/sensor/pose; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp odom_republisher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_path; exec bash" &

# fuzia stuchlo (odometria je z csm, nie z rosbagu)
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/csm_bp.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --topics /scan_merged_filtered /amrapi/sensor/velocity; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_path; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher; exec bash" &

# genz icp and wheel fusion via EKF
# /usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/ros2_ws_host/rviz/genz_wheel_ekf_fusion_bp.rviz; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/brightpick_degen --topics /scan_merged_filtered /amrapi/sensor/velocity; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_publisher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp wheel_odom_path; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_republisher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch robot_localization ekf.launch.py params_file:=/home/samuelg9/ros2_ws_host/src/robot_localization/params/ekf.yaml; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp ekf_path; exec bash" &
