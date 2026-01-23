#!/bin/bash

# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp teleop; exec bash" &


# Classic ICP and wheel fusion
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/.rviz2/koleso_a_lidar_odom.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp fusion; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp fusion_git_ekf; exec bash" &

# GenZ-ICP and wheel fusion
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/.rviz2/koleso_a_lidar_odom_2.rviz; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp scan_to_pc; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 launch genz_icp odometry.launch.py topic:=/pointcloud_topic; exec bash" &

