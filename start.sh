#!/bin/bash

/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "rviz2 -d /home/samuelg9/.rviz2/koleso_a_lidar_odom.rviz; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run ros2_laser_scan_matcher laser_scan_matcher; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp teleop; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp fusion; exec bash" &