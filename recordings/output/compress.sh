BAG_NAME="imperfect/genz_ekf_Candy_4m"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/output/${BAG_NAME}; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -o /home/samuelg9/ros2_ws_host/recordings/output/${BAG_NAME}_compressed /genz/local_map /genz/non_planar_points /genz/odometry /genz/planar_points /ground_truth_wrapper /imu /odometry/filtered /scan_merged_c /tf /tf_static /wheel_odom; exec bash" &



# /genz/local_map
# /genz/non_planar_points
# /genz/odometry
# /genz/planar_points
# /ground_truth_wrapper
# /imu
# /odometry/filtered
# /scan_merged_c
# /tf
# /tf_static
# /wheel_odom

# /rosout
# /events/read_split
# /genz/trajectory
# /ground_truth
# /ground_truth_path
# /odometry/path
# /parameter_events
# /pointcloud_topic
# /scan_merged
# /wheel_odom_path