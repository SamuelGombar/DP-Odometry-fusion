BAG_NAME="Frodo_4m"

/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record --topics /ground_truth -o /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_"$BAG_NAME"; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp groundtruth_republisher; exec bash" &

sleep 3
/usr/bin/gnome-terminal --tab -- bash -c "ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/"$BAG_NAME" --topics /tf /tf_static; paplay /usr/share/sounds/freedesktop/stereo/complete.oga; exec bash" &





# BAG_NAME="Ralf_7m"

# ros2 bag record --topics /ground_truth -o /home/samuelg9/ros2_ws_host/recordings/gt/gt_topic_"$BAG_NAME" &
# RECORD_PID=$!

# ros2 run robot_control_cpp groundtruth_republisher &
# REPRUB_PID=$!

# sleep 3
# ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/"$BAG_NAME" --topics /tf /tf_static
# kill $RECORD_PID $REPRUB_PID
# wait $RECORD_PID $REPRUB_PID 2>/dev/null
# paplay /usr/share/sounds/freedesktop/stereo/complete.oga