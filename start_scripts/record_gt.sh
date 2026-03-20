ros2 bag record --topics /ground_truth -o /home/samuelg9/ros2_ws_host/recordings/gt/ground_truth_topic_candy_7m &
RECORD_PID=$!

ros2 run robot_control_cpp groundtruth_republisher &
REPRUB_PID=$!

sleep 3
ros2 bag play /home/samuelg9/ros2_ws_host/recordings/bp/candy_7m --exclude-topics "/hw_layer/addon_mission_state"

kill $RECORD_PID $REPRUB_PID
wait $RECORD_PID $REPRUB_PID 2>/dev/null
paplay /usr/share/sounds/freedesktop/stereo/complete.oga