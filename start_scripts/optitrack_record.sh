OUTPUT_NAME="opti_2"
OUTPUT_PATH=/home/samuelg9/ros2_ws_host/recordings/optitrack/${OUTPUT_NAME}
RECORD=true


/usr/bin/gnome-terminal --tab -- bash -c "ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.101; exec bash" &
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
if [ "$RECORD" = true ]; then
    /usr/bin/gnome-terminal --tab -- bash -c "ros2 bag record -a -o ${OUTPUT_PATH}; exec bash" &
fi
sleep 3
/usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp teleop; exec bash" &


# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run uds_kobuki_ros uds_kobuki_ros; exec bash" &
# /usr/bin/gnome-terminal --tab -- bash -c "ros2 run robot_control_cpp teleop; exec bash" &

# 0.6 - 0.72 filtracia laser bodov kobuki

### POZOR, ABY FUNGOVALO SPRAVNE, NASTAV
# A ESTE ODOM_TO_BASE_LINK ODKOMENTUJ