BAG_NAME=Frodo_4m_4_0075
KOBUKI=false

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
read -rp "Enter choice [1-3]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm";     ODOM_TOPIC="/fusion_odometry" ;;  #/odom_icp
  2) ODOM_TYPE="genz_ekf"; ODOM_TOPIC="/odometry/filtered" ;;   #/genz/odometry
  3) ODOM_TYPE="kin";     ODOM_TOPIC="/kinematic_icp/lidar_odometry" ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

source /home/samuelg9/ros2_ws_host/recordings/venv/bin/activate
if [ "$KOBUKI" = "true" ]; then
  python3 /home/samuelg9/ros2_ws_host/trajectory_eval.py \
    /home/samuelg9/ros2_ws_host/recordings/output/kobuki/${ODOM_TYPE}/${BAG_NAME} \
    --odom-topic ${ODOM_TOPIC} \
    --align \
    --kobuki \
    --output-csv /home/samuelg9/ros2_ws_host/results.csv \
    --interpolate-gt 0.5 
else
  python3 /home/samuelg9/ros2_ws_host/trajectory_eval.py \
    /home/samuelg9/ros2_ws_host/recordings/output/${ODOM_TYPE}/${BAG_NAME} \
    --odom-topic ${ODOM_TOPIC} \
    --align \
    --output-csv /home/samuelg9/ros2_ws_host/results.csv
fi
python3 /home/samuelg9/ros2_ws_host/plot_trajectory_eval.py /home/samuelg9/ros2_ws_host/results.csv \
  --separate \
  --match-lines
  # --rotate -10

