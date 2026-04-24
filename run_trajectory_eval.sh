BAG_NAME=optitrack_6_1
KOBUKI=true

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
read -rp "Enter choice [1-3]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm";     ODOM_TOPIC="/fusion_odometry" ;;
  2) ODOM_TYPE="genz_ekf"; ODOM_TOPIC="/genz/odometry" ;;
  3) ODOM_TYPE="kin";     ODOM_TOPIC="/kinematic_icp/lidar_odometry" ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

source /home/samuelg9/ros2_ws_host/recordings/venv/bin/activate
if [ "$KOBUKI" = "true" ]; then
  python3 trajectory_eval.py \
    /home/samuelg9/ros2_ws_host/recordings/output/kobuki/${ODOM_TYPE}/${BAG_NAME} \
    --odom-topic ${ODOM_TOPIC} \
    --interpolate-gt 0.5 \
    --kobuki \
    --align \
    --output-csv results.csv
else
  python3 trajectory_eval.py \
    /home/samuelg9/ros2_ws_host/recordings/output/${ODOM_TYPE}/${BAG_NAME} \
    --odom-topic ${ODOM_TOPIC} \
    --align \
    --output-csv results.csv
fi
python3 plot_trajectory_eval.py results.csv --separate


