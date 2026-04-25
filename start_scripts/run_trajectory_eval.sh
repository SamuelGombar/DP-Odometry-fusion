BAG_NAME=Candy_4m_0075_1_4
KOBUKI=false
SAVE_DIR=/home/samuelg9/Documents/Skola/DP/latex/img

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
read -rp "Enter choice [1-3]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm";      ODOM_TOPIC="/fusion_odometry";              EST_LABEL="CSM" ;;               #/odom_icp
  2) ODOM_TYPE="genz"; ODOM_TOPIC="/odometry/filtered";           EST_LABEL="GenZ-ICP" ;;             #/genz/odometry
  3) ODOM_TYPE="kin";      ODOM_TOPIC="/kinematic_icp/lidar_odometry"; EST_LABEL="Kinematic ICP" ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

read -rp "Enter trajectory plot title: " TRAJ_TITLE

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
    --output-csv /home/samuelg9/ros2_ws_host/results.csv \
    --align \
    --mode temporal \
      
fi
mkdir -p "$SAVE_DIR"
python3 /home/samuelg9/ros2_ws_host/plot_trajectory_eval.py /home/samuelg9/ros2_ws_host/results.csv \
  --separate \
  --match-lines \
  --rotate -180 \
  --traj-title "$TRAJ_TITLE" \
  --est-label "$EST_LABEL" \
  --save "${SAVE_DIR}/${ODOM_TYPE}_$(echo "$BAG_NAME" | cut -d'_' -f1-2).png"

