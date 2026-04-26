BAG_NAME=Ralf_7m_03
KOBUKI=false
MODE=temporal
SAVE_DIR=/home/samuelg9/Documents/Skola/DP/latex/img
CUTOFF_TS=1777213240     #9999999999 for no cutoff  
TIMESTAMP_OFFSET=-23644.927743
GT_BAG=genz/Ralf_4m_4_0075

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
    --mode ${MODE} \
    --cutoff-timestamp "$CUTOFF_TS" \
    --gt-bag /home/samuelg9/ros2_ws_host/recordings/output/${GT_BAG} \
    --timestamp-offset "$TIMESTAMP_OFFSET" \
    #-6.5
    #  --hybrid-fraction 1 0.09    #0 - temporal first     
fi
mkdir -p "$SAVE_DIR"
python3 /home/samuelg9/ros2_ws_host/plot_trajectory_eval.py /home/samuelg9/ros2_ws_host/results.csv \
  --separate \
  --match-lines \
  --rotate -180 \
  --traj-title "$TRAJ_TITLE" \
  --est-label "$EST_LABEL" \
  --save "${SAVE_DIR}/${ODOM_TYPE}_$(echo "$BAG_NAME" | cut -d'_' -f1-2)_${MODE}.png" \
  # --cutoff-timestamp "$CUTOFF_TS" \
