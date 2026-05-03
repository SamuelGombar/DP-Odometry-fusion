BAG_NAME=Candy_7m_0075_regul_true_4
KOBUKI=false
MODE=temporal
SAVE_DIR=/home/samuelg9/Desktop/kin_revisited   #/home/samuelg9/Documents/Skola/DP/latex/img
GT_BAG=/home/samuelg9/ros2_ws_host/recordings/output/genz/Candy_7m
CUTOFF_TS=1777204630     #9999999999 for no cutoff 
SAVE_PREFIX="regul_" #append "_" at the end if not empty

echo "Select odometry pipeline:"
echo "  1) CSM"
echo "  2) GenZ-ICP + EKF"
echo "  3) Kinematic ICP"
echo "  4) Pure CSM"
read -rp "Enter choice [1-4]: " ODOM_CHOICE

case "$ODOM_CHOICE" in
  1) ODOM_TYPE="csm";      ODOM_TOPIC="/fusion_odometry";              EST_LABEL='P$_L$ICP';          COLORMAP="copper" ;;      # /odom_icp
  2) ODOM_TYPE="genz"; ODOM_TOPIC="/odometry/filtered";           EST_LABEL="GenZ-ICP";     COLORMAP="plasma" ;;     #/genz/odometry
  3) ODOM_TYPE="kin";      ODOM_TOPIC="/kinematic_icp/lidar_odometry"; EST_LABEL="Kinematic ICP"; COLORMAP="viridis" ;;
  4) ODOM_TYPE="csm"; ODOM_TOPIC="/odom_icp";                    EST_LABEL='pôvodný P$_L$ICP';     COLORMAP="cividis";    SAVE_PREFIX="pure_" ;;
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
    --mode ${MODE} \
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
    --gt-bag ${GT_BAG} \
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
  --colormap "$COLORMAP" \
  --save "${SAVE_DIR}/${SAVE_PREFIX}${ODOM_TYPE}_$(echo "$BAG_NAME" | cut -d'_' -f1-2)_${MODE}.png" \
  # --cutoff-timestamp "$CUTOFF_TS" \


# MODE=spatial
# if [ "$KOBUKI" = "true" ]; then
#   python3 /home/samuelg9/ros2_ws_host/trajectory_eval.py \
#     /home/samuelg9/ros2_ws_host/recordings/output/kobuki/${ODOM_TYPE}/${BAG_NAME} \
#     --odom-topic ${ODOM_TOPIC} \
#     --align \
#     --kobuki \
#     --mode ${MODE} \
#     --output-csv /home/samuelg9/ros2_ws_host/results.csv \
#     --interpolate-gt 0.5 
# else
#   python3 /home/samuelg9/ros2_ws_host/trajectory_eval.py \
#     /home/samuelg9/ros2_ws_host/recordings/output/${ODOM_TYPE}/${BAG_NAME} \
#     --odom-topic ${ODOM_TOPIC} \
#     --output-csv /home/samuelg9/ros2_ws_host/results.csv \
#     --align \
#     --mode ${MODE} \
#     --cutoff-timestamp "$CUTOFF_TS" \
#     --timestamp-offset "$TIMESTAMP_OFFSET" \
#     # --gt-bag ${GT_BAG} \
#     #-6.5
#     #  --hybrid-fraction 1 0.09    #0 - temporal first     
# fi
# mkdir -p "$SAVE_DIR"
# python3 /home/samuelg9/ros2_ws_host/plot_trajectory_eval.py /home/samuelg9/ros2_ws_host/results.csv \
#   --separate \
#   --match-lines \
#   --rotate -180 \
#   --traj-title "$TRAJ_TITLE" \
#   --est-label "$EST_LABEL" \
#   --colormap "$COLORMAP" \
#   --save "${SAVE_DIR}/pure_${ODOM_TYPE}_$(echo "$BAG_NAME" | cut -d'_' -f1-2)_${MODE}.png" \
#   # --cutoff-timestamp "$CUTOFF_TS" \
