BAG_NAME=opti_structured_current_conf
KOBUKI=true
MODE=temporal
SAVE_DIR=/home/samuelg9/Desktop/kobuki_znova   #/home/samuelg9/Documents/Skola/DP/latex/img
GT_BAG=/home/samuelg9/ros2_ws_host/recordings/optitrack/opti_structured
# CUTOFF_TS=1777213240     #9999999999 for no cutoff 
SAVE_PREFIX="pokus_" #append "_" at the end if not empty
CSV_NAME="pokus"

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
  4) ODOM_TYPE="csm"; ODOM_TOPIC="/odom_icp";                    EST_LABEL='pôvodný P$_L$ICP';     COLORMAP="cividis" ;;
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
    --align \
    --kobuki \
    --mode ${MODE} \
    --output-csv /home/samuelg9/ros2_ws_host/${CSV_NAME}.csv \
    --odom-topic ${ODOM_TOPIC} \
    --timestamp-offset 0.0 \

else
  python3 /home/samuelg9/ros2_ws_host/trajectory_eval.py \
    /home/samuelg9/ros2_ws_host/recordings/output/${ODOM_TYPE}/${BAG_NAME} \
    --align \
    --odom-topic ${ODOM_TOPIC} \
    --output-csv /home/samuelg9/ros2_ws_host/${CSV_NAME}.csv \
    --mode ${MODE} \
    --cutoff-timestamp "$CUTOFF_TS" \
    --gt-bag ${GT_BAG} \
    #-6.5
    #  --hybrid-fraction 1 0.09    #0 - temporal first     
fi
mkdir -p "$SAVE_DIR"
python3 /home/samuelg9/ros2_ws_host/plot_trajectory_eval.py /home/samuelg9/ros2_ws_host/${CSV_NAME}.csv \
  --separate \
  --rotate -180 \
  --traj-title "$TRAJ_TITLE" \
  --est-label "$EST_LABEL" \
  --colormap "$COLORMAP" \
  --match-lines \
  --save "${SAVE_DIR}/${SAVE_PREFIX}${ODOM_TYPE}_$(echo "$BAG_NAME" | cut -d'_' -f1-2)_${MODE}.png" \

# python3 plot_error_over_time.py bag1.csv bag2.csv bag3.csv \
#     --labels "CSM ICP" "GenZ-ICP" "EKF" \
#     --title "Chyba polôh v čase" \
#     --save comparison.png