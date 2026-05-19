
#---------------CONFIGURATION---------------

WORKSPACE_PATH=~/ros2_ws_host
BAG_NAME="Tetragon_4m"
KOBUKI=false
MODE="temporal"                         # pose-to-pose error calculation, other mode is spatial
SAVE_DIR="~/Desktop/temp"
SAVE_PREFIX="pokus"

#-------------------------------------------

source "$WORKSPACE_PATH/start_scripts/venv/bin/activate"
python3 "$WORKSPACE_PATH/start_scripts/run_trajectory_eval.py" \
  --workspace   "$WORKSPACE_PATH" \
  --bag-name    "$BAG_NAME" \
  --kobuki      "$KOBUKI" \
  --mode        "$MODE" \
  --save-dir    "$SAVE_DIR" \
  --save-prefix "$SAVE_PREFIX"