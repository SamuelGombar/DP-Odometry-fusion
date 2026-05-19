#!/usr/bin/env bash
# Creates the Python venv for trajectory evaluation scripts.
# Run once after cloning the repository.
#
# Usage:
#   chmod +x setup_venv.sh
#   ./setup_venv.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v python3 &>/dev/null; then
  echo "ERROR: python3 not found." >&2
  exit 1
fi

# ROS must be sourced so the venv can inherit rosbag2_py / rclpy
if [[ -z "$AMENT_PREFIX_PATH" ]]; then
  echo "ERROR: ROS 2 is not sourced. Run 'source /opt/ros/<distro>/setup.bash' first." >&2
  exit 1
fi

echo "Creating venv at $SCRIPT_DIR/venv ..."
python3 -m venv --system-site-packages "$SCRIPT_DIR/venv"

echo "Installing dependencies..."
"$SCRIPT_DIR/venv/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"

echo "Done. Venv is ready at $SCRIPT_DIR/venv"
