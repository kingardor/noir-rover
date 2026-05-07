#!/usr/bin/env bash
# Start the ROS bridge API natively via RoboStack micromamba.
# Called by Tiltfile and Makefile.
set -euo pipefail
cd "$(dirname "$0")/.."

# Kill any process already holding :8012 so uvicorn can bind cleanly.
lsof -ti:8012 | xargs kill -9 2>/dev/null || true

exec /opt/homebrew/opt/micromamba/bin/micromamba run -n ros_env bash -c "
  source $(pwd)/catkin_ws/devel/setup.bash
  cd $(pwd)/ros-noetic
  ROS_MASTER_URI=${ROS_MASTER_URI:-http://10.42.0.1:11311} \
  ROS_IP=${ROS_IP:-10.42.0.181} \
  REDIS_URL=${REDIS_URL:-redis://localhost:6380} \
  python -m uvicorn bridge-api:app --host 0.0.0.0 --port 8012 --log-level info
"
