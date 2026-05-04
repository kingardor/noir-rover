#!/usr/bin/env bash
set -Eeuo pipefail

# ─────────── Config ───────────
: "${REMOTE_HOST:=10.42.0.1}"
: "${REMOTE_USER:=linaro}"
: "${REMOTE_PASS:=linaro}"
: "${REMOTE_NODE:=/NavPathNode}"

: "${ROSBRIDGE_ADDRESS:=0.0.0.0}"
: "${ROSBRIDGE_PORT:=9090}"

# API
: "${API_HOST:=0.0.0.0}"
: "${PORT:=8011}"
: "${API_WORKERS:=1}"
: "${API_LOG_LEVEL:=info}"

# Rosbridge WebSocket
: "${RUN_ROSBRIDGE:=1}"            # set to 0 to disable (not needed for HTTP agent mode)

# ─────────── ROS env ───────────
source /opt/ros/noetic/setup.bash
[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash

echo "[entrypoint] ROS env ready"
echo "[entrypoint] ROS_MASTER_URI  : ${ROS_MASTER_URI:-"(unset - roslaunch will start roscore)"}"

echo "[entrypoint] Available roller_eye messages:" && (rosmsg list | grep -F "roller_eye" || true)

# ─────────── Best-effort remote kill ───────────
if command -v sshpass >/dev/null 2>&1; then
  sshpass -p "${REMOTE_PASS}" \
    ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o ConnectTimeout=3 -o BatchMode=no -o ConnectionAttempts=1 \
        "${REMOTE_USER}@${REMOTE_HOST}" \
        'bash -lc "source /opt/ros/noetic/setup.bash || true; (rosnode kill '"${REMOTE_NODE}"' || true)"' \
    || echo "[entrypoint] Remote kill failed/skipped."
fi

# ─────────── Signal handling ───────────
pids=()
cleanup() {
  echo "[entrypoint] stopping children..."
  for pid in "${pids[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup SIGTERM SIGINT

# ─────────── Start rosbridge (optional) ───────────
if [ "${RUN_ROSBRIDGE}" = "1" ]; then
  echo "[entrypoint] rosbridge_websocket @ ${ROSBRIDGE_ADDRESS}:${ROSBRIDGE_PORT}"
  roslaunch rosbridge_server rosbridge_websocket.launch \
    address:="${ROSBRIDGE_ADDRESS}" port:="${ROSBRIDGE_PORT}" &
  pids+=($!)
  echo "[entrypoint] rosbridge pid=${pids[-1]}"
else
  echo "[entrypoint] RUN_ROSBRIDGE=0; skipping rosbridge."
fi

# ─────────── Start FastAPI ───────────
echo "[entrypoint] FastAPI @ ${API_HOST}:${PORT}"
python3 -m uvicorn bridge-api:app --host "${API_HOST}" --port "${PORT}" \
  --workers "${API_WORKERS}" --log-level "${API_LOG_LEVEL}" &
pids+=($!)
echo "[entrypoint] uvicorn pid=${pids[-1]}"

# ─────────── Wait for any to exit ───────────
set +e
wait -n "${pids[@]}"
code=$?
cleanup
echo "[entrypoint] exiting with code ${code}"
exit "${code}"