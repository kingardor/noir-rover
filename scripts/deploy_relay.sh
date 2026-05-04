#!/usr/bin/env bash
# Deploy cmd_vel_relay.py to the robot and (re)start it under systemd.
# Idempotent — safe to re-run on every code change.
#
# Usage:
#   bash scripts/deploy_relay.sh
#
# Override defaults via environment:
#   ROBOT_HOST=10.42.0.1 ROBOT_USER=linaro ROBOT_PASS=linaro bash scripts/deploy_relay.sh

set -Eeuo pipefail

ROBOT_HOST="${ROBOT_HOST:-10.42.0.1}"
ROBOT_USER="${ROBOT_USER:-linaro}"
ROBOT_PASS="${ROBOT_PASS:-linaro}"
RELAY_DIR="/home/${ROBOT_USER}/noir-relay"
SERVICE_NAME="noir-relay"

SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5"

_ssh() { sshpass -p "$ROBOT_PASS" ssh $SSH_OPTS "${ROBOT_USER}@${ROBOT_HOST}" "$@"; }
_scp() { sshpass -p "$ROBOT_PASS" scp $SSH_OPTS "$@"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ── Step 0: verify prerequisites ─────────────────────────────────────────────
echo "[deploy] Waiting for robot (${ROBOT_HOST}) — power it on if not already..."
until _ssh 'echo ok' 2>/dev/null; do
  echo "[deploy] Robot unreachable — retrying in 5 s..."
  sleep 5
done
echo "[deploy] Connected to ${ROBOT_HOST}"

echo "[deploy] Checking Python 2 + rospy on robot (ROS Melodic)..."
_ssh 'source /opt/ros/melodic/setup.bash && python2 -c "import rospy; print(\"rospy ok\")"' || {
  echo "[deploy] ERROR: rospy not importable — check /opt/ros/melodic on robot"
  exit 1
}

if _ssh 'ss -lun 2>/dev/null | grep -q ":9999"'; then
  echo "[deploy] Port 9999 already bound — will restart service to take over"
fi

# ── Step 1: copy files ────────────────────────────────────────────────────────
echo "[deploy] Creating ${RELAY_DIR}..."
_ssh "sudo mkdir -p ${RELAY_DIR} && sudo chown ${ROBOT_USER}: ${RELAY_DIR}"

echo "[deploy] Copying relay files..."
_scp "${REPO_ROOT}/ros-noetic/cmd_vel_relay.py"  "${ROBOT_USER}@${ROBOT_HOST}:${RELAY_DIR}/"
_scp "${REPO_ROOT}/ros-noetic/relay_protocol.py"  "${ROBOT_USER}@${ROBOT_HOST}:${RELAY_DIR}/"
_ssh "chmod +x ${RELAY_DIR}/cmd_vel_relay.py"

# ── Step 2: install/update systemd unit ───────────────────────────────────────
echo "[deploy] Installing systemd unit..."

# Write unit file — heredoc passed via stdin to avoid quoting nightmares
_ssh "sudo tee /etc/systemd/system/${SERVICE_NAME}.service > /dev/null" <<UNIT
[Unit]
Description=Noir cmd_vel UDP relay
After=network.target

[Service]
Type=simple
User=${ROBOT_USER}
WorkingDirectory=${RELAY_DIR}
ExecStart=/bin/bash -c 'source /opt/ros/melodic/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11311 python2 ${RELAY_DIR}/cmd_vel_relay.py'
Restart=always
RestartSec=3
StandardOutput=append:/var/log/noir-relay.log
StandardError=append:/var/log/noir-relay.log

[Install]
WantedBy=multi-user.target
UNIT

_ssh "sudo systemctl daemon-reload"
_ssh "sudo systemctl enable ${SERVICE_NAME}"
_ssh "sudo systemctl restart ${SERVICE_NAME}"

# ── Step 3: verify ────────────────────────────────────────────────────────────
echo "[deploy] Waiting for service to start..."
sleep 3

STATUS=$(_ssh "systemctl is-active ${SERVICE_NAME}" 2>/dev/null || echo "failed")
if [ "$STATUS" != "active" ]; then
  echo "[deploy] ERROR: service status=${STATUS}"
  echo "[deploy] Logs:"
  _ssh "sudo journalctl -u ${SERVICE_NAME} -n 20 --no-pager" || true
  exit 1
fi

echo ""
echo "[deploy] SUCCESS — noir-relay is active on ${ROBOT_HOST}:9999"
echo "[deploy] Verify publisher:  sshpass -p ${ROBOT_PASS} ssh ${ROBOT_USER}@${ROBOT_HOST} 'source /opt/ros/melodic/setup.bash && rostopic info /cmd_vel'"
echo "[deploy] Check logs:        sshpass -p ${ROBOT_PASS} ssh ${ROBOT_USER}@${ROBOT_HOST} 'tail -f /var/log/noir-relay.log'"
