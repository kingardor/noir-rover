#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash

echo "[ros] ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=$ROS_IP"

roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=9090 &
RB_PID=$!
echo "[ros] rosbridge started with PID $RB_PID"

trap "kill -TERM $RB_PID || true" SIGINT SIGTERM
wait -n