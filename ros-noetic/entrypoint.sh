#!/bin/bash

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "Starting ROS bridge with minimal roller_eye support..."
echo "Available roller_eye messages:"
rosmsg list | grep roller_eye
echo "Available roller_eye services:"
rossrv list | grep roller_eye

sshpass -p "linaro" ssh linaro@10.42.0.1 'bash -lc "source /opt/ros/noetic/setup.bash && (rosnode kill /NavPathNode || true)"'

roslaunch rosbridge_server rosbridge_websocket.launch