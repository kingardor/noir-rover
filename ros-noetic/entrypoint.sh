#!/bin/bash

# Source ROS
source /opt/ros/noetic/setup.bash

# Source our custom workspace
source /catkin_ws/devel/setup.bash

echo "Starting ROS bridge with minimal roller_eye support..."
echo "Available roller_eye messages:"
rosmsg list | grep roller_eye
echo "Available roller_eye services:"
rossrv list | grep roller_eye

# Start rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch