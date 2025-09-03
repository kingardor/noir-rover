FROM ros:noetic-ros-base

# Install only essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbridge-suite \
    ros-noetic-tf2-web-republisher \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy our minimal message package
COPY roller_eye /catkin_ws/src/roller_eye

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Setup workspace sourcing
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Copy and setup entrypoint
WORKDIR /app
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]