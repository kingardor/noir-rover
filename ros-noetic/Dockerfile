FROM ros:noetic-ros-base

# Install necessary system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev \
    python3-pip \
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh

# Use the entrypoint script to start all necessary services
ENTRYPOINT ["./entrypoint.sh"]