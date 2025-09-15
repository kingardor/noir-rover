FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      libevdev2 \
      openssh-client \
      python3-dev \
      python3-pip \
      python3-setuptools \
      ros-noetic-rosbridge-server \
      ros-noetic-tf2-web-republisher \
      sshpass \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --no-build-isolation \
    evdev==1.6.1 \
    fastapi==0.70.0 \
    uvicorn==0.16.0 \
    "pydantic<2" \
    roslibpy \
    requests \
    cryptography==3.3.2 \
    pyOpenSSL==20.0.1 \
    pillow \
    opencv-python-headless \
    numpy \
    gradio \
    redis

# Catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
COPY roller_eye /catkin_ws/src/roller_eye

RUN bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

WORKDIR /app
COPY scoutros.py /app/scoutros.py
COPY xbox.py /app/xbox.py
COPY bridge-api.py /app/bridge-api.py
COPY noir.py /app/noir.py
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENTRYPOINT ["/app/entrypoint.sh"]
