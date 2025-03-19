FROM nvidia/cuda:12.5.1-cudnn-devel-ubuntu22.04 AS noir-asr

# Add open GL libraries
RUN apt-get update && \
        DEBIAN_FRONTEND=noninteractive  apt-get install -y --no-install-recommends \
        python3-dev \
        python3-pip \
        python3-setuptools \
        alsa-base \
        alsa-utils \
        pulseaudio \
        libportaudio2 \
        libasound-dev \
        libportaudiocpp0 \
        portaudio19-dev \
        ffmpeg \
        git

# Cleanup
RUN rm -rf /var/lib/apt/lists/* && \
apt autoremove

RUN pip3 install --no-cache-dir \
    git+https://github.com/openai/whisper.git \
    openai \
    sounddevice \
    scipy \
    redis

RUN update-ca-certificates

# To get video driver libraries at runtime (libnvidia-encode.so/libnvcuvid.so)
ENV NVIDIA_DRIVER_CAPABILITIES $NVIDIA_DRIVER_CAPABILITIES,video,compute,graphics,utility

ENV CUDA_HOME=/usr/local/cuda
ENV CFLAGS="-I$CUDA_HOME/include $CFLAGS"
ENV RUNNING_IN_DOCKER true

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

ENV PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native

WORKDIR /app

COPY models/asr /app/models
COPY core /app/core
COPY noir-asr.py /app/noir-asr.py

CMD ["python3", "veronica-asr.py"]