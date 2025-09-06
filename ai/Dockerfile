FROM pytorch/pytorch:2.8.0-cuda12.9-cudnn9-runtime

# Add open GL libraries
RUN apt-get update && \
        DEBIAN_FRONTEND=noninteractive  apt-get install -y --no-install-recommends \
        pkg-config \
        wget \
        python3-dev \
        python3-pip \
        python3-setuptools \
        tree \
        git \
        libgl1 \
        libglib2.0-0 \
        libgtk-3-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        ffmpeg \
        && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN pip3 install --no-cache-dir --no-build-isolation \
    requests \
    "git+https://github.com/THU-MIG/yoloe.git#subdirectory=third_party/CLIP" \
    "git+https://github.com/THU-MIG/yoloe.git#subdirectory=third_party/ml-mobileclip" \
    "git+https://github.com/THU-MIG/yoloe.git#subdirectory=third_party/lvis-api" \
    "git+https://github.com/THU-MIG/yoloe.git"

COPY mobileclip_blt.pt /app/mobileclip_blt.pt
COPY yoloe-11l-seg.pt /app/yoloe-11l-seg.pt
COPY app.py /app/app.py

RUN update-ca-certificates

# To get video driver libraries at runtime (libnvidia-encode.so/libnvcuvid.so)
ENV NVIDIA_DRIVER_CAPABILITIES $NVIDIA_DRIVER_CAPABILITIES,video,compute,graphics,utility

ENV CUDA_HOME=/usr/local/cuda
ENV CFLAGS="-I$CUDA_HOME/include $CFLAGS"
ENV RUNNING_IN_DOCKER true

CMD ["python3", "app.py"]