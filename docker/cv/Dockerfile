FROM nvidia/cuda:12.5.1-cudnn-devel-ubuntu22.04 AS noir-cv

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
        libgl1

# Cleanup
RUN rm -rf /var/lib/apt/lists/* && \
apt autoremove

RUN pip3 install --no-cache-dir \
    packaging \
    torch \
    torchvision \
    opencv_python \
    pillow \
    onnx \
    onnxruntime-gpu \
    scikit-image \
    redis \
    pyvips \
    einops

RUN pip3 install  --no-cache-dir \
        transformers \
        accelerate
                
RUN update-ca-certificates

# To get video driver libraries at runtime (libnvidia-encode.so/libnvcuvid.so)
ENV NVIDIA_DRIVER_CAPABILITIES $NVIDIA_DRIVER_CAPABILITIES,video,compute,graphics,utility

ENV CUDA_HOME=/usr/local/cuda
ENV CFLAGS="-I$CUDA_HOME/include $CFLAGS"
ENV RUNNING_IN_DOCKER true

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
    
WORKDIR /app

COPY models/cv /app/models
COPY core /app/core
COPY faces /app/faces
COPY noir-cv.py /app/noir-cv.py

CMD ["python3", "noir-cv.py"]