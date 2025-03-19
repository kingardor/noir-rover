FROM nvidia/cuda:12.4.0-devel-ubuntu22.04 AS noir

# Add open GL libraries
RUN apt-get update && \
        DEBIAN_FRONTEND=noninteractive \
        apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        libatomic1 \
        libgl1 \
        libnuma1 \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-tk \
        tree  \
        wget \
        && \
        rm -rf /var/lib/apt/lists/* && \
        apt autoremove

RUN pip3 install --no-cache-dir \
        opencv_python \
        customtkinter \
        redis

RUN pip3 install --no-cache-dir \
        langchain \
        langchain-community \
        langchain-huggingface \
        langchain-ollama \
        sentence-transformers \
        torch \
        transformers \
        openai

RUN update-ca-certificates

# To get video driver libraries at runtime (libnvidia-encode.so/libnvcuvid.so)
ENV NVIDIA_DRIVER_CAPABILITIES $NVIDIA_DRIVER_CAPABILITIES,video,compute,graphics,utility

ENV CUDA_HOME=/usr/local/cuda
ENV CFLAGS="-I$CUDA_HOME/include $CFLAGS"
ENV RUNNING_IN_DOCKER true

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

WORKDIR /app

COPY ollama-install.sh /app/ollama-install.sh
COPY ollama-pull.sh /app/ollama-pull.sh

RUN chmod +x ollama-install.sh && \
    ./ollama-install.sh && \
    chmod +x ollama-pull.sh && \
    ./ollama-pull.sh llama3.2:3b 

COPY hfembeddings-pull.sh /app/hfembeddings-pull.sh

RUN chmod +x hfembeddings-pull.sh && \
    ./hfembeddings-pull.sh

COPY fonts /usr/share/fonts
COPY core /app/core
COPY noir.py /app/noir.py
COPY noir.sh /app/noir.sh

RUN chmod +x noir.sh

EXPOSE 11434

CMD ["./noir.sh"]