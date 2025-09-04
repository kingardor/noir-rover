FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libevdev2 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    evdev==1.6.1 \
    requests \
    pillow \
    opencv-python-headless \
    streamlit \
    numpy

WORKDIR /app

COPY xbox.py .
COPY noir.py .
RUN chmod +x noir.py

CMD ["streamlit", "run", "noir.py", "--server.port", "8501", "--server.address", "0.0.0.0"]