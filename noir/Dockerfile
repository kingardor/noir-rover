FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libevdev2 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    evdev \
    requests

WORKDIR /app

COPY xbox.py .
COPY noir.py .
RUN chmod +x noir.py

CMD ["python3", "noir.py"]
