#!/usr/bin/env python3
"""
VLM live scene description — native macOS.

Reads frames from Redis (camera:frame written by bridge).
Runs one inference every VLM_INTERVAL seconds on the most recent camera frame.
If inference takes longer than VLM_INTERVAL the sleep is skipped so the next
cycle starts immediately — no queue builds up.  While waiting, newly
arrived frames are polled at 0.1 s so the freshest frame is always used.

Redis keys written:
  vlm:latest  JSON {text, ts, frame_id}  TTL 30s
"""
import base64
import io
import json
import os
import time

import redis as redis_lib
import requests
from PIL import Image

REDIS_URL    = os.getenv("REDIS_URL",    "redis://localhost:6380")
MLX_VLM_URL  = os.getenv("MLX_VLM_URL",  "http://localhost:8000")
VLM_MODEL    = os.getenv("VLM_MODEL",    "mlx-community/Qwen3-VL-2B-Instruct-4bit")
VLM_INTERVAL = float(os.getenv("VLM_INTERVAL", "5.0"))
VLM_SIZE     = int(os.getenv("VLM_SIZE", "384"))

_PROMPT = (
    "Describe what you see in one short sentence. "
    "Mention objects, people, and the setting. Be concise. /no_think"
)


def _resize_b64(b64_jpeg: str, size: int) -> str:
    img = Image.open(io.BytesIO(base64.b64decode(b64_jpeg))).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _describe(b64_jpeg: str) -> str:
    b64 = _resize_b64(b64_jpeg, VLM_SIZE)
    data_url = f"data:image/jpeg;base64,{b64}"
    resp = requests.post(
        f"{MLX_VLM_URL}/v1/chat/completions",
        headers={"Content-Type": "application/json"},
        json={
            "model": VLM_MODEL,
            "messages": [{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": data_url}},
                {"type": "text", "text": _PROMPT},
            ]}],
            "temperature": 0.0,
            "max_tokens": 80,
            "stream": False,
        },
        timeout=30,
    )
    resp.raise_for_status()
    return (resp.json().get("choices") or [{}])[0].get("message", {}).get("content", "").strip()


def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    print(f"[vlm] model={VLM_MODEL}  size={VLM_SIZE}  interval={VLM_INTERVAL}s  server={MLX_VLM_URL}", flush=True)

    last_cam_ts = None

    while True:
        # Wait up to VLM_INTERVAL for a new camera frame, polling at 100 ms.
        deadline = time.monotonic() + VLM_INTERVAL
        b64 = None
        cam_ts = None
        while time.monotonic() < deadline:
            ts = r.get("camera:ts")
            if ts and ts != last_cam_ts:
                frame = r.get("camera:frame")
                if frame:
                    b64 = frame
                    cam_ts = ts
                    break
            time.sleep(0.1)

        if b64 is None:
            continue

        # Re-check: a newer frame may have arrived while we fetched
        ts2 = r.get("camera:ts")
        if ts2 and ts2 != cam_ts:
            frame2 = r.get("camera:frame")
            if frame2:
                b64 = frame2
                cam_ts = ts2

        last_cam_ts = cam_ts

        try:
            t0 = time.monotonic()
            text = _describe(b64)
            elapsed = time.monotonic() - t0
            if text:
                print(f"[vlm] {elapsed:.2f}s  {text[:100]}", flush=True)
                r.set(
                    "vlm:latest",
                    json.dumps({"text": text, "ts": time.time(), "frame_id": cam_ts}),
                    ex=30,
                )
        except Exception as e:
            print(f"[vlm] error: {e}", flush=True)


if __name__ == "__main__":
    main()
