#!/usr/bin/env python3
"""
VLM live scene description — native macOS.

Runs one inference every VLM_INTERVAL seconds on the most recent camera frame.
If inference takes longer than VLM_INTERVAL the sleep is skipped so the next
cycle starts immediately — no queue builds up.  While inference runs, newly
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

REDIS_URL    = os.getenv("REDIS_URL",  "redis://localhost:6380")
OLLAMA_URL   = os.getenv("OLLAMA_URL", "http://localhost:11434")
VLM_MODEL    = os.getenv("VLM_MODEL",  "qwen3-vl:2b-instruct")
VLM_INTERVAL = float(os.getenv("VLM_INTERVAL", "2.0"))
VLM_SIZE     = int(os.getenv("VLM_SIZE", "384"))

_PROMPT = (
    "Describe what you see in one short sentence. "
    "Mention objects, people, and the setting. Be concise. /no_think"
)


def _current_frame_id(r: redis_lib.Redis) -> str | None:
    raw = r.get("vision:latest")
    if not raw:
        return None
    return json.loads(raw).get("frame_id")


def _fetch_frame(r: redis_lib.Redis, frame_id: str) -> str | None:
    return r.get(f"vision:thumb:{frame_id}")


def _resize_b64(b64_jpeg: str, size: int) -> str:
    img = Image.open(io.BytesIO(base64.b64decode(b64_jpeg))).convert("RGB")
    img = img.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return base64.b64encode(buf.getvalue()).decode()


def _describe(b64_jpeg: str) -> str:
    b64 = _resize_b64(b64_jpeg, VLM_SIZE)
    resp = requests.post(
        f"{OLLAMA_URL}/api/generate",
        json={
            "model": VLM_MODEL,
            "prompt": _PROMPT,
            "images": [b64],
            "stream": False,
            "options": {"temperature": 0.0, "num_predict": 80},
        },
        timeout=30,
    )
    resp.raise_for_status()
    return resp.json().get("response", "").strip()


def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    print(f"[vlm] model={VLM_MODEL}  size={VLM_SIZE}  interval={VLM_INTERVAL}s", flush=True)

    last_frame_id = None

    while True:
        # Wait up to VLM_INTERVAL for a new frame, polling at 100 ms.
        # This means we always pick up the freshest frame, not one that was
        # current when inference finished.
        deadline = time.monotonic() + VLM_INTERVAL
        frame_id = None
        while time.monotonic() < deadline:
            fid = _current_frame_id(r)
            if fid and fid != last_frame_id:
                frame_id = fid
                break
            time.sleep(0.1)

        if frame_id is None:
            continue

        b64 = _fetch_frame(r, frame_id)
        if not b64:
            continue

        # Re-check: a newer frame may have arrived while we fetched the thumb.
        latest = _current_frame_id(r)
        if latest and latest != frame_id:
            b64 = _fetch_frame(r, latest) or b64
            frame_id = latest

        last_frame_id = frame_id

        try:
            t0 = time.monotonic()
            text = _describe(b64)
            elapsed = time.monotonic() - t0
            if text:
                print(f"[vlm] {elapsed:.2f}s  {text[:100]}", flush=True)
                r.set(
                    "vlm:latest",
                    json.dumps({"text": text, "ts": time.time(), "frame_id": frame_id}),
                    ex=30,
                )
        except Exception as e:
            print(f"[vlm] error: {e}", flush=True)


if __name__ == "__main__":
    main()
