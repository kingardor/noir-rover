#!/usr/bin/env python3
"""
VLM live scene description — native macOS.

Polls vision:latest every VLM_INTERVAL_S seconds.
Fetches the current frame thumbnail from Redis, sends to Ollama (qwen2.5vl or
compatible multimodal model), and stores the result in vlm:latest.

Redis keys written:
  vlm:latest  JSON {text, ts, frame_id}  TTL 30s
"""
import base64
import json
import os
import time

import redis as redis_lib
import requests

REDIS_URL    = os.getenv("REDIS_URL",    "redis://localhost:6380")
OLLAMA_URL   = os.getenv("OLLAMA_URL",   "http://localhost:11434")
VLM_MODEL    = os.getenv("VLM_MODEL",    "qwen2.5vl:3b")
VLM_INTERVAL = float(os.getenv("VLM_INTERVAL", "4.0"))

_PROMPT = (
    "Describe what you see in one short sentence. "
    "Mention objects, people, and the setting. Be concise."
)


def _latest_frame(r: redis_lib.Redis) -> tuple[str, str] | None:
    raw = r.get("vision:latest")
    if not raw:
        return None
    data = json.loads(raw)
    frame_id = data.get("frame_id")
    if not frame_id:
        return None
    b64 = r.get(f"vision:thumb:{frame_id}")
    if not b64:
        return None
    return frame_id, b64


def _describe(b64_jpeg: str) -> str:
    resp = requests.post(
        f"{OLLAMA_URL}/api/generate",
        json={
            "model": VLM_MODEL,
            "prompt": _PROMPT,
            "images": [b64_jpeg],
            "stream": False,
            "options": {"temperature": 0.1, "num_predict": 80},
        },
        timeout=20,
    )
    resp.raise_for_status()
    return resp.json().get("response", "").strip()


def main():
    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    print(f"[vlm] model={VLM_MODEL}  interval={VLM_INTERVAL}s", flush=True)

    last_frame_id = None

    while True:
        try:
            result = _latest_frame(r)
            if result is None:
                time.sleep(VLM_INTERVAL)
                continue

            frame_id, b64 = result
            if frame_id == last_frame_id:
                time.sleep(VLM_INTERVAL)
                continue

            last_frame_id = frame_id
            text = _describe(b64)
            if text:
                print(f"[vlm] {text[:100]}", flush=True)
                r.set(
                    "vlm:latest",
                    json.dumps({"text": text, "ts": time.time(), "frame_id": frame_id}),
                    ex=30,
                )
        except Exception as e:
            print(f"[vlm] error: {e}", flush=True)

        time.sleep(VLM_INTERVAL)


if __name__ == "__main__":
    main()
