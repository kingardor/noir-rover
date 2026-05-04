"""
Memory writer — subscribes to vision:events pubsub, debounces per-label,
and appends to the Redis stream memory:events.
Run as a daemon thread from vision/app.py.
"""
import json
import threading
import time

import redis


DEBOUNCE_S = 10.0   # min seconds between memory writes per unique label
MIN_CONF   = 0.45   # ignore detections below this confidence


def _run(redis_url: str):
    r = redis.from_url(redis_url, decode_responses=True)
    pubsub = r.pubsub()
    pubsub.subscribe("vision:events")
    last_seen: dict[str, float] = {}

    for message in pubsub.listen():
        if message["type"] != "message":
            continue
        frame_id = message["data"]
        try:
            raw = r.get("vision:latest")
            if not raw:
                continue
            payload = json.loads(raw)
            ts = payload.get("ts", time.time())
            for det in payload.get("detections", []):
                label = det.get("label", "")
                conf  = det.get("conf", 0.0)
                if conf < MIN_CONF:
                    continue
                now = time.time()
                if label in last_seen and (now - last_seen[label]) < DEBOUNCE_S:
                    continue
                last_seen[label] = now
                r.xadd("memory:events", {
                    "ts":       str(ts),
                    "label":    label,
                    "conf":     str(conf),
                    "thumb_id": frame_id,
                    "bbox":     json.dumps(det.get("bbox", [])),
                })
        except Exception as e:
            print(f"[memory] {e}", flush=True)


def start(redis_url: str = "redis://localhost:6379") -> threading.Thread:
    t = threading.Thread(target=_run, args=(redis_url,), daemon=True, name="memory-writer")
    t.start()
    return t
