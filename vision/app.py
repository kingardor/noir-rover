#!/usr/bin/env python3
"""
YOLOE continuous detection loop — native macOS, Metal/MPS.

Reads frames from Redis (camera:frame written by bridge),
runs YOLOE zero-shot detection, publishes to Redis:
  vision:latest              JSON {frame_id, ts, detections, frame_w, frame_h}  TTL 10s
  vision:thumb:{frame_id}    base64 JPEG                                         TTL 60s
  vision:events  (pubsub)    frame_id string

Also starts the memory writer thread (vision/memory.py).
"""
import base64
import json
import os
import sys
import time

import cv2
import numpy as np
import redis
from PIL import Image
from ultralytics import YOLOE

# ── Config ────────────────────────────────────────────────────────────────────

_HERE      = os.path.dirname(os.path.abspath(__file__))
_ROOT      = os.path.join(_HERE, "..")

REDIS_URL      = os.getenv("REDIS_URL",     "redis://localhost:6379")
MODEL_PATH     = os.getenv("YOLOE_MODEL",   os.path.join(_ROOT, "ai", "yoloe-11m-seg.pt"))
FRAME_INTERVAL = float(os.getenv("VISION_INTERVAL", "2.0"))   # seconds between runs
INFER_W        = int(os.getenv("INFER_W",   "640"))
INFER_H        = int(os.getenv("INFER_H",   "384"))
LABELS         = os.getenv(
    "YOLOE_LABELS",
    "person,spectacles,tattoos,bottle,cup,mug,chair,laptop,phone,backpack,dog,cat,book,keyboard,remote,vase"
).split(",")

# ── Device ────────────────────────────────────────────────────────────────────

def _best_device() -> str:
    import torch
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    # Start memory writer thread
    sys.path.insert(0, _HERE)
    from memory import start as start_memory
    start_memory(REDIS_URL)

    device = _best_device()
    print(f"[vision] device={device}  model={MODEL_PATH}", flush=True)
    print(f"[vision] labels={LABELS}", flush=True)

    model = YOLOE(MODEL_PATH)
    # Compute text PE on CPU first: mobileclip_blt.ts has float64 ops that MPS can't handle
    text_pe = model.get_text_pe(LABELS)
    try:
        model.to(device)
        text_pe = text_pe.to(device)
    except Exception as e:
        print(f"[vision] MPS move failed ({e}), running on CPU", flush=True)
        device = "cpu"
    model.set_classes(LABELS, text_pe)
    print("[vision] Model ready.", flush=True)

    r = redis.from_url(REDIS_URL, decode_responses=True)
    frame_count = 0
    last_cam_ts = None

    while True:
        t_start = time.time()
        try:
            # Wait for a new camera frame (poll camera:ts at 100 ms)
            deadline = t_start + FRAME_INTERVAL
            cam_ts_raw = None
            while time.time() < deadline:
                ts = r.get("camera:ts")
                if ts and ts != last_cam_ts:
                    cam_ts_raw = ts
                    break
                time.sleep(0.1)

            if cam_ts_raw is None:
                continue   # no new frame within interval; loop immediately

            b64 = r.get("camera:frame")
            if not b64:
                continue
            last_cam_ts = cam_ts_raw

            jpg_bytes = base64.b64decode(b64)

            # Decode + resize for inference
            nparr    = np.frombuffer(jpg_bytes, np.uint8)
            frame_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                continue
            frame_small = cv2.resize(frame_bgr, (INFER_W, INFER_H))
            pil_img = Image.fromarray(cv2.cvtColor(frame_small, cv2.COLOR_BGR2RGB))

            # Inference
            results = model.predict(pil_img, verbose=False)

            # Parse boxes
            detections: list[dict] = []
            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                names = model.names  # updated after set_classes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf   = float(box.conf[0])
                    xyxy   = box.xyxy[0].tolist()
                    label  = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else (
                        LABELS[cls_id] if cls_id < len(LABELS) else str(cls_id)
                    )
                    detections.append({
                        "label":  label,
                        "conf":   round(conf, 3),
                        "bbox":   [round(v, 1) for v in xyxy],
                        "cls_id": cls_id,
                    })
            detections.sort(key=lambda d: d["conf"], reverse=True)

            frame_id = f"f{frame_count}_{int(t_start * 1000)}"
            frame_count += 1

            # Publish to Redis
            payload = {
                "frame_id":   frame_id,
                "ts":         t_start,
                "detections": detections,
                "frame_w":    INFER_W,
                "frame_h":    INFER_H,
            }
            r.set("vision:latest", json.dumps(payload), ex=10)
            r.set(f"vision:thumb:{frame_id}", b64, ex=60)
            r.publish("vision:events", frame_id)

            if frame_count % 30 == 0:
                dt = time.time() - t_start
                labels_seen = ", ".join(d["label"] for d in detections[:5]) or "none"
                print(
                    f"[vision] frame={frame_count}  dt={dt*1000:.0f}ms"
                    f"  detections={len(detections)} [{labels_seen}]",
                    flush=True,
                )

        except Exception as e:
            print(f"[vision] error: {e}", flush=True)
            time.sleep(0.5)


if __name__ == "__main__":
    main()
