#!/usr/bin/env python3
"""
Face recognition service — native macOS.

Uses InsightFace (buffalo_l: SCRFD detector + ArcFace embeddings).
Enrolled faces are read from FACES_DIR at startup — filename stem = person name.

Reads frames from Redis (camera:frame written by bridge).
Runs detection + recognition every FACEREC_INTERVAL seconds,
publishes to Redis face:latest.

Redis keys written:
  face:latest  JSON {faces:[{name,score,bbox},...], ts, frame_id, frame_w, frame_h}  TTL 10s
"""
import base64
import json
import os
import time
from io import BytesIO
from pathlib import Path

import numpy as np
import redis as redis_lib
from PIL import Image

REDIS_URL        = os.getenv("REDIS_URL",        "redis://localhost:6380")
FACES_DIR        = os.getenv("FACES_DIR",        "faces")
FACEREC_INTERVAL = float(os.getenv("FACEREC_INTERVAL", "2.0"))
MATCH_THRESHOLD  = float(os.getenv("FACEREC_THRESHOLD", "0.35"))
DET_SIZE         = (640, 640)


def _cos(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-8))


def _load_db(app, faces_dir: str) -> dict[str, np.ndarray]:
    db: dict[str, np.ndarray] = {}
    root = Path(faces_dir)
    if not root.exists():
        print(f"[facerec] faces dir not found: {faces_dir}", flush=True)
        return db
    for img_path in sorted(root.glob("*.jpg")) + sorted(root.glob("*.png")):  # type: ignore[operator]
        name = img_path.stem
        try:
            img = np.array(Image.open(img_path).convert("RGB"))
            faces = app.get(img)
            if not faces:
                print(f"[facerec] no face found in {img_path.name}", flush=True)
                continue
            embd = faces[0].normed_embedding
            db[name.upper()] = embd
            print(f"[facerec] enrolled: {name}", flush=True)
        except Exception as e:
            print(f"[facerec] enrollment error ({img_path.name}): {e}", flush=True)
    return db


def _match(db: dict[str, np.ndarray], embd: np.ndarray) -> tuple[str, float]:
    best_name, best_score = "UNKNOWN", 0.0
    for name, ref in db.items():
        score = _cos(embd, ref)
        if score > best_score:
            best_score, best_name = score, name
    return best_name, best_score


def _decode_frame(b64_jpeg: str) -> np.ndarray:
    img_bytes = base64.b64decode(b64_jpeg)
    img = Image.open(BytesIO(img_bytes)).convert("RGB")
    return np.array(img)


def main():
    import insightface
    from insightface.app import FaceAnalysis

    app = FaceAnalysis(name="buffalo_l", providers=["CPUExecutionProvider"])
    app.prepare(ctx_id=0, det_size=DET_SIZE)
    print("[facerec] InsightFace loaded", flush=True)

    db = _load_db(app, FACES_DIR)
    print(f"[facerec] {len(db)} face(s) enrolled: {list(db)}", flush=True)

    r = redis_lib.from_url(REDIS_URL, decode_responses=True)
    last_cam_ts = None

    while True:
        try:
            cam_ts = r.get("camera:ts")
            if not cam_ts or cam_ts == last_cam_ts:
                time.sleep(FACEREC_INTERVAL)
                continue

            b64 = r.get("camera:frame")
            if not b64:
                time.sleep(FACEREC_INTERVAL)
                continue

            last_cam_ts = cam_ts
            img = _decode_frame(b64)
            h, w = img.shape[:2]
            faces = app.get(img)

            results = []
            for face in faces:
                name, score = _match(db, face.normed_embedding)
                if score < MATCH_THRESHOLD:
                    name = "UNKNOWN"
                bbox = [int(v) for v in face.bbox.tolist()]
                results.append({"name": name, "score": round(score, 3), "bbox": bbox})
                print(f"[facerec] {name} ({score:.2f})", flush=True)

            r.set(
                "face:latest",
                json.dumps({
                    "faces":   results,
                    "ts":      time.time(),
                    "frame_id": cam_ts,
                    "frame_w": w,
                    "frame_h": h,
                }),
                ex=10,
            )
        except Exception as e:
            print(f"[facerec] error: {e}", flush=True)

        time.sleep(FACEREC_INTERVAL)


if __name__ == "__main__":
    main()
