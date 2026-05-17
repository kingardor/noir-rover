#!/usr/bin/env python3
"""
Test camera feed — noir_env, no robot required.

Reads frames from a recorded MP4 (TEST_VIDEO env var, default
recordings/test_feed.mp4) and writes them to Redis at TEST_FPS.
Loops the video on end-of-file so the pipeline runs continuously.

Yields immediately to real bridge frames: if camera:ts was updated
recently by the real bridge, skips writing for that cycle.

Usage:
    REDIS_URL=redis://localhost:6380 python -u vision/test_feed.py
"""

import base64
import os
import time
from pathlib import Path

import cv2
import redis as redis_lib

# ── Config ─────────────────────────────────────────────────────────────────────
REDIS_URL  = os.getenv("REDIS_URL",   "redis://localhost:6380")
FPS        = float(os.getenv("TEST_FPS", "5"))
_ROOT      = Path(__file__).parent.parent
TEST_VIDEO = os.getenv("TEST_VIDEO", str(_ROOT / "recordings" / "test_feed.mp4"))


def main():
    r     = redis_lib.from_url(REDIS_URL, decode_responses=False)
    r_str = redis_lib.from_url(REDIS_URL, decode_responses=True)

    interval = 1.0 / FPS

    cap = cv2.VideoCapture(TEST_VIDEO)
    if not cap.isOpened():
        raise RuntimeError(f"[test-feed] cannot open video: {TEST_VIDEO}")

    print(f"[test-feed] {TEST_VIDEO}  →  camera:frame @ {FPS:.0f} fps  (Redis {REDIS_URL})", flush=True)

    frame_no = 0
    while True:
        t0 = time.time()

        # Yield to real bridge: if camera:ts is fresh, let the bridge own the key.
        real_ts = r_str.get("camera:ts")
        if real_ts:
            try:
                if (t0 - float(real_ts)) < interval * 2:
                    time.sleep(interval)
                    continue
            except ValueError:
                pass

        ret, frame = cap.read()
        if not ret:
            # End of file — loop back to start.
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret:
                print("[test-feed] video unreadable after seek — stopping", flush=True)
                break

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            continue

        now = time.time()
        b64 = base64.b64encode(buf.tobytes())
        r.set("camera:frame", b64, ex=5)
        r_str.set("camera:ts", str(now), ex=5)
        frame_no += 1

        if frame_no % (int(FPS) * 10) == 0:
            print(f"[test-feed] frame {frame_no:06d}  {time.strftime('%H:%M:%S')}", flush=True)

        elapsed = time.time() - t0
        time.sleep(max(0.0, interval - elapsed))

    cap.release()


if __name__ == "__main__":
    main()
